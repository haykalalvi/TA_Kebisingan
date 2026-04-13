/**
 * =============================================================================
 * ESP32-S3 Audio Streaming + GPS + Calibration Firmware
 * =============================================================================
 */

#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <driver/i2s.h>
#include <Preferences.h>
#include <atomic>
#include "sos-iir-filter.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ─── I2C / LCD ─────────────────────────────────────────────────────────── */
#define I2C_SDA 4
#define I2C_SCL 5
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ─── AUDIO CONFIG ───────────────────────────────────────────────────────── */
#define SAMPLE_RATE       48000
#define SAMPLE_BITS       32
#define MIC_BITS          24
#define SAMPLES_SHORT     (SAMPLE_RATE / 8)   // 6000 samples per frame
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))

/* ─── I2S PINS ───────────────────────────────────────────────────────────── */
#define I2S_WS   9
#define I2S_SCK  39
#define I2S_SD   15
#define I2S_PORT I2S_NUM_0

/* ─── GPS ────────────────────────────────────────────────────────────────── */
#define RXD2     16
#define TXD2     17
#define GPS_BAUD 9600

/* ─── FRAMING MARKERS ────────────────────────────────────────────────────── */
#define FRAME_START_MARKER  0xAA55AA55
#define FRAME_END_MARKER    0x55AA55AA

/* ─── BUTTON PINS ────────────────────────────────────────────────────────── */
#define BTN_UP   3
#define BTN_DOWN 10

/* ─── TIMING ─────────────────────────────────────────────────────────────── */
#define HOLD_DURATION_MS   2000   // how long to hold both buttons to toggle mode
#define NVS_SAVE_DELAY_MS  8000   // minimum ms between NVS writes
#define SAVE_NOTIFY_MS     1500   // how long "Tersimpan!" stays on LCD

/* ─── MODE ENUM ──────────────────────────────────────────────────────────── */
// Using uint8_t as the underlying type so std::atomic<uint8_t> works on all
// ESP-IDF versions without needing lock-free guarantee for larger types.
enum class DeviceMode : uint8_t {
    STREAMING   = 0,
    CALIBRATING = 1
};

/* ═══════════════════════════════════════════════════════════════════════════
   GLOBALS
   ═══════════════════════════════════════════════════════════════════════════ */
HardwareSerial gpsSerial(2);
TinyGPSPlus    gps;
Preferences    preferences;

/* --- Shared state --------------------------------------------------------- */
// deviceMode:          written and read only by systemTask.
//                      audioDSPTask runs unconditionally regardless of mode;
//                      mode only gates USB send and button logic in systemTask.
// currentCalibration:  written by systemTask when buttons change the gain,
//                      packed into each USB packet header by systemTask,
//                      NEVER read by audioDSPTask — gain is applied on the Pi.
// Both use atomic to avoid needing a mutex for single-value reads/writes.
std::atomic<uint8_t> deviceMode(static_cast<uint8_t>(DeviceMode::STREAMING));
std::atomic<float>   currentCalibration(1.0f);

/* --- USB packet struct + its mutex --------------------------------------- */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    latitude;
    float    longitude;
    float    calibration_multiplier;   // proses ngaliin ke sampelnya dilakuin di raspi
    float    samples[SAMPLES_SHORT];
} usb_packet_t;

static usb_packet_t usb_packet;
static SemaphoreHandle_t packetMutex = nullptr;

/* --- Audio double-buffer + queue ----------------------------------------- */
// Two buffers: audioDSPTask writes into one while systemTask reads the other.
// The queue carries only the *index* (0 or 1) of the ready buffer.
static float audio_buffers[2][SAMPLES_SHORT] __attribute__((aligned(4)));
static QueueHandle_t audioQueue = nullptr;

/* --- Drop counter (diagnostic) ------------------------------------------- */
static std::atomic<uint32_t> droppedFrames(0);

/* ─── IIR FILTERS ────────────────────────────────────────────────────────── */
SOS_IIR_Filter DC_BLOCKER = {
    1.0,
    { {-1.0, 0.0, 0.9992, 0} }
};
SOS_IIR_Filter INMP441 = {
    1.001978346553777,
    { {-1.986920458359494, 0.986963226961561,
       1.995178510521705, -0.995184322211588} }
};

/* ═══════════════════════════════════════════════════════════════════════════
   TASK: audioDSPTask  (Core 1, highest priority)
   Responsibilities:
     - Read raw I2S samples from DMA
     - Apply DC block + microphone EQ
     - Push buffer index to audioQueue

   Cuma akuisisi data aja, pengalinya dipindah ke raspi
   ═══════════════════════════════════════════════════════════════════════════ */
void audioDSPTask(void *pvParameters) {
    int current_buffer = 0;

    while (true) {
        /* 1. Block until DMA has a full frame ready */
        size_t bytes_read = 0;
        i2s_read(I2S_PORT,
                 audio_buffers[current_buffer],
                 SAMPLES_SHORT * sizeof(int32_t),
                 &bytes_read,
                 portMAX_DELAY);

        /* 2. Convert raw int32 to float in-place */
        int32_t *raw = reinterpret_cast<int32_t *>(audio_buffers[current_buffer]);
        for (int i = 0; i < SAMPLES_SHORT; i++) {
            audio_buffers[current_buffer][i] = static_cast<float>(MIC_CONVERT(raw[i]));
        }

        /* 3. DSP chain — DC block then mic compensation EQ */
        DC_BLOCKER.filter(audio_buffers[current_buffer],
                          audio_buffers[current_buffer],
                          SAMPLES_SHORT);
        INMP441.filter(audio_buffers[current_buffer],
                       audio_buffers[current_buffer],
                       SAMPLES_SHORT);

        /* 4. Hand off buffer index to systemTask.
         *    Timeout = 0: if queue is full we drop this frame rather than
         *    block the DMA pipeline. The drop counter lets us diagnose this. */
        if (xQueueSend(audioQueue, &current_buffer, 0) != pdPASS) {
            droppedFrames.fetch_add(1, std::memory_order_relaxed);
        }

        /* 5. Flip buffer */
        current_buffer ^= 1;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   TASK: systemTask  (Core 0, low priority)
   Responsibilities:
     - Debounce / edge-detect two push buttons
     - Toggle between STREAMING and CALIBRATING mode (hold both 2 s)
     - Adjust calibration value only while CALIBRATING      
     - Pack calibration_multiplier into packet header every frame
       so Pi always has the current gain value alongside the raw samples
     - Deferred NVS save with non-blocking notify           
     - Feed GPS UART
     - Drain audioQueue and send USB CDC frames (with mutex) 
   ═══════════════════════════════════════════════════════════════════════════ */
void systemTask(void *pvParameters) {

    /* Local copy of calibration — avoids repeated atomic loads in hot path */
    float localCalibration = currentCalibration.load();

    /* ── Initial LCD display ─────────────────────────────────────────────── */
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mode: STREAMING");
    lcd.setCursor(0, 1);
    lcd.printf("Gain: %.2f", localCalibration);

    /* ── Button state ────────────────────────────────────────────────────── */
    bool lastUpState   = HIGH;
    bool lastDownState = HIGH;

    /* ── Hold-both state (shared for both ENTER and EXIT) ────────────────── */
    unsigned long bothPressStart = 0;
    bool          holdFired      = false;   // prevents repeated triggers

    /* ── NVS deferred save ───────────────────────────────────────────────── */
    bool          calibChanged  = false;
    unsigned long lastSaveTime  = 0;

    /* ── Non-blocking LCD notify timer (FIX #6) ──────────────────────────── */
    unsigned long saveNotifyEnd = 0;

    /* ── Diagnostic: last reported drop count ────────────────────────────── */
    uint32_t lastDropReport = 0;

    while (true) {
        unsigned long now       = millis();
        bool          upPressed   = (digitalRead(BTN_UP)   == LOW);
        bool          downPressed = (digitalRead(BTN_DOWN) == LOW);
        bool currentIsCalib = (static_cast<DeviceMode>(deviceMode.load())
                               == DeviceMode::CALIBRATING);

        /* ── HOLD-BOTH-BUTTONS: toggle mode (enter OR exit) ─────────────────
         *   same gesture enters and exits calibration mode  */
        if (upPressed && downPressed) {
            if (bothPressStart == 0) {
                bothPressStart = now;
                holdFired      = false;
            } else if (!holdFired && (now - bothPressStart >= HOLD_DURATION_MS)) {
                holdFired = true;   // arm once per press

                if (!currentIsCalib) {
                    /* ── Enter CALIBRATING ─── */
                    deviceMode.store(static_cast<uint8_t>(DeviceMode::CALIBRATING));

                    Serial.println("\n=================================");
                    Serial.println(" MASUK MODE KALIBRASI");
                    Serial.println(" USB CDC Dihentikan.");
                    Serial.println("=================================\n");

                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("MODE: KALIBRASI ");
                    lcd.setCursor(0, 1);
                    lcd.printf("Gain: %.2f      ", localCalibration);

                } else {
                    /* ── Exit CALIBRATING → STREAMING ─── */
                    deviceMode.store(static_cast<uint8_t>(DeviceMode::STREAMING));

                    Serial.println("\n=================================");
                    Serial.println(" KELUAR DARI KALIBRASI");
                    Serial.println(" USB CDC Dilanjutkan.");
                    Serial.println("=================================\n");

                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Mode: STREAMING ");
                    lcd.setCursor(0, 1);
                    lcd.printf("Gain: %.2f      ", localCalibration);
                }

                currentIsCalib = !currentIsCalib;   // update local view
            }
        } else {
            bothPressStart = 0;
            holdFired      = false;
        }

        /* ── SINGLE-BUTTON EDGE DETECTION
         *   Only process UP/DOWN presses when in CALIBRATING mode.
         *   In STREAMING mode these lines are completely skipped, so the
         *   gain value is frozen regardless of button activity.             */
        if (currentIsCalib) {

            // /* Rising edge: button was HIGH (released) last tick, LOW (pressed) now */
            // if (!lastUpState && !upPressed) {
            //     /* ignore — both being held is handled above */
            // }

            // bool upEdge   = (lastUpState   == HIGH && upPressed);
            // bool downEdge = (lastDownState == HIGH && downPressed);

            /* Falling edge: button was LOW (pressed) last tick, HIGH (released) now */
            bool upEdge   = (lastUpState   == LOW && !upPressed);
            bool downEdge = (lastDownState == LOW && !downPressed);

            if (upEdge && !downPressed) {   // ignore if both held simultaneously
                localCalibration += 0.5f;
                currentCalibration.store(localCalibration);
                calibChanged = true;
                saveNotifyEnd = 0;          // clear any pending notify

                Serial.printf("[KALIBRASI] Gain: %.2f\n", localCalibration);

                lcd.setCursor(0, 1);
                lcd.printf("Gain: %.2f      ", localCalibration);
            }

            if (downEdge && !upPressed) {
                localCalibration -= 0.5f;
                if (localCalibration < 0.0f) localCalibration = 0.0f;
                currentCalibration.store(localCalibration);
                calibChanged = true;
                saveNotifyEnd = 0;

                Serial.printf("[KALIBRASI] Gain: %.2f\n", localCalibration);

                lcd.setCursor(0, 1);
                lcd.printf("Gain: %.2f      ", localCalibration);
            }
        }

        /* Store last button states AFTER edge detection */
        lastUpState   = upPressed   ? LOW : HIGH;
        lastDownState = downPressed ? LOW : HIGH;

        /* ── NVS DEFERRED SAVE ───────────────────────────────────────────── */
        if (calibChanged && (now - lastSaveTime >= NVS_SAVE_DELAY_MS)) {
            preferences.putFloat("multiplier", localCalibration);
            calibChanged  = false;
            lastSaveTime  = now;

            Serial.printf("[NVS] Kalibrasi disimpan: %.2f\n", localCalibration);

            /* Start non-blocking notify timer  */
            if (currentIsCalib) {
                saveNotifyEnd = now + SAVE_NOTIFY_MS;
                lcd.setCursor(0, 1);
                lcd.print("Tersimpan!      ");
            }
        }

        /* ── NON-BLOCKING LCD NOTIFY TIMER  ─────────────────────── */
        if (saveNotifyEnd != 0 && now >= saveNotifyEnd) {
            saveNotifyEnd = 0;
            /* Restore gain display after notify expires */
            lcd.setCursor(0, 1);
            lcd.printf("Gain: %.2f      ", localCalibration);
        }

        /* ── GPS FEED ────────────────────────────────────────────────────── */
        while (gpsSerial.available()) {
            gps.encode(gpsSerial.read());
        }

        /* ── AUDIO FRAME → USB CDC ───────────────────────────────────────── */
        int ready_buf = -1;
        if (xQueueReceive(audioQueue, &ready_buf, pdMS_TO_TICKS(5)) == pdPASS) {

            if (!currentIsCalib) {
                /* Acquire mutex before touching the packet struct 
                 * Timeout = 10 ms: if we can't get it, skip this frame rather
                 * than block forever. In practice the mutex should always be
                 * free here because only this task writes/reads usb_packet. */
                if (xSemaphoreTake(packetMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

                    usb_packet.timestamp_ms          = now;
                    usb_packet.latitude              = gps.location.lat();
                    usb_packet.longitude             = gps.location.lng();
                    usb_packet.calibration_multiplier = localCalibration;

                    memcpy(usb_packet.samples,
                           audio_buffers[ready_buf],
                           SAMPLES_SHORT * sizeof(float));

                    uint32_t start_m = FRAME_START_MARKER;
                    uint32_t end_m   = FRAME_END_MARKER;

                    Serial.write(reinterpret_cast<uint8_t *>(&start_m), sizeof(start_m));
                    Serial.write(reinterpret_cast<uint8_t *>(&usb_packet), sizeof(usb_packet_t));
                    Serial.write(reinterpret_cast<uint8_t *>(&end_m),   sizeof(end_m));

                    xSemaphoreGive(packetMutex);
                }
            }
        }

        /* ── DIAGNOSTIC: report dropped frames periodically ─────────────── */
        uint32_t drops = droppedFrames.load(std::memory_order_relaxed);
        if (drops != lastDropReport) {
            Serial.printf("[WARN] Frame drops: %lu total\n",
                          static_cast<unsigned long>(drops));
            lastDropReport = drops;
        }

        vTaskDelay(1);   
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   I2S INIT
   ═══════════════════════════════════════════════════════════════════════════ */
void mic_i2s_init() {
    i2s_config_t i2s_config = {
        .mode              = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate       = SAMPLE_RATE,
        .bits_per_sample   = static_cast<i2s_bits_per_sample_t>(SAMPLE_BITS),
        .channel_format    = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_I2S),
        .intr_alloc_flags  = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count     = 8,
        .dma_buf_len       = 512,
        .use_apll          = true
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num    = I2S_SCK,
        .ws_io_num     = I2S_WS,
        .data_out_num  = I2S_PIN_NO_CHANGE,
        .data_in_num   = I2S_SD
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
}

/* ═══════════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════════ */
void setup() {
    Serial.begin(115200);
    setCpuFrequencyMhz(240);

    /* LCD */
    Wire.begin(I2C_SDA, I2C_SCL);
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Memulai Sistem..");

    /* Buttons */
    pinMode(BTN_UP,   INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);

    /* NVS – restore saved calibration */
    preferences.begin("audio-calib", false);
    float saved = preferences.getFloat("multiplier", 1.0f);
    currentCalibration.store(saved);

    /* I2S mic */
    mic_i2s_init();

    /* GPS UART */
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

    /* Queue: depth 8 to reduce drops under heavy load.
     * Each slot is sizeof(int) = 4 bytes. Total overhead: 32 bytes. */
    audioQueue = xQueueCreate(8, sizeof(int));
    configASSERT(audioQueue);

    /* Mutex protecting usb_packet */
    packetMutex = xSemaphoreCreateMutex();
    configASSERT(packetMutex);

    /* --- Spawn tasks ---
     *
     * audioDSPTask: Core 1, priority = configMAX_PRIORITIES-1 (highest)
     *   Must never be pre-empted by systemTask so DMA buffers are drained
     *   fast enough that I2S hardware does not overflow.
     *
     * systemTask:  Core 0, priority = 1 (just above idle)
     *   All user-facing logic: buttons, GPS, USB send, LCD.
     *   Lower priority is intentional — audio integrity > UI latency.
     *
     * Stack sizes bumped to 10240 to give headroom for printf/LCD calls.
     */
    xTaskCreatePinnedToCore(audioDSPTask, "AudioDSP",  10240, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);

    xTaskCreatePinnedToCore(systemTask,   "SystemTask", 10240, NULL,
                            1,                         NULL, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
   LOOP — not used; Arduino loop() task is deleted immediately
   ═══════════════════════════════════════════════════════════════════════════ */
void loop() {
    vTaskDelete(NULL);
}