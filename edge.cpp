/**
 * =============================================================================
 * ESP32-S3 Integrated Firmware
 * Audio Streaming + GPS + Calibration + INA219 Battery Monitor
 * =============================================================================
 *
 * ARCHITECTURE:
 *   Core 1 — audioDSPTask (highest priority)
 *     • i2s_read → MIC_CONVERT → DC_BLOCKER → INMP441 EQ → audioQueue
 *     • Runs unconditionally regardless of mode. No shared state reads.
 *
 *   Core 0 — systemTask (priority 1)
 *     • Reads INA219 (voltage, current, mAh) every 1 s — non-blocking timer
 *     • Reads GPS UART
 *     • Handles buttons (UP/DOWN/hold-both) for calibration mode toggle
 *     • Updates LCD based on current mode (STREAMING = battery, CALIB = gain)
 *     • Drains audioQueue and sends USB CDC binary frames
 *     • All diagnostic Serial.printf ONLY fires when in CALIBRATING mode
 *       (streaming is paused, so binary/text collision is impossible)
 *
 * USB PACKET LAYOUT (little-endian, packed):
 *   [0x AA55AA55] start marker       4 B
 *   timestamp_ms                     4 B  (uint32)
 *   latitude                         4 B  (float)
 *   longitude                        4 B  (float)
 *   calibration_multiplier           4 B  (float)  ← Pi applies gain here
 *   battery_percentage               4 B  (float)  ← NEW field
 *   samples[SAMPLES_SHORT]       24000 B  (float[6000])
 *   [0x 55AA55AA] end marker         4 B
 *   TOTAL                        24028 B
 *
 * NOTE FOR RASPBERRY PI:
 *   HEADER_FMT  = '<II4f'        (start, ts, lat, lon, calib, batt)
 *   HEADER_SIZE = 24 bytes
 *   FRAME_SIZE  = 24 + 24000 + 4 = 24028 bytes
 *   Update noise_monitor.py accordingly.
 *
 * LCD OWNERSHIP:
 *   STREAMING mode  → row 0: voltage + current   row 1: battery % + mAh used
 *   CALIBRATING mode → row 0: "MODE: KALIBRASI"  row 1: gain value
 *   Mode transition clears the screen to avoid ghost text.
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
#include <Adafruit_INA219.h>

/* ─── I2C ────────────────────────────────────────────────────────────────── */
// Both LCD (0x27) and INA219 (0x40) share the same I2C bus — Wire.begin()
// only called once. Do NOT call Wire.begin() more than once.
#define I2C_SDA 4
#define I2C_SCL 5

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_INA219    ina219;            // default address 0x40

/* ─── AUDIO CONFIG ───────────────────────────────────────────────────────── */
#define SAMPLE_RATE    48000
#define SAMPLE_BITS    32
#define MIC_BITS       24
#define SAMPLES_SHORT  (SAMPLE_RATE / 8)          // 125 ms = 6000 samples
#define MIC_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))

/* ─── I2S PINS ───────────────────────────────────────────────────────────── */
#define I2S_WS   9
#define I2S_SCK  39
#define I2S_SD   15     
#define I2S_PORT I2S_NUM_0

/* ─── GPS ────────────────────────────────────────────────────────────────── */
#define RXD2     16
#define TXD2     17
#define GPS_BAUD 9600

/* ─── FRAME MARKERS ──────────────────────────────────────────────────────── */
#define FRAME_START_MARKER  0xAA55AA55
#define FRAME_END_MARKER    0x55AA55AA

/* ─── BUTTONS ────────────────────────────────────────────────────────────── */
#define BTN_UP   3
#define BTN_DOWN 10

/* ─── TIMING ─────────────────────────────────────────────────────────────── */
#define HOLD_DURATION_MS    2000   // hold both buttons to toggle mode
#define NVS_SAVE_DELAY_MS   8000   // min ms between NVS writes
#define SAVE_NOTIFY_MS      1500   // "Tersimpan!" display duration
#define BATTERY_INTERVAL_MS 1000   // INA219 sample interval

/* ─── BATTERY SPEC (3S LiPo 12 V nominal) ───────────────────────────────── */
#define BATT_V_MAX  12.6f
#define BATT_V_MIN   9.6f

/* ─── MODE ENUM ──────────────────────────────────────────────────────────── */
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

/* --- Atomic shared state -------------------------------------------------- */
// Only systemTask writes these; audioDSPTask never reads either of them.
std::atomic<uint8_t> deviceMode(static_cast<uint8_t>(DeviceMode::STREAMING));
std::atomic<float>   currentCalibration(1.0f);

/* --- USB packet ----------------------------------------------------------- */
// IMPORTANT: if you add/remove fields here, update HEADER_FMT in
// noise_monitor.py on the Raspberry Pi accordingly.
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    latitude;
    float    longitude;
    float    calibration_multiplier;   // Pi multiplies raw samples by this
    float    battery_percentage;       // informational — logged by Pi
    float    samples[SAMPLES_SHORT];
} usb_packet_t;

static usb_packet_t      usb_packet;
static SemaphoreHandle_t packetMutex = nullptr;

/* --- Audio double-buffer + queue ----------------------------------------- */
static float         audio_buffers[2][SAMPLES_SHORT] __attribute__((aligned(4)));
static QueueHandle_t audioQueue = nullptr;

/* --- Diagnostics ---------------------------------------------------------- */
static std::atomic<uint32_t> droppedFrames(0);

/* ─── IIR FILTERS ────────────────────────────────────────────────────────── */
// Coefficients must match the specific INMP441 unit used.
SOS_IIR_Filter DC_BLOCKER = {
    1.0,
    { {-1.0, 0.0, 0.9992, 0} }
};
SOS_IIR_Filter INMP441 = {
    1.00197834654696,
    { {-1.986920458344451, 0.986963226946616,
       1.995178510504166, -0.995184322194091} }
};

/* ═══════════════════════════════════════════════════════════════════════════
   I2S INIT
   ═══════════════════════════════════════════════════════════════════════════ */
void mic_i2s_init() {
    i2s_config_t cfg = {
        .mode              = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate       = SAMPLE_RATE,
        .bits_per_sample   = static_cast<i2s_bits_per_sample_t>(SAMPLE_BITS),
        .channel_format    = I2S_CHANNEL_FMT_ONLY_LEFT,
        // BUG in new code: I2S_COMM_FORMAT_I2S_MSB is deprecated / undefined on
        // esp-idf ≥4.4. Use I2S_COMM_FORMAT_I2S only (standard Philips I2S).
        .communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_I2S),
        .intr_alloc_flags  = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count     = 8,
        .dma_buf_len       = 512,
        .use_apll          = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk        = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num    = I2S_SCK,
        .ws_io_num     = I2S_WS,
        .data_out_num  = I2S_PIN_NO_CHANGE,
        .data_in_num   = I2S_SD
    };
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pins);
}

/* ═══════════════════════════════════════════════════════════════════════════
   TASK: audioDSPTask  (Core 1, max priority)

   Pure acquisition: i2s_read → convert → DC block → EQ → queue.
   Zero shared-state reads. Gain is NOT applied — Pi does that.
   ═══════════════════════════════════════════════════════════════════════════ */
void audioDSPTask(void *) {
    int buf = 0;
    while (true) {
        size_t bytes_read = 0;
        i2s_read(I2S_PORT,
                 audio_buffers[buf],
                 SAMPLES_SHORT * sizeof(int32_t),
                 &bytes_read,
                 portMAX_DELAY);

        // In-place int32 → float conversion
        int32_t *raw = reinterpret_cast<int32_t *>(audio_buffers[buf]);
        for (int i = 0; i < SAMPLES_SHORT; i++) {
            audio_buffers[buf][i] = static_cast<float>(MIC_CONVERT(raw[i]));
        }

        DC_BLOCKER.filter(audio_buffers[buf], audio_buffers[buf], SAMPLES_SHORT);
        INMP441.filter   (audio_buffers[buf], audio_buffers[buf], SAMPLES_SHORT);

        // Offer buffer index to systemTask. Timeout = 0 to never block DMA.
        if (xQueueSend(audioQueue, &buf, 0) != pdPASS) {
            droppedFrames.fetch_add(1, std::memory_order_relaxed);
        }
        buf ^= 1;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   TASK: systemTask  (Core 0, priority 1)

   All user-facing logic. LCD ownership is gated by DeviceMode:
     STREAMING   → battery info on LCD, binary frames sent via USB CDC
     CALIBRATING → gain display on LCD, USB CDC paused
   ═══════════════════════════════════════════════════════════════════════════ */
void systemTask(void *) {

    // ── Local copies ─────────────────────────────────────────────────────
    float localCalibration = currentCalibration.load();

    // ── Battery state (all local to this task — no sharing needed) ───────
    float         battVoltage    = 0.0f;
    float         battCurrent_mA = 0.0f;
    float         battPercent    = 0.0f;
    float         mAh_used       = 0.0f;
    unsigned long battLastMs     = millis();

    // ── Button state ──────────────────────────────────────────────────────
    bool          lastUp   = HIGH;
    bool          lastDown = HIGH;
    unsigned long bothPressStart = 0;
    bool          holdFired      = false;

    // ── NVS deferred save ─────────────────────────────────────────────────
    bool          calibChanged = false;
    unsigned long lastSaveMs   = 0;

    // ── Non-blocking LCD "Tersimpan!" notify ──────────────────────────────
    unsigned long saveNotifyEnd = 0;

    // ── Drop diagnostics ──────────────────────────────────────────────────
    uint32_t lastDropReport = 0;

    // ── Initial LCD (STREAMING mode default) ─────────────────────────────
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("0.00V   0.0mA   ");
    lcd.setCursor(0, 1);
    lcd.print("0.0%    0.0mAh  ");

    while (true) {
        unsigned long now = millis();
        bool upPressed    = (digitalRead(BTN_UP)   == LOW);
        bool downPressed  = (digitalRead(BTN_DOWN) == LOW);
        bool isCalib      = (static_cast<DeviceMode>(deviceMode.load())
                             == DeviceMode::CALIBRATING);

        /* ════════════════════════════════════════════════════════════════
           BATTERY MONITOR — runs every BATTERY_INTERVAL_MS
           Non-blocking: uses elapsed-time check, never delays.
           Only updates LCD when in STREAMING mode so calibration display
           is not clobbered.
           ════════════════════════════════════════════════════════════════ */
        if (now - battLastMs >= BATTERY_INTERVAL_MS) {
            unsigned long elapsed_ms = now - battLastMs;
            battLastMs = now;

            // battVoltage    = ina219.getBusVoltage_V();
            // battCurrent_mA = ina219.getCurrent_mA();
            
            // Inject fake battery data (e.g., a healthy 11.4V drawing 250mA)
            battVoltage    = 11.4f;  
            battCurrent_mA = 250.0f;

            // Coulomb counting: mAh += mA * hours
            mAh_used += battCurrent_mA * (elapsed_ms / 3600000.0f);

            // Linear voltage → percentage mapping, clamped 0–100 %
            battPercent = ((battVoltage - BATT_V_MIN) / (BATT_V_MAX - BATT_V_MIN)) * 100.0f;
            if (battPercent > 100.0f) battPercent = 100.0f;
            if (battPercent < 0.0f)   battPercent = 0.0f;

            // Only update LCD if we own it (STREAMING mode)
            if (!isCalib) {
                lcd.setCursor(0, 0);
                lcd.printf("%.2fV  %.1fmA   ", battVoltage, battCurrent_mA);
                lcd.setCursor(0, 1);
                lcd.printf("%.1f%%  %.1fmAh  ", battPercent, mAh_used);
            }
        }

        /* ════════════════════════════════════════════════════════════════
           HOLD-BOTH-BUTTONS → toggle STREAMING ↔ CALIBRATING
           ════════════════════════════════════════════════════════════════ */
        if (upPressed && downPressed) {
            if (bothPressStart == 0) {
                bothPressStart = now;
                holdFired      = false;
            } else if (!holdFired && (now - bothPressStart >= HOLD_DURATION_MS)) {
                holdFired = true;

                if (!isCalib) {
                    // ── Enter CALIBRATING ──────────────────────────────
                    deviceMode.store(static_cast<uint8_t>(DeviceMode::CALIBRATING));
                    isCalib = true;

                    // Only print diagnostics when streaming is paused
                    // (mixing text + binary on CDC corrupts the binary stream)
                    Serial.println("\n[MODE] KALIBRASI — streaming paused");

                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("MODE: KALIBRASI ");
                    lcd.setCursor(0, 1);
                    lcd.printf("Gain: %.2f      ", localCalibration);

                } else {
                    // ── Exit CALIBRATING → STREAMING ──────────────────
                    deviceMode.store(static_cast<uint8_t>(DeviceMode::STREAMING));
                    isCalib = false;

                    Serial.println("[MODE] STREAMING — resumed");

                    // Restore battery display
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.printf("%.2fV  %.1fmA   ", battVoltage, battCurrent_mA);
                    lcd.setCursor(0, 1);
                    lcd.printf("%.1f%%  %.1fmAh  ", battPercent, mAh_used);
                }
            }
        } else {
            bothPressStart = 0;
            holdFired      = false;
        }

        /* ════════════════════════════════════════════════════════════════
           SINGLE-BUTTON EDGE DETECTION
           Only processed in CALIBRATING mode — FIX from previous code.
           ════════════════════════════════════════════════════════════════ */
        if (isCalib) {
            // bool upEdge   = (!lastUp   && upPressed);
            // bool downEdge = (!lastDown && downPressed);

            /* Falling edge: button was LOW (pressed) last tick, HIGH (released) now */
            bool upEdge   = (lastUp   == LOW && !upPressed);
            bool downEdge = (lastDown == LOW && !downPressed);

            if (upEdge && !downPressed) {
                localCalibration += 0.5f;
                currentCalibration.store(localCalibration);
                calibChanged  = true;
                saveNotifyEnd = 0;

                Serial.printf("[CALIB] Gain: %.2f\n", localCalibration);
                lcd.setCursor(0, 1);
                lcd.printf("Gain: %.2f      ", localCalibration);
            }

            if (downEdge && !upPressed) {
                localCalibration -= 0.5f;
                if (localCalibration < 0.0f) localCalibration = 0.0f;
                currentCalibration.store(localCalibration);
                calibChanged  = true;
                saveNotifyEnd = 0;

                Serial.printf("[CALIB] Gain: %.2f\n", localCalibration);
                lcd.setCursor(0, 1);
                lcd.printf("Gain: %.2f      ", localCalibration);
            }
        }

        lastUp   = upPressed   ? LOW : HIGH;
        lastDown = downPressed ? LOW : HIGH;

        /* ════════════════════════════════════════════════════════════════
           NVS DEFERRED SAVE
           ════════════════════════════════════════════════════════════════ */
        if (calibChanged && (now - lastSaveMs >= NVS_SAVE_DELAY_MS)) {
            preferences.putFloat("multiplier", localCalibration);
            calibChanged = false;
            lastSaveMs   = now;

            Serial.printf("[NVS] Saved: %.2f\n", localCalibration);

            if (isCalib) {
                saveNotifyEnd = now + SAVE_NOTIFY_MS;
                lcd.setCursor(0, 1);
                lcd.print("Tersimpan!      ");
            }
        }

        /* ════════════════════════════════════════════════════════════════
           NON-BLOCKING "Tersimpan!" NOTIFY EXPIRY
           ════════════════════════════════════════════════════════════════ */
        if (saveNotifyEnd != 0 && now >= saveNotifyEnd) {
            saveNotifyEnd = 0;
            if (isCalib) {
                lcd.setCursor(0, 1);
                lcd.printf("Gain: %.2f      ", localCalibration);
            }
        }

        /* ════════════════════════════════════════════════════════════════
           GPS FEED
           ════════════════════════════════════════════════════════════════ */
        while (gpsSerial.available()) {
            gps.encode(gpsSerial.read());
        }

        /* ════════════════════════════════════════════════════════════════
           AUDIO FRAME → USB CDC
           Only sends when in STREAMING mode.
           packetMutex prevents struct corruption if a second task ever
           needs to inspect usb_packet in the future.
           ════════════════════════════════════════════════════════════════ */
        int ready_buf = -1;
        if (xQueueReceive(audioQueue, &ready_buf, pdMS_TO_TICKS(5)) == pdPASS) {
            if (!isCalib) {
                if (xSemaphoreTake(packetMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    usb_packet.timestamp_ms           = now;
                    usb_packet.latitude               = gps.location.lat();
                    usb_packet.longitude              = gps.location.lng();
                    usb_packet.calibration_multiplier = localCalibration;
                    usb_packet.battery_percentage     = battPercent;

                    memcpy(usb_packet.samples,
                           audio_buffers[ready_buf],
                           SAMPLES_SHORT * sizeof(float));

                    uint32_t sm = FRAME_START_MARKER;
                    uint32_t em = FRAME_END_MARKER;
                    Serial.write(reinterpret_cast<uint8_t *>(&sm), 4);
                    Serial.write(reinterpret_cast<uint8_t *>(&usb_packet), sizeof(usb_packet_t));
                    Serial.write(reinterpret_cast<uint8_t *>(&em), 4);

                    xSemaphoreGive(packetMutex);
                }
            }
        }

        /* ════════════════════════════════════════════════════════════════
           DROP DIAGNOSTIC
           Serial.printf is safe here because it only fires when streaming
           is paused (isCalib=true) OR we accept a brief text blip.
           For a production build, remove or gate behind isCalib.
           ════════════════════════════════════════════════════════════════ */
        uint32_t drops = droppedFrames.load(std::memory_order_relaxed);
        if (drops != lastDropReport) {
            // Only print in calib mode to avoid corrupting binary stream
            if (isCalib) {
                Serial.printf("[WARN] Frame drops: %lu total\n",
                              static_cast<unsigned long>(drops));
            }
            lastDropReport = drops;
        }

        vTaskDelay(1);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════════ */
void setup() {
    Serial.begin(115200);
    // BUG in new code: while (!Serial) blocks forever if Pi is not actively
    // reading the CDC port at boot. Removed — the device must boot standalone.
    setCpuFrequencyMhz(240);

    /* ── I2C bus (shared by LCD + INA219) ──────────────────────────────── */
    Wire.begin(I2C_SDA, I2C_SCL);

    /* ── LCD splash ─────────────────────────────────────────────────────── */
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Memulai Sistem..");
    lcd.setCursor(0, 1);
    lcd.print("Versi Terpadu   ");

    /* ── INA219 ─────────────────────────────────────────────────────────── */
    if (!ina219.begin()) {
        // lcd.clear();
        // lcd.setCursor(0, 0);
        // lcd.print("ERROR: INA219!  ");
        // lcd.setCursor(0, 1);
        // lcd.print("Cek Kabel I2C   ");
        // // Halt — no point continuing without battery monitoring
        // while (true) { vTaskDelay(pdMS_TO_TICKS(100)); }
        Serial.println("SIMULATION MODE: INA219 Not Found. Using fake data.");
    }

    /* ── Buttons ─────────────────────────────────────────────────────────── */
    pinMode(BTN_UP,   INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);

    /* ── NVS calibration restore ─────────────────────────────────────────── */
    preferences.begin("audio-calib", false);
    float saved = preferences.getFloat("multiplier", 1.0f);
    currentCalibration.store(saved);

    /* ── I2S mic ─────────────────────────────────────────────────────────── */
    mic_i2s_init();

    /* ── GPS UART ────────────────────────────────────────────────────────── */
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

    /* ── RTOS primitives ─────────────────────────────────────────────────── */
    audioQueue = xQueueCreate(8, sizeof(int));
    configASSERT(audioQueue);

    packetMutex = xSemaphoreCreateMutex();
    configASSERT(packetMutex);

    /* ── Splash delay ────────────────────────────────────────────────────── */
    delay(1500);
    lcd.clear();

    /* ── Tasks ───────────────────────────────────────────────────────────── */
    // audioDSPTask: Core 1, max priority — must never be starved so DMA
    // hardware does not overflow between reads.
    xTaskCreatePinnedToCore(
        audioDSPTask, "AudioDSP",
        10240, NULL, configMAX_PRIORITIES - 1, NULL, 1
    );

    // systemTask: Core 0, priority 1 — all user-facing and I/O work.
    // Lower priority than audioDSPTask is intentional.
    xTaskCreatePinnedToCore(
        systemTask, "SystemTask",
        12288, NULL, 1, NULL, 0   // 12 KB: larger for INA219 + LCD printf
    );

    // Log frame layout so Pi-side developer can verify HEADER_FMT
    Serial.printf("[INIT] Frame: %u B | CPU: %u MHz | Gain: %.2f\n",
                  4 + sizeof(usb_packet_t) + 4,
                  getCpuFrequencyMhz(),
                  currentCalibration.load());
}

/* ═══════════════════════════════════════════════════════════════════════════
   LOOP — deleted immediately; all work is in RTOS tasks
   ═══════════════════════════════════════════════════════════════════════════ */
void loop() {
    vTaskDelete(NULL);
}