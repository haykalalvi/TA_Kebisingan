/**

 * ESP32-S3 (Super-Loop / Single Core Architecture)
 * Audio Streaming + Calibration + INA219 Battery Monitor (Coulomb Counting)
 */

#include <TinyGPS++.h>
#include <Arduino.h>
#include <driver/i2s.h>
#include <Preferences.h>
#include "sos-iir-filter.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

/* ─── I2C ────────────────────────────────────────────────────────────────── */
#define I2C_SDA 4 
#define I2C_SCL 5 
Adafruit_INA219 ina219;

/* ─── AUDIO CONFIG ───────────────────────────────────────────────────────── */
#define SAMPLE_RATE    48000
#define SAMPLE_BITS    32
#define MIC_BITS       24
#define SAMPLES_SHORT  (SAMPLE_RATE / 8)   // 125ms = 6000 samples
#define MIC_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))

/* ─── I2S PINS ───────────────────────────────────────────────────────────── */
#define I2S_WS   9
#define I2S_SCK  39
#define I2S_SD   15
#define I2S_PORT I2S_NUM_0

/* ─── GPS ────────────────────────────────────────────────────────────────── */
#define RXD2     18
#define TXD2     17
#define GPS_BAUD 9600

/* ─── FRAME MARKERS ──────────────────────────────────────────────────────── */
#define FRAME_START_MARKER  0xAA55AA55
#define FRAME_END_MARKER    0x55AA55AA

/* ─── BUTTONS ────────────────────────────────────────────────────────────── */
#define BTN_UP   3
#define BTN_DOWN 10

/* ─── TIMING ─────────────────────────────────────────────────────────────── */
#define HOLD_DURATION_MS    2000
#define NVS_SAVE_DELAY_MS   8000
#define SAVE_NOTIFY_MS      1500
#define BATTERY_INTERVAL_MS 1000

/* ─── BATTERY SPEC (Coulomb Counting) ────────────────────────────────────── */
// const float ENERGI_TOTAL_Wh = 111.0;
const float ENERGI_TOTAL_Wh =  87.759;
float energiTerpakai_Wh = 0.0;
float kapasitasTerpakai_mAh = 0.0;

/* ─── MODE ENUM ──────────────────────────────────────────────────────────── */
enum class DeviceMode : uint8_t {
    STREAMING   = 0,
    CALIBRATING = 1
};

/* ═══════════════════════════════════════════════════════════════════════════
   GLOBALS
   ═══════════════════════════════════════════════════════════════════════════ */
TinyGPSPlus    gps;
HardwareSerial gpsSerial(1);
Preferences    preferences;

DeviceMode deviceMode        = DeviceMode::STREAMING;
float      currentCalibration = 1.0f;

/* ─── USB PACKET ─────────────────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    latitude;
    float    longitude;
    float    calibration_multiplier;
    float    battery_percentage;
    float    calibration_mode;
    float    samples[SAMPLES_SHORT];
} usb_packet_t;

static usb_packet_t usb_packet;
static int32_t raw_i2s_buffer[SAMPLES_SHORT] __attribute__((aligned(4)));

/* ─── IIR FILTERS ────────────────────────────────────────────────────────── */
SOS_IIR_Filter DC_BLOCKER = {
    1.0,
    { {-1.0, 0.0, 0.999975, 0} }   // cutoff ~0.19Hz — removes DC + sub-Hz drift
};

// Option 1b: ~20 Hz cutoff — good balance, noise at 20Hz band drops ~8 dB
// SOS_IIR_Filter DC_BLOCKER = {
//     1.0, { {-1.0, 0.0, 0.997383, 0} }
// };

// SOS_IIR_Filter DC_BLOCKER = {
//     1.0, { {-1.0, 0.0, 0.993446, 0} }
// };

// ini default
// SOS_IIR_Filter INMP441 = {
//     1.00197834654696,
//     { {-1.986920458344451, 0.986963226946616,
//        1.995178510504166, -0.995184322194091} }
// };

SOS_IIR_Filter INMP441 = {
    1.002293656656743,
    { { -1.984311050946219, 0.984433168289200,
        1.988892905899653, -0.988954249933127 } }
};

/* ═══════════════════════════════════════════════════════════════════════════
   FILTER RESET
   ═══════════════════════════════════════════════════════════════════════════ */
void resetFilters() {
    for (int i = 0; i < DC_BLOCKER.num_sos; i++) {
        DC_BLOCKER.w[i].w0 = 0.0f;
        DC_BLOCKER.w[i].w1 = 0.0f;
    }
    for (int i = 0; i < INMP441.num_sos; i++) {
        INMP441.w[i].w0 = 0.0f;
        INMP441.w[i].w1 = 0.0f;
    }
    i2s_zero_dma_buffer(I2S_PORT);
}

/* ═══════════════════════════════════════════════════════════════════════════
   I2S INIT
   ═══════════════════════════════════════════════════════════════════════════ */
void mic_i2s_init() {
    i2s_config_t cfg = {
        .mode              = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate       = SAMPLE_RATE,
        .bits_per_sample   = static_cast<i2s_bits_per_sample_t>(SAMPLE_BITS),
        .channel_format    = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_I2S),
        .intr_alloc_flags  = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count     = 8,
        .dma_buf_len       = 512,
        .use_apll          = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk        = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = I2S_SCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_SD
    };
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pins);
}

/* ═══════════════════════════════════════════════════════════════════════════
   RESET COULOMB COUNTING
   ═══════════════════════════════════════════════════════════════════════════ */
void resetCoulombCounting() {
    energiTerpakai_Wh = 0.0;
    kapasitasTerpakai_mAh = 0.0;
}

/* ═══════════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════════ */
void setup() {
    Serial.setTxBufferSize(32768);
    Serial.begin(115200);
    setCpuFrequencyMhz(80); // mungkin karena dia cuma 80mhz

    Wire.begin(I2C_SDA, I2C_SCL);     
    ina219.begin();  
    pinMode(BTN_UP,   INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);

    preferences.begin("audio-calib", false);
    currentCalibration = preferences.getFloat("multiplier", 1.0f);

    mic_i2s_init();
}

/* ═══════════════════════════════════════════════════════════════════════════
   MAIN LOOP
   ═══════════════════════════════════════════════════════════════════════════ */
void loop() {
    static float         battPercent     = 100.0f; // Default assume full
    static unsigned long battLastMs      = millis();

    static bool          lastUp          = HIGH;
    static bool          lastDown        = HIGH;
    static unsigned long bothPressStart  = 0;
    static bool          holdFired       = false;

    static bool          calibChanged    = false;
    static unsigned long lastSaveMs      = 0;
    static unsigned long saveNotifyEnd   = 0;

    static bool          lastUsbConnected = false;
    static bool          warmupDone       = false;  // ← one-shot guard

    unsigned long now          = millis();
    bool          usbConnected = (bool)Serial;

    // =========================================================================
    // BUTTON & CALIBRATION LOGIC FIRST — before any blocking calls
    // =========================================================================
    bool upPressed   = (digitalRead(BTN_UP)   == LOW);
    bool downPressed = (digitalRead(BTN_DOWN) == LOW);
    bool isCalib     = (deviceMode == DeviceMode::CALIBRATING);

    // Hold both buttons to toggle mode
    if (upPressed && downPressed) {
        if (bothPressStart == 0) {
            bothPressStart = now;
            holdFired      = false;
        } else if (!holdFired && (now - bothPressStart >= HOLD_DURATION_MS)) {
            holdFired  = true;
            deviceMode = isCalib ? DeviceMode::STREAMING : DeviceMode::CALIBRATING;
            isCalib    = (deviceMode == DeviceMode::CALIBRATING);
        }
    } else {
        bothPressStart = 0;
        holdFired      = false;
    }

    if (isCalib) {
        bool upEdge   = (lastUp   == LOW && !upPressed);
        bool downEdge = (lastDown == LOW && !downPressed);

        if (upEdge && !downPressed) {
            currentCalibration += 0.0625f;
            calibChanged  = true;
            saveNotifyEnd = 0;
        }
        if (downEdge && !upPressed) {
            currentCalibration -= 0.0625f;
            if (currentCalibration < 0.0f) currentCalibration = 0.0f;
            calibChanged  = true;
            saveNotifyEnd = 0;
        }
    }
    lastUp   = upPressed   ? LOW : HIGH;
    lastDown = downPressed ? LOW : HIGH;

    // -- NVS Save --
    if (calibChanged && (now - lastSaveMs >= NVS_SAVE_DELAY_MS)) {
        preferences.putFloat("multiplier", currentCalibration);
        calibChanged = false;
        lastSaveMs   = now;
        if (isCalib) saveNotifyEnd = now + SAVE_NOTIFY_MS;
    }
    if (saveNotifyEnd != 0 && now >= saveNotifyEnd) {
        saveNotifyEnd = 0;
    }

    // =========================================================================
    // USB RECONNECT: reset filters + warmup (one-shot per connection)
    // =========================================================================
    if (usbConnected && !lastUsbConnected) {
        warmupDone = false;  // new connection → allow warmup once
    }
    lastUsbConnected = usbConnected;

    if (usbConnected && !warmupDone) {
        resetFilters();
        for (int warmup = 0; warmup < 4; warmup++) {
            size_t dummy;
            i2s_read(I2S_PORT, raw_i2s_buffer,
                     SAMPLES_SHORT * sizeof(int32_t), &dummy, portMAX_DELAY);
            for (int i = 0; i < SAMPLES_SHORT; i++)
                usb_packet.samples[i] = static_cast<float>(MIC_CONVERT(raw_i2s_buffer[i]));
            INMP441.filter(usb_packet.samples, usb_packet.samples, SAMPLES_SHORT);
            DC_BLOCKER.filter(usb_packet.samples, usb_packet.samples, SAMPLES_SHORT);
        }
        warmupDone = true;  // ← never runs again until next disconnect+reconnect
    }

    // =========================================================================
    // 1. AUDIO ACQUISITION & DSP
    // =========================================================================
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, raw_i2s_buffer,
             SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

    for (int i = 0; i < SAMPLES_SHORT; i++) {
        usb_packet.samples[i] = static_cast<float>(MIC_CONVERT(raw_i2s_buffer[i]));
    }

    INMP441.filter(usb_packet.samples, usb_packet.samples, SAMPLES_SHORT);
    DC_BLOCKER.filter(usb_packet.samples, usb_packet.samples, SAMPLES_SHORT);

    // =========================================================================
    // 2. BATTERY MONITOR (Coulomb Counting Integration)
    // =========================================================================
    if (now - battLastMs >= BATTERY_INTERVAL_MS) {
        unsigned long elapsed_ms = now - battLastMs;
        battLastMs = now; // update timer

        float tegangan_V = ina219.getBusVoltage_V();
        float arus_mA    = ina219.getCurrent_mA();
        
        float arus_A      = arus_mA / 1000.0f;
        float daya_W      = tegangan_V * arus_A;
        float selisih_jam = elapsed_ms / 3600000.0f;
        
        float energiSesi_Wh     = daya_W * selisih_jam;
        float kapasitasSesi_mAh = arus_mA * selisih_jam;
        
        energiTerpakai_Wh     += energiSesi_Wh;
        kapasitasTerpakai_mAh += kapasitasSesi_mAh;
        
        if (energiTerpakai_Wh < 0.0f) energiTerpakai_Wh = 0.0f;
        if (energiTerpakai_Wh > ENERGI_TOTAL_Wh) energiTerpakai_Wh = ENERGI_TOTAL_Wh;
        
        battPercent = 100.0f - (energiTerpakai_Wh / ENERGI_TOTAL_Wh) * 100.0f;
        
        // Clamp the percentage strictly between 0 and 100
        if (battPercent > 100.0f) battPercent = 100.0f;
        if (battPercent < 0.0f)   battPercent = 0.0f;
    }

    // =========================================================================
    // 3. USB CDC STREAMING
    // =========================================================================
    usb_packet.timestamp_ms           = now;
    usb_packet.latitude               = 0.0f;
    usb_packet.longitude              = 0.0f;
    // usb_packet.latitude               = gps.location.lat();
    // usb_packet.longitude              = gps.location.lng();
    usb_packet.calibration_multiplier = currentCalibration;
    usb_packet.battery_percentage     = battPercent;
    usb_packet.calibration_mode       = static_cast<uint8_t>(deviceMode);
    uint32_t sm = FRAME_START_MARKER;
    uint32_t em = FRAME_END_MARKER;

    Serial.write(reinterpret_cast<uint8_t*>(&sm), 4);
    Serial.write(reinterpret_cast<uint8_t*>(&usb_packet), sizeof(usb_packet_t));
    Serial.write(reinterpret_cast<uint8_t*>(&em), 4);
}