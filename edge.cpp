/**
 * =============================================================================
 * ESP32-S3 (Super-Loop / Single Core Architecture)
 * Audio Streaming + GPS + Calibration + INA219 Battery Monitor
 * =============================================================================
 *
 * ARCHITECTURE CHANGES:
 *   • Removed FreeRTOS tasks (audioDSPTask, systemTask), Queues, and Mutexes.
 *   • Removed atomic variables (no longer needed for single-thread).
 *   • Everything runs synchronously in the main Arduino loop().
 *   • I2S reads directly, processes, checks system state, and writes to USB sequentially.
 * =============================================================================
 */

#include <HardwareSerial.h>
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
#define SAMPLES_SHORT  (SAMPLE_RATE / 8)          // 125 ms = 6000 samples
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
TinyGPSPlus    gps;
HardwareSerial gpsSerial(1);
Preferences    preferences;

// Single-thread state variables
DeviceMode deviceMode = DeviceMode::STREAMING;
float      currentCalibration = 1.0f;

/* --- USB packet ----------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    latitude;
    float    longitude;
    float    calibration_multiplier;   
    float    battery_percentage;      
    float    samples[SAMPLES_SHORT];
} usb_packet_t;

static usb_packet_t usb_packet;

// Raw buffer for I2S read
static int32_t raw_i2s_buffer[SAMPLES_SHORT] __attribute__((aligned(4)));

/* ─── IIR FILTERS ────────────────────────────────────────────────────────── */
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

// Safety function to send dta
void send_all_bytes(uint8_t* data, size_t total_length) {
    size_t bytes_written = 0;
    while (bytes_written < total_length) {
        size_t w = Serial.write(data + bytes_written, total_length - bytes_written);
        if (w == 0) {
            delay(1); 
        } else {
            bytes_written += w;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════════ */
void setup() {
    Serial.setTxBufferSize(32768); 
    Serial.begin(115200);
    setCpuFrequencyMhz(240);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!ina219.begin()) {
        // Serial.println("INA219 Not Found.");
    }

    pinMode(BTN_UP,   INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);

    preferences.begin("audio-calib", false);
    currentCalibration = preferences.getFloat("multiplier", 1.0f);

    mic_i2s_init();
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
}

/* ═══════════════════════════════════════════════════════════════════════════
   MAIN LOOP (Synchronous Super-Loop)
   ═══════════════════════════════════════════════════════════════════════════ */
void loop() {
    static float         battVoltage    = 0.0f;
    static float         battCurrent_mA = 0.0f;
    static float         battPercent    = 0.0f;
    static float         mAh_used       = 0.0f;
    static unsigned long battLastMs     = millis();

    static bool          lastUp         = HIGH;
    static bool          lastDown       = HIGH;
    static unsigned long bothPressStart = 0;
    static bool          holdFired      = false;

    static bool          calibChanged   = false;
    static unsigned long lastSaveMs     = 0;
    static unsigned long saveNotifyEnd  = 0;

    unsigned long now = millis();

    // =========================================================================
    // 1. AUDIO ACQUISITION & DSP (Blocks until 125ms of data is ready)
    // =========================================================================
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, raw_i2s_buffer, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

    // Convert directly into the USB packet struct to save memory
    for (int i = 0; i < SAMPLES_SHORT; i++) {
        usb_packet.samples[i] = static_cast<float>(MIC_CONVERT(raw_i2s_buffer[i]));
    }

    DC_BLOCKER.filter(usb_packet.samples, usb_packet.samples, SAMPLES_SHORT);
    INMP441.filter(usb_packet.samples, usb_packet.samples, SAMPLES_SHORT);

    // =========================================================================
    // 2. SYSTEM LOGIC (Battery, Buttons, GPS, NVS)
    // =========================================================================
    
    // -- Battery Monitor --
    if (now - battLastMs >= BATTERY_INTERVAL_MS) {
        unsigned long elapsed_ms = now - battLastMs;
        battLastMs = now;
        
        battVoltage    = ina219.getBusVoltage_V();
        battCurrent_mA = ina219.getCurrent_mA();
        mAh_used      += battCurrent_mA * (elapsed_ms / 3600000.0f);

        // Dummy data options (e.g., a healthy 11.4V drawing 250mA)
        //battVoltage    = 11.4f;  
        //battCurrent_mA = 250.0f;

        battPercent = ((battVoltage - BATT_V_MIN) / (BATT_V_MAX - BATT_V_MIN)) * 100.0f;
        if (battPercent > 100.0f) battPercent = 100.0f;
        if (battPercent < 0.0f)   battPercent = 0.0f;
    }

    // -- Button State --
    bool upPressed    = (digitalRead(BTN_UP)   == LOW);
    bool downPressed  = (digitalRead(BTN_DOWN) == LOW);
    bool isCalib      = (deviceMode == DeviceMode::CALIBRATING);

    // Hold both buttons
    if (upPressed && downPressed) {
        if (bothPressStart == 0) {
            bothPressStart = now;
            holdFired      = false;
        } else if (!holdFired && (now - bothPressStart >= HOLD_DURATION_MS)) {
            holdFired = true;
            deviceMode = isCalib ? DeviceMode::STREAMING : DeviceMode::CALIBRATING;
            isCalib = (deviceMode == DeviceMode::CALIBRATING);
        }
    } else {
        bothPressStart = 0;
        holdFired      = false;
    }

    // Single edge detection for Calibration
    if (isCalib) {
        bool upEdge   = (lastUp   == LOW && !upPressed);
        bool downEdge = (lastDown == LOW && !downPressed);

        if (upEdge && !downPressed) {
            currentCalibration += 0.25f;
            calibChanged  = true;
            saveNotifyEnd = 0;
        }
        if (downEdge && !upPressed) {
            currentCalibration -= 0.25f;
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

    // -- GPS Feed --
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // =========================================================================
    // 3. USB CDC STREAMING
    // =========================================================================
    usb_packet.timestamp_ms           = now;
    usb_packet.latitude               = gps.location.lat();
    usb_packet.longitude              = gps.location.lng();
    usb_packet.calibration_multiplier = currentCalibration;
    usb_packet.battery_percentage     = battPercent;

    uint32_t sm = FRAME_START_MARKER;
    uint32_t em = FRAME_END_MARKER;
    
    Serial.write(reinterpret_cast<uint8_t *>(&sm), 4);
    Serial.write(reinterpret_cast<uint8_t *>(&usb_packet), sizeof(usb_packet_t));
    Serial.write(reinterpret_cast<uint8_t *>(&em), 4);

    // Opsi makai fungsi send all bytes (safety option)
    // send_all_bytes(reinterpret_cast<uint8_t*>(&sm), 4);
    // send_all_bytes(reinterpret_cast<uint8_t*>(&usb_packet), sizeof(usb_packet_t));
    // send_all_bytes(reinterpret_cast<uint8_t*>(&em), 4);
}