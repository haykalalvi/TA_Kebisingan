#include <HardwareSerial.h>
#include <TinyGPSPlus.h> 
#include <Arduino.h>
#include <driver/i2s.h>
#include "sos-iir-filter.h"

/* ===================== AUDIO CONFIG ===================== */
#define SAMPLE_RATE       48000
#define SAMPLE_BITS       32
#define MIC_BITS          24
#define SAMPLES_SHORT     (SAMPLE_RATE / 8)   // 125 ms = 6000 samples
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))

/* ===================== I2S PINS ===================== */
#define I2S_WS   9
#define I2S_SCK  39
#define I2S_SD   15 //8
#define I2S_PORT I2S_NUM_0

/* ===================== MIC PARAM ===================== */
#define MIC_SENSITIVITY   -26
#define MIC_REF_DB        94.0
#define MIC_OFFSET_DB     3.0103

constexpr double MIC_REF_AMPL =
pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

/*====================== GPS PARAM ===================== */
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// Buat objek untuk Serial 2 (GPS)
HardwareSerial gpsSerial(2);
// Buat objek untuk TinyGPSPlus
TinyGPSPlus gps;

/* ===================== FILTERS ===================== */
SOS_IIR_Filter DC_BLOCKER = {
  1.0,
  {
    {-1.0, 0.0, 0.9992, 0}
  }
};

SOS_IIR_Filter INMP441 = {
  1.001978346553777,
  {
    {-1.986920458359494, 0.986963226961561,
     1.995178510521705, -0.995184322211588}
  }
};

/* ===================== BUFFERS ===================== */
static float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

/* ===================== USB CDC PACKET ===================== */
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  float latitude;
  float longitude;
  float samples[SAMPLES_SHORT];
} usb_packet_t;

usb_packet_t usb_packet;

/* ===================== FRAME MARKERS ===================== */
#define FRAME_START_MARKER  0xAA55AA55
#define FRAME_END_MARKER    0x55AA55AA

/* ===================== PERFORMANCE MONITORING ===================== */
unsigned long last_send_time = 0;
unsigned long total_packets = 0;
unsigned long total_time = 0;

/* ===================== I2S INIT ===================== */
void mic_i2s_init() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format =
      (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

/* ===================== OPTIMIZED USB CDC SEND ===================== */
void usb_send_packet(const usb_packet_t* packet) {
  uint32_t start_marker = FRAME_START_MARKER;
  uint32_t end_marker = FRAME_END_MARKER;
  
  // Pre-calculate total size
  size_t total_size = sizeof(start_marker) + sizeof(usb_packet_t) + sizeof(end_marker);
  
  // Reserve buffer space (hint to USB stack)
  //Serial.setTxBufferSize(total_size);
  
  // Send all data without intermediate flushes
  Serial.write((uint8_t*)&start_marker, sizeof(start_marker));
  Serial.write((uint8_t*)packet, sizeof(usb_packet_t));
  Serial.write((uint8_t*)&end_marker, sizeof(end_marker));
  
  // Serial.flush();  // biarin usb automatically yang handle aja
}

/* ===================== SETUP ===================== */
void setup() {
  // Initialize USB CDC Serial with larger buffer
  Serial.begin(115200);
  //Serial.setTxBufferSize(32768);  // Larger TX buffer
  
  // Wait for USB connection
  while (!Serial) {
    delay(10);
  }
  
  delay(1000);
  
  // Pastiin CPU di max speed
  setCpuFrequencyMhz(240); 
  
  mic_i2s_init();
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  Serial.println("ESP32-S3 USB CDC TX READY - OPTIMIZED");
  
  // Calculate frame info
  size_t frame_size = 4 + sizeof(usb_packet_t) + 4;
  Serial.printf("Frame size: %d bytes\n", frame_size);
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("Expected rate: 8 packets/sec (125ms per frame)\n");
  
  last_send_time = millis();
}

/* ===================== OPTIMIZED MAIN LOOP ===================== */
void loop() {
  unsigned long loop_start = micros();
  
  // Read I2S data
  size_t bytes_read = 0;
  i2s_read(
    I2S_PORT,
    samples,
    SAMPLES_SHORT * sizeof(int32_t),
    &bytes_read,
    portMAX_DELAY
  );
  unsigned long i2s_time = micros() - loop_start;

  // Convert int → float (optimized)
  /*unsigned long convert_start = micros();
  int32_t* raw = (int32_t*)samples;
  for (int i = 0; i < SAMPLES_SHORT; i++) {
    samples[i] = MIC_CONVERT(raw[i]);
  }
  unsigned long convert_time = micros() - convert_start;

  // Apply filters
  unsigned long filter_start = micros();
  DC_BLOCKER.filter(samples, samples, SAMPLES_SHORT);
  INMP441.filter(samples, samples, SAMPLES_SHORT);
  unsigned long filter_time = micros() - filter_start;
  */

  // Convert int → float (optimized)
  unsigned long convert_start = micros();
  int32_t* raw = (int32_t*)samples;
  for (int i = 0; i < SAMPLES_SHORT; i++) {
    samples[i] = MIC_CONVERT(raw[i]);
  }
  unsigned long convert_time = micros() - convert_start;

  // --- TAMBAHAN UNTUK DEMONSTRASI FILTER ---
  // 1. Simpan beberapa sampel mentah untuk perbandingan
  float raw_samples_preview[5];
  for(int i = 0; i < 5; i++) raw_samples_preview[i] = samples[i];
  
  // 2. Hitung nilai Rata-rata (DC Offset) sebelum filter
  double sum_raw = 0;
  for (int i = 0; i < SAMPLES_SHORT; i++) sum_raw += samples[i];
  float dc_offset_before = sum_raw / SAMPLES_SHORT;
  // -----------------------------------------

  // Apply filters
  unsigned long filter_start = micros();
  DC_BLOCKER.filter(samples, samples, SAMPLES_SHORT);
  INMP441.filter(samples, samples, SAMPLES_SHORT);
  unsigned long filter_time = micros() - filter_start;

  // --- TAMBAHAN UNTUK DEMONSTRASI FILTER ---
  // 3. Hitung nilai Rata-rata (DC Offset) setelah filter
  double sum_filtered = 0;
  for (int i = 0; i < SAMPLES_SHORT; i++) sum_filtered += samples[i];
  float dc_offset_after = sum_filtered / SAMPLES_SHORT;

  // 4. Print perbandingan (hanya setiap 100 paket agar tidak membanjiri Serial)
  /*
  if ((total_packets + 1) % 100 == 0) { 
    Serial.println("\n--- FILTER DEMONSTRATION ---");
    Serial.printf("DC Offset Sebelum Filter : %.2f\n", dc_offset_before);
    Serial.printf("DC Offset Sesudah Filter : %.2f\n", dc_offset_after);
    Serial.println("Cuplikan 5 Sampel Pertama:");
    Serial.println("Index | Mentah      | Terfilter");
    for(int i = 0; i < 5; i++) {
      Serial.printf("  %d   | %10.2f | %10.2f\n", i, raw_samples_preview[i], samples[i]);
    }
    Serial.println("----------------------------");
  }
  */
  // -----------------------------------------
  // Baca GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Fill packet
  unsigned long packet_start = micros();
  usb_packet.timestamp_ms = millis();
  usb_packet.latitude  = gps.location.lat();
  usb_packet.longitude = gps.location.lng();
  memcpy(usb_packet.samples, samples, sizeof(samples));
  unsigned long packet_time = micros() - packet_start;

  // Send via USB CDC
  unsigned long send_start = micros();
  usb_send_packet(&usb_packet);
  unsigned long send_time = micros() - send_start;

  // Performance monitoring (tiap 100 packets)
  total_packets++;
  unsigned long loop_total = micros() - loop_start;
  total_time += loop_total;
  /*
  if (total_packets % 100 == 0) {
    unsigned long now = millis();
    float actual_rate = 100000.0 / (now - last_send_time);  // packets per second
    float avg_loop_time = total_time / 100.0;
    
    Serial.printf("\n=== Performance Stats (Packet #%lu) ===\n", total_packets);
    Serial.printf("Actual rate: %.2f packets/sec\n", actual_rate);
    Serial.printf("Avg loop time: %.2f ms\n", avg_loop_time / 1000.0);
    Serial.printf("  - I2S read: %.2f ms\n", i2s_time / 1000.0);
    Serial.printf("  - Convert: %.2f ms\n", convert_time / 1000.0);
    Serial.printf("  - Filter: %.2f ms\n", filter_time / 1000.0);
    Serial.printf("  - Packet prep: %.2f ms\n", packet_time / 1000.0);
    Serial.printf("  - USB send: %.2f ms\n", send_time / 1000.0);
    Serial.printf("Target: 125 ms/packet (8 packets/sec)\n\n");
    
    last_send_time = now;
    total_time = 0;
  }
  */
}