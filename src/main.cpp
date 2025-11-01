#include <Arduino.h>
#ifndef ESP32
  #error This example runs on ESP32
#endif

// --- LIDAR Pin Config ---
const uint8_t LIDAR_GPIO_TX = 21;   // Lidar TX -> ESP32 RX
const uint8_t LIDAR_GPIO_PWM = 15;  // PWM motor pin

// --- Lidar Model ---
#define XIAOMI_LDS02RR

// --- Debug Options ---
#define DEBUG_MISSED_ANGLES
//#define DEBUG_PACKETS

// --- Core Settings ---
const uint32_t SERIAL_MONITOR_BAUD = 250000;
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t  LIDAR_PWM_BITS = 11;
const uint8_t  LIDAR_PWM_CHANNEL = 2;
const uint16_t LIDAR_SERIAL_RX_BUF_LEN = 8192;  // boosted
const uint16_t PRINT_EVERY_NTH_POINT = 50;
const uint16_t HEX_DUMP_WIDTH = 16;

#define MAX_SCAN_POINTS 360
float scan_angles[MAX_SCAN_POINTS];
float scan_distances[MAX_SCAN_POINTS];
int scan_point_count = 0;

#include "lds_all_models.h"

// Prototypes
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed);
void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed);
size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length);
int lidar_serial_read_callback();
void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin);
void lidar_info_callback(LDS::info_t code, String info);
void lidar_error_callback(LDS::result_t code, String aux_info);

HardwareSerial LidarSerial(1);
LDS *lidar;

void setupLidar() {
  lidar = new LDS_LDS02RR();

  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setMotorPinCallback(lidar_motor_pin_callback);
  lidar->setInfoCallback(lidar_info_callback);
  lidar->setErrorCallback(lidar_error_callback);

  // RX only (Lidar TX → ESP32 RX)
  LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_TX, -1);
  LidarSerial.setRxBufferSize(LIDAR_SERIAL_RX_BUF_LEN);
  LidarSerial.flush();

  lidar->init();
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  Serial.println("\n=== LiDAR Startup ===");

  setupLidar();

  Serial.print("LiDAR model: ");
  Serial.println(lidar->getModelName());
  Serial.print("Baud rate: ");
  Serial.println(lidar->getSerialBaudRate());

  LDS::result_t result = lidar->start();
  Serial.print("Start result: ");
  Serial.println(lidar->resultCodeToString(result));

  lidar->setScanTargetFreqHz(5.0f);
}

int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  // TX-only config: ignore outgoing data
  return length;
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  static int i = 0;
  static float last_angle = -1;

  // Normalize angle
  if (angle_deg < 0) angle_deg += 360.0f;
  if (angle_deg >= 360.0f) angle_deg -= 360.0f;

  if (scan_point_count < MAX_SCAN_POINTS) {
    scan_angles[scan_point_count] = angle_deg;
    scan_distances[scan_point_count] = distance_mm;
    scan_point_count++;
  } else {
    // Overflow protection
    scan_point_count = MAX_SCAN_POINTS - 1;
  }

#ifdef DEBUG_MISSED_ANGLES
  if (last_angle >= 0) {
    float diff = angle_deg - last_angle;
    if (diff < 0) diff += 360;
    if (diff > 5.0f) {
      Serial.print("⚠ Missed angle gap: ");
      Serial.print(diff);
      Serial.print("° near ");
      Serial.println(angle_deg);
    }
  }
  last_angle = angle_deg;
#endif

  if (scan_completed) {
    Serial.println("SCAN_START");
    for (int j = 0; j < scan_point_count; j++) {
      Serial.print(scan_angles[j]);
      Serial.print(",");
      Serial.println(scan_distances[j]);
    }
    Serial.println("SCAN_END");

    scan_point_count = 0;
    last_angle = -1;

    Serial.print("Scan completed; SPS: ");
    Serial.println(lidar->getCurrentScanFreqHz());
    LidarSerial.flush();
  }

  if (i++ % PRINT_EVERY_NTH_POINT == 0) {
    Serial.print(distance_mm);
    Serial.print(" mm @ ");
    Serial.println(angle_deg);
  }
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = LIDAR_GPIO_PWM;
  if (value <= (float)LDS::DIR_INPUT) {
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
#if ESP_IDF_VERSION_MAJOR < 5
      ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
#else
      ledcAttachChannel(pin, LIDAR_PWM_FREQ, LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL);
#endif
    } else {
      pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }

  if (value < (float)LDS::VALUE_PWM) {
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
  } else {
    int pwm_value = ((1 << LIDAR_PWM_BITS) - 1) * value;
#if ESP_IDF_VERSION_MAJOR < 5
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
#else
    ledcWriteChannel(LIDAR_PWM_CHANNEL, pwm_value);
#endif
  }
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("[INFO] ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("[ERROR] ");
  Serial.print(lidar->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
#ifdef DEBUG_PACKETS
  Serial.print("Packet len=");
  Serial.print(length);
  Serial.print(" completed=");
  Serial.println(scan_completed);
#endif
}

void loop() {
  lidar->loop();
  // No yield/delay → keep UART serviced continuously
  // Helps prevent angular data dropouts
}
