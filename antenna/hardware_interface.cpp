#include "hardware_interface.h"
#include "Last_Wish.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// ═══════════════════════════════════════════════════════
//  PIN & PERIPHERAL CONFIGURATION
// ═══════════════════════════════════════════════════════

// GPS (antenna-side) — Serial1
// Wiring: GPS TX → GPIO4, GPS RX → GPIO5
#define GPS_TX_GPIO 4 // GPS module TX connects here (ESP32 receives)
#define GPS_RX_GPIO 5 // GPS module RX connects here (ESP32 transmits)
#define GPS_BAUD 9600

// Telemetry (rover coordinates) — Serial2, RX=D16, TX=D17
#define TELEM_RX_PIN 16
#define TELEM_TX_PIN 17
#define TELEM_BAUD 57600

// Compass HMC5883L — I2C, SDA=21, SCL=22
#define COMPASS_ADDR 0x1E
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Servo — Pin 14, LEDC channel 0
#define SERVO_PIN 14
#define LEDC_CH 0
#define PWM_FREQ 50      // Hz
#define PWM_RES 16       // bits
#define PULSE_MIN 801.0  // µs at 0°
#define PULSE_MAX 1641.0 // µs at 180°
#define ANGLE_MIN 0.0
#define ANGLE_MAX 180.0
const int step = 3; // µs per sweep increment (smooth motion)

// Power-loss detection — voltage divider midpoint
#define SAVE_PIN 33

// NVS auto-save delay (ms after servo stabilizes)
#define SAVE_DELAY_MS 3000

// ═══════════════════════════════════════════════════════
//  COMPASS CALIBRATION (from GPS_Compass_main.ino)
// ═══════════════════════════════════════════════════════
static const float offsetX = -3.50;
static const float offsetY = 8.50;
static const float scaleX = 1.05;
static const float scaleY = 0.99;

static const float declinationAngle = -0.0089;
static const float mountingOffsetDegrees = -88.0;

// ═══════════════════════════════════════════════════════
//  OBJECTS & STATE
// ═══════════════════════════════════════════════════════

// GPS — UART1
static TinyGPSPlus gps;
static HardwareSerial gpsSerial(1);

// Telemetry — UART2 (native for GPIO 16/17)
static HardwareSerial telemSerial(2);
static String telemBuf = "";

// NVS
static Preferences prefs;

// Cached antenna position
static double ant_lat = 0.0;
static double ant_lon = 0.0;

// Cached rover position
static double rov_lat = 0.0;
static double rov_lon = 0.0;

// Servo state
static double cur_servo_angle = 90.0;
static volatile int cur_servo_us =
    1221; // Live PWM µs — Last_Wish ISR reads this
static bool servo_save_pending = false;
static unsigned long servo_stop_time = 0;

// Compass state — I2C failure recovery
static double last_valid_heading =
    0.0; // Cached heading for I2C failure fallback
static unsigned long last_compass_warn = 0; // Rate-limit warning messages

// ═══════════════════════════════════════════════════════
//  SERVO HELPERS  (from servo.ino calibration)
// ═══════════════════════════════════════════════════════
static float angle_to_pulse(float a) {
  if (a < ANGLE_MIN)
    a = ANGLE_MIN;
  if (a > ANGLE_MAX)
    a = ANGLE_MAX;
  return PULSE_MIN + (a / (float)ANGLE_MAX) * (PULSE_MAX - PULSE_MIN);
}

static float pulse_to_angle(float us) {
  if (us < PULSE_MIN)
    us = PULSE_MIN;
  if (us > PULSE_MAX)
    us = PULSE_MAX;
  return ((us - PULSE_MIN) / (PULSE_MAX - PULSE_MIN)) * (float)ANGLE_MAX;
}

static uint32_t pulse_to_duty(float us) {
  return (uint32_t)((us / 20000.0f) * ((1 << PWM_RES) - 1));
}

// Low-level PWM write — centralized API version guard
static void write_servo_duty(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(LEDC_CH, duty);
#endif
}

// Writes servo angle and keeps cur_servo_us in sync for Last_Wish ISR
static void write_servo_raw(float angle) {
  float us = angle_to_pulse(angle);
  cur_servo_us = (int)us;
  write_servo_duty(pulse_to_duty(us));
}

// ═══════════════════════════════════════════════════════
//  TELEMETRY PARSER
//  Expected format from rover:  "lat:<value>,lon:<value>\n"
//  Example: "lat:23.8103000,lon:90.4125000\n"
// ═══════════════════════════════════════════════════════
static void parse_telemetry_line(const String &line) {
  int latIdx = line.indexOf("lat:");
  int lonIdx = line.indexOf("lon:");

  if (latIdx < 0 || lonIdx < 0) {
    Serial.println("[PARSER ERROR] Missing 'lat:' or 'lon:'. Line was: " +
                   line);
    return;
  }

  int commaIdx = line.indexOf(',', latIdx);
  if (commaIdx < 0) {
    Serial.println("[PARSER ERROR] Missing comma. Line was: " + line);
    return;
  }

  // Extract value after "lat:" up to the comma
  String latStr = line.substring(latIdx + 4, commaIdx);
  // Extract value after "lon:" to end of line
  String lonStr = line.substring(lonIdx + 4);
  latStr.trim();
  lonStr.trim();

  double lat = latStr.toDouble();
  double lon = lonStr.toDouble();

  // Basic validity check
  if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 &&
      (lat != 0.0 || lon != 0.0)) {
    rov_lat = lat;
    rov_lon = lon;
    Serial.println("[PARSER SUCCESS] Parsed Lat: " + String(lat, 6) +
                   " Lon: " + String(lon, 6));
  } else {
    Serial.println(
        "[PARSER VALIDATION FAILED] Lat/Lon out of bounds or Zero. " +
        String(lat, 6) + " / " + String(lon, 6));
  }
}

// ═══════════════════════════════════════════════════════
//  INIT
// ═══════════════════════════════════════════════════════
void hardware_init() {

  // ── 0. LAST WISH — Power-loss servo protection ──────────
  //    Must init FIRST so the ISR can protect the servo
  //    position even if power drops during the init sweep.
  //    Monitors cur_servo_us (volatile int) via pointer.
  LastWish_begin(SAVE_PIN, cur_servo_us);

  // ── 1. GPS on Serial1 (GPS TX→GPIO4, GPS RX→GPIO5, 9600 baud)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_TX_GPIO, GPS_RX_GPIO);

  // ── 2. Telemetry on Serial2 (RX=D16, TX=D17, 57600 baud)
  telemSerial.begin(TELEM_BAUD, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);

  // ── 3. Compass I2C (SDA=21, SCL=22, HMC5883L @ 0x1E)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x00);
  Wire.write(0x70); // 8-average, 15 Hz, normal measurement
  Wire.endTransmission();

  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x01);
  Wire.write(0x20); // Gain = 1090 LSb/Gauss
  Wire.endTransmission();

  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x02);
  Wire.write(0x00); // Continuous-measurement mode
  Wire.endTransmission();

  // ── 4. Servo LEDC (Pin 14, 50 Hz, 16-bit, 801–1641 µs)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES);
#else
  ledcSetup(LEDC_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(SERVO_PIN, LEDC_CH);
#endif

  // ── 5. Restore servo position ──────────────────────────
  //    Priority: Last_Wish (power-loss save) → Preferences (debounced save) →
  //    default 90° Last_Wish stores µs as uint32, Preferences stores degrees as
  //    float.
  prefs.begin("servo", false);

  int loaded_us = LastWish_load((int)angle_to_pulse(90.0f));

  if (loaded_us >= (int)PULSE_MIN && loaded_us <= (int)PULSE_MAX) {
    // Last_Wish had a valid saved position (power-loss recovery)
    cur_servo_angle = (double)pulse_to_angle((float)loaded_us);
    cur_servo_us = loaded_us;
    Serial.printf("[LastWish] Restored: %d µs → %.1f°\n", loaded_us,
                  cur_servo_angle);
  } else {
    // Fallback to Preferences (debounced save from normal operation)
    float saved = prefs.getFloat("ang", -1.0);
    if (saved >= ANGLE_MIN && saved <= ANGLE_MAX) {
      cur_servo_angle = (double)saved;
      cur_servo_us = (int)angle_to_pulse(saved);
      Serial.printf("[NVS] Restored: %.1f°\n", saved);
    } else {
      cur_servo_angle = 90.0;
      cur_servo_us = (int)angle_to_pulse(90.0f);
      Serial.println("[NVS] No save — default 90.0°");
    }
  }

  // Start servo at restored position (no jump)
  write_servo_duty(pulse_to_duty((float)cur_servo_us));
  delay(300);

  Serial.printf("[LastWish] Loaded position: %d µs\n", cur_servo_us);

  // ── 6. Sweep from restored position → 90° (home) ──────
  //    Uses µs-based stepping for precise motion control.
  //    cur_servo_us stays in sync during sweep so Last_Wish
  //    can save the correct position if power drops mid-sweep.
  int target_us = (int)angle_to_pulse(90.0f);

  if (abs(cur_servo_us - target_us) > step) {
    Serial.printf("[HW] Sweeping %.1f° → 90.0°...\n", cur_servo_angle);

    while (abs(cur_servo_us - target_us) > step) {
      if (cur_servo_us < target_us) {
        cur_servo_us += step;
      } else {
        cur_servo_us -= step;
      }
      write_servo_duty(pulse_to_duty((float)cur_servo_us));
      delay(15);
    }

    // Final position exactly at home
    cur_servo_us = target_us;
    write_servo_duty(pulse_to_duty((float)cur_servo_us));
    cur_servo_angle = 90.0;
    Serial.println("[HW] Sweep complete");
  }

  delay(200);
  Serial.println("[HW] Hardware initialized");
  Serial.println("[HW] Waiting for GPS satellites...");
}

// ═══════════════════════════════════════════════════════
//  ANTENNA GPS
// ═══════════════════════════════════════════════════════
void get_antenna_coordinates(double *lat, double *lon) {

  // Feed GPS parser with all available bytes
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Update cached position when valid
  if (gps.location.isValid() && gps.location.isUpdated()) {
    ant_lat = gps.location.lat();
    ant_lon = gps.location.lng();
  }

  *lat = ant_lat;
  *lon = ant_lon;
}

// ═══════════════════════════════════════════════════════
//  ROVER GPS (TELEMETRY)
// ═══════════════════════════════════════════════════════
void get_rover_coordinates(double *lat, double *lon) {

  // Read and parse all available telemetry bytes
  while (telemSerial.available() > 0) {
    char c = (char)telemSerial.read();

    // Uncomment the line below to watch raw telemetry bytes arrive in the
    // Serial Monitor. If you see weird symbols, your baud rate (9600 vs 57600)
    // is mismatched. If you see absolutely nothing, check your RX/TX pins (they
    // might be swapped).
    // Serial.print(c);

    if (c == '\n' || c == '\r') {
      if (telemBuf.length() > 0) {
        parse_telemetry_line(telemBuf);
        telemBuf = "";
      }
    } else {
      telemBuf += c;
      // Safety: prevent buffer overflow from malformed data
      if (telemBuf.length() > 100) {
        telemBuf = "";
      }
    }
  }

  *lat = rov_lat;
  *lon = rov_lon;
}

// ═══════════════════════════════════════════════════════
//  COMPASS HEADING
//  On I2C failure: returns last valid heading (servo holds position)
//  and logs a rate-limited warning to Serial Monitor.
// ═══════════════════════════════════════════════════════
double get_antenna_heading() {

  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(COMPASS_ADDR, 6);

  if (Wire.available() == 6) {
    int16_t x = (Wire.read() << 8) | Wire.read();
    int16_t z = (Wire.read() << 8) | Wire.read();
    int16_t y = (Wire.read() << 8) | Wire.read();

    // Apply calibration
    float cal_x = (x - offsetX) * scaleX;
    float cal_y = (y - offsetY) * scaleY;

    // Compute heading with declination and mounting offset
    float headingRadians = atan2(-cal_y, cal_x);
    headingRadians += declinationAngle;
    headingRadians += (mountingOffsetDegrees * PI / 180.0f);

    // Normalize to [0, 2π)
    if (headingRadians < 0)
      headingRadians += 2.0f * PI;
    if (headingRadians >= 2.0f * PI)
      headingRadians -= 2.0f * PI;

    // Valid reading — update cache
    last_valid_heading = (double)(headingRadians * 180.0f / PI);

  } else {
    // I2C failure — log warning (rate-limited to avoid flooding)
    if (millis() - last_compass_warn >= 2000) {
      last_compass_warn = millis();
      Serial.printf("[COMPASS] I2C read failed — holding last heading: %.1f°\n",
                    last_valid_heading);
    }
  }

  return last_valid_heading;
}

// ═══════════════════════════════════════════════════════
//  SERVO CURRENT ANGLE
// ═══════════════════════════════════════════════════════
double get_servo_angle() { return cur_servo_angle; }

// ═══════════════════════════════════════════════════════
//  SET SERVO ANGLE
//  Writes directly — the tracking math LPF provides
//  smooth transitions, so no additional motion buffer needed.
// ═══════════════════════════════════════════════════════
void set_servo_angle(double angle) {

  if (angle < ANGLE_MIN)
    angle = ANGLE_MIN;
  if (angle > ANGLE_MAX)
    angle = ANGLE_MAX;

  write_servo_raw((float)angle);

  // Debounced NVS save — only writes flash when angle
  // has been stable for SAVE_DELAY_MS to avoid flash wear
  if (fabs(angle - cur_servo_angle) > 0.1) {
    servo_save_pending = true;
    servo_stop_time = millis();
  }

  cur_servo_angle = angle;

  if (servo_save_pending && (millis() - servo_stop_time >= SAVE_DELAY_MS)) {
    prefs.putFloat("ang", (float)cur_servo_angle);
    servo_save_pending = false;
  }
}