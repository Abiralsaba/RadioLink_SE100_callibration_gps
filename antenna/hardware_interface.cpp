#include "hardware_interface.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// ═══════════════════════════════════════════════════════
//  PIN & PERIPHERAL CONFIGURATION
// ═══════════════════════════════════════════════════════

// GPS (antenna-side) — Serial1
// Wiring: GPS TX → GPIO4, GPS RX → GPIO5
#define GPS_TX_GPIO 4
#define GPS_RX_GPIO 5
#define GPS_BAUD 9600

// Telemetry (rover coordinates) — Serial2, RX=D16, TX=D17
#define TELEM_RX_PIN 16
#define TELEM_TX_PIN 17
#define TELEM_BAUD 57600

// Compass HMC5883L — I2C, SDA=21, SCL=22
#define COMPASS_ADDR 0x1E
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// DC Motor — Sabertooth driver, pulse-based control
// Right (CW):  pulse 1460→2000  (speed 0→100%)
// Left  (CCW): pulse 1460→1000  (speed 0→100%)
// Stop:        pulse 1460
#define MOTOR1_PIN 18
#define MOTOR_PULSE_MIN 1000
#define MOTOR_PULSE_MAX 2000
#define MOTOR_PULSE_STOP 1460

// Motor position limits (virtual, wire-tangle prevention)
#define MOTOR_POS_MIN 0.0
#define MOTOR_POS_MAX 360.0
#define MOTOR_POS_BOOT 180.0

// Telemetry staleness — if no new rover data for this long, stop tracking
#define TELEM_STALE_MS 5000

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

// Cached antenna position
static double ant_lat = 0.0;
static double ant_lon = 0.0;

// Cached rover position
static double rov_lat = 0.0;
static double rov_lon = 0.0;
static bool rov_updated = false;
static unsigned long last_rover_update = 0; // Staleness tracking
static double last_recv_lat = 0.0; // For robust duplicate detection
static double last_recv_lon = 0.0;

// Compass state — I2C failure recovery
static double last_valid_heading = 0.0;
static unsigned long last_compass_warn = 0;
static bool _compass_healthy = false;

// Compass heading filter — circular EMA to eliminate jitter
// Raw heading is kept in last_valid_heading for position tracking.
// Filtered heading is returned to the control loop for smooth control.
static double filtered_heading = 0.0;
static bool heading_filter_init = false;
#define HEADING_ALPHA 0.4 // 0.0 = no update, 1.0 = no filtering

// Motor position tracking (wire-tangle prevention)
// Virtual position: boots at 180°, range [0°, 360°]
// Tracked by accumulating compass heading deltas
static double motor_pos = MOTOR_POS_BOOT;
static double prev_heading = 0.0;
static bool motor_pos_initialized = false;

// ═══════════════════════════════════════════════════════
//  MOTOR CONTROL (Sabertooth pulse-based)
//  Sends a single servo-style pulse (1000–2000µs).
//  Must be called repeatedly (~50Hz) to keep motor running.
// ═══════════════════════════════════════════════════════
static void send_motor_pulse(int microseconds) {
  microseconds = constrain(microseconds, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
  digitalWrite(MOTOR1_PIN, HIGH);
  delayMicroseconds(microseconds);
  digitalWrite(MOTOR1_PIN, LOW);
}

void stop_motor() { send_motor_pulse(MOTOR_PULSE_STOP); }

void rotate_motor_right(int speed) {
  speed = constrain(speed, 0, 100);
  int pulse = map(speed, 0, 100, MOTOR_PULSE_STOP, MOTOR_PULSE_MAX);
  send_motor_pulse(pulse);
}

void rotate_motor_left(int speed) {
  speed = constrain(speed, 0, 100);
  int pulse = map(speed, 0, 100, MOTOR_PULSE_STOP, MOTOR_PULSE_MIN);
  send_motor_pulse(pulse);
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

  String latStr = line.substring(latIdx + 4, commaIdx);
  String lonStr = line.substring(lonIdx + 4);
  latStr.trim();
  lonStr.trim();

  double lat = latStr.toDouble();
  double lon = lonStr.toDouble();

  if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 &&
      (lat != 0.0 || lon != 0.0)) {
    // Only refresh staleness timer when coordinate actually CHANGES.
    // Use last_recv_lat to check for changes so it doesn't false-trigger
    // if rov_lat was zeroed out by the timeout.
    if (fabs(lat - last_recv_lat) > 0.0000001 || fabs(lon - last_recv_lon) > 0.0000001) {
      last_rover_update = millis();
    }
    last_recv_lat = lat;
    last_recv_lon = lon;
    
    rov_lat = lat;
    rov_lon = lon;
    rov_updated = true;
  } else {
    Serial.println("[PARSER VALIDATION FAILED] " + String(lat, 6) + " / " +
                   String(lon, 6));
  }
}

// ═══════════════════════════════════════════════════════
//  INIT
// ═══════════════════════════════════════════════════════
void hardware_init() {

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

  // ── 4. DC Motor init (Sabertooth on GPIO 18)
  //    Send 50 stop pulses (~1 second) to arm the Sabertooth driver.
  pinMode(MOTOR1_PIN, OUTPUT);
  digitalWrite(MOTOR1_PIN, LOW);
  for (int i = 0; i < 50; i++) {
    send_motor_pulse(MOTOR_PULSE_STOP);
    delay(20);
  }

  Serial.println("[HW] Hardware initialized");
  Serial.printf("[HW] Motor position: %.1f° (home)\n", MOTOR_POS_BOOT);
  Serial.println("[HW] Waiting for GPS satellites...");
}

// ═══════════════════════════════════════════════════════
//  ANTENNA GPS
// ═══════════════════════════════════════════════════════
void get_antenna_coordinates(double *lat, double *lon) {

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

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

  while (telemSerial.available() > 0) {
    char c = (char)telemSerial.read();

    if (c == '\n' || c == '\r') {
      if (telemBuf.length() > 0) {
        parse_telemetry_line(telemBuf);
        telemBuf = "";
      }
    } else {
      telemBuf += c;
      if (telemBuf.length() > 100) {
        telemBuf = "";
      }
    }
  }

  // Staleness check: if no new telemetry for 5 seconds, clear rover position.
  // This stops the motor when the bridge/rover stops sending data.
  if (last_rover_update > 0 && (millis() - last_rover_update >= TELEM_STALE_MS)) {
    rov_lat = 0.0;
    rov_lon = 0.0;
  }

  *lat = rov_lat;
  *lon = rov_lon;
}

bool rover_coordinate_updated() {
  bool updated = rov_updated;
  rov_updated = false;
  return updated;
}

// ═══════════════════════════════════════════════════════
//  COMPASS HEADING
//  On I2C failure: returns last valid heading (motor should STOP)
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

    float cal_x = (x - offsetX) * scaleX;
    float cal_y = (y - offsetY) * scaleY;

    float headingRadians = atan2(-cal_y, cal_x);
    headingRadians += declinationAngle;
    headingRadians += (mountingOffsetDegrees * PI / 180.0f);

    if (headingRadians < 0)
      headingRadians += 2.0f * PI;
    if (headingRadians >= 2.0f * PI)
      headingRadians -= 2.0f * PI;

    last_valid_heading = (double)(headingRadians * 180.0f / PI);
    _compass_healthy = true;

    // Apply circular EMA filter for smooth control output.
    // Uses angular difference to avoid discontinuity at 0°/360°.
    if (!heading_filter_init) {
      filtered_heading = last_valid_heading;
      heading_filter_init = true;
    } else {
      double diff = last_valid_heading - filtered_heading;
      if (diff > 180.0) diff -= 360.0;
      if (diff < -180.0) diff += 360.0;
      filtered_heading += HEADING_ALPHA * diff;
      if (filtered_heading < 0.0) filtered_heading += 360.0;
      if (filtered_heading >= 360.0) filtered_heading -= 360.0;
    }

  } else {
    _compass_healthy = false;
    if (millis() - last_compass_warn >= 2000) {
      last_compass_warn = millis();
      Serial.printf("[COMPASS] I2C read failed — holding last heading: %.1f°\n",
                    filtered_heading);
    }
  }

  return filtered_heading;
}

bool is_compass_healthy() { return _compass_healthy; }

// ═══════════════════════════════════════════════════════
//  MOTOR POSITION TRACKING
//  Accumulates compass heading deltas to track how far
//  the motor has turned from boot (virtual 0–360° range).
//  CRITICAL: if compass fails, position is NOT updated
//  and the motor should be stopped by the caller.
// ═══════════════════════════════════════════════════════
void update_motor_position() {

  if (!_compass_healthy) {
    return; // Can't track position without valid compass data
  }

  double heading = last_valid_heading; // Use the latest reading

  if (!motor_pos_initialized) {
    // First valid compass reading — capture boot heading
    prev_heading = heading;
    motor_pos_initialized = true;
    Serial.printf("[MOTOR] Boot heading captured: %.1f°\n", heading);
    return;
  }

  // Compute heading delta (normalized to [-180, +180])
  double delta = heading - prev_heading;
  if (delta > 180.0)
    delta -= 360.0;
  if (delta < -180.0)
    delta += 360.0;

  motor_pos += delta;

  // Clamp to [0, 360] — hard limits for wire-tangle prevention
  if (motor_pos < MOTOR_POS_MIN)
    motor_pos = MOTOR_POS_MIN;
  if (motor_pos > MOTOR_POS_MAX)
    motor_pos = MOTOR_POS_MAX;

  prev_heading = heading;
}

double get_motor_position() { return motor_pos; }