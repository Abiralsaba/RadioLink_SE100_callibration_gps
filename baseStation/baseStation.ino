// ═══════════════════════════════════════════════════════
//  BASE SIDE (Side 2) — Coordinate Relay
//
//  Receives rover coordinates from Python bridge
//  (send_rover_coordinate.py) via USB Serial at 115200.
//
//  Forwards them via telemetry to Side 1 (servo side)
//  using Serial2 at 9600 baud.
//
//  Bridge format:  "lat:<value>,lon:<value>\n"
//  Relay format:   "lat:<value>,lon:<value>\n"  (same)
//
//  Telemetry wiring:
//    ESP32 RX → D16
//    ESP32 TX → D17
// ═══════════════════════════════════════════════════════

#include <HardwareSerial.h>

// Telemetry — Serial2, RX=16, TX=17, 9600 baud
#define TELEM_RX_PIN 16
#define TELEM_TX_PIN 17
#define TELEM_BAUD 57600

HardwareSerial telemSerial(2);

// Parsed coordinates
double rLat = 0.0;
double rLon = 0.0;

// Serial monitor & Telemetry send timers
unsigned long lastPrint = 0;
unsigned long lastTelemSend = 0;

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  // USB Serial — receives data from Python bridge at 115200
  Serial.begin(115200);

  // Telemetry Serial — sends data to Side 1 at 9600
  telemSerial.begin(TELEM_BAUD, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);

  delay(1000);
  Serial.println("\n--- Base Relay (Side 2) Booted ---");
  Serial.println("Waiting for coordinates from bridge...");
}

// ═══════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════
void loop() {

  // Read from USB Serial (Python bridge)
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0)
      continue;

    // Parse "lat:<value>,lon:<value>"
    int latIdx = line.indexOf("lat:");
    int lonIdx = line.indexOf("lon:");

    if (latIdx < 0 || lonIdx < 0)
      continue;

    int commaIdx = line.indexOf(',', latIdx);
    if (commaIdx < 0)
      continue;

    String latStr = line.substring(latIdx + 4, commaIdx);
    String lonStr = line.substring(lonIdx + 4);
    latStr.trim();
    lonStr.trim();

    double lat = latStr.toDouble();
    double lon = lonStr.toDouble();

    // Validate
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
      continue;
    if (lat == 0.0 && lon == 0.0)
      continue;

    // Store
    rLat = lat;
    rLon = lon;
  }

  // --- 2. SEND TO TELEMETRY (Rate Limited to 5Hz) ---
  // Telemetry modules (9600 baud) can get overwhelmed if we instantly forward
  // every high-frequency ROS message. This ensures we safely send the latest.
  if (millis() - lastTelemSend >= 200) {
    lastTelemSend = millis();
    String out = "lat:" + String(rLat, 7) + ",lon:" + String(rLon, 7);
    telemSerial.println(out);
  }

  // --- 3. PRINT TO PC SERIAL MONITOR ---
  // Serial monitor — print status once per second
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.print("Relay: ");
    if (rLat != 0.0 || rLon != 0.0) {
      Serial.printf("lat:%.7f,lon:%.7f", rLat, rLon);
    } else {
      Serial.print("Waiting for bridge data");
    }
    Serial.println();
  }
}
