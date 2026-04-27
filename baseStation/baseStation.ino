// ═══════════════════════════════════════════════════════
//  BASE SIDE (Side 2) — Coordinate Relay
//
//  Receives rover coordinates from Python bridge
//  (send_rover_coordinate.py) via USB Serial at 115200.
//
//  Forwards them via telemetry to Side 1 (antenna side)
//  using Serial2 at 57600 baud.
//
//  Bridge format:  "lat:<value>,lon:<value>\n"
//  Relay format:   "lat:<value>,lon:<value>\n"  (same)
//
//  When the bridge stops sending, the relay STOPS
//  forwarding after 5 seconds (BRIDGE_TIMEOUT_MS).
//  This ensures the antenna stops tracking stale data.
//
//  Telemetry wiring:
//    ESP32 RX → D16
//    ESP32 TX → D17
// ═══════════════════════════════════════════════════════

#include <HardwareSerial.h>

// Telemetry — Serial2, RX=16, TX=17, 57600 baud
#define TELEM_RX_PIN 16
#define TELEM_TX_PIN 17
#define TELEM_BAUD 57600

// Bridge data timeout — stop relaying after 5 seconds of no bridge input
#define BRIDGE_TIMEOUT_MS 5000

HardwareSerial telemSerial(2);

// Parsed coordinates
double rLat = 0.0;
double rLon = 0.0;

// Timers
unsigned long lastPrint = 0;
unsigned long lastTelemSend = 0;
unsigned long lastBridgeData = 0; // When bridge last sent valid data

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  // USB Serial — receives data from Python bridge at 115200
  Serial.begin(115200);

  // Telemetry Serial — sends data to Side 1 at 57600
  telemSerial.begin(TELEM_BAUD, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);

  delay(1000);
  Serial.println("\n--- Base Relay (Side 2) Booted ---");
  Serial.println("Waiting for coordinates from bridge...");
}

// ═══════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════
void loop() {

  // --- 1. READ FROM USB SERIAL (Python bridge) ---
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

    // Store and mark as fresh
    rLat = lat;
    rLon = lon;
    lastBridgeData = millis();
  }

  // --- 2. SEND TO TELEMETRY (Rate Limited to 5Hz) ---
  // Only relay when bridge data is fresh (received within last 5 seconds).
  // When the bridge/rover stops sending, the relay STOPS — the antenna
  // will detect the silence and stop the motor.
  if (millis() - lastTelemSend >= 200) {
    lastTelemSend = millis();

    bool bridgeFresh =
        (lastBridgeData > 0 && (millis() - lastBridgeData < BRIDGE_TIMEOUT_MS));

    if (bridgeFresh) {
      String out = "lat:" + String(rLat, 7) + ",lon:" + String(rLon, 7);
      telemSerial.println(out);
    }
  }

  // --- 3. PRINT TO PC SERIAL MONITOR ---
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    bool bridgeFresh =
        (lastBridgeData > 0 && (millis() - lastBridgeData < BRIDGE_TIMEOUT_MS));

    Serial.print("Relay: ");
    if (bridgeFresh) {
      Serial.printf("lat:%.7f,lon:%.7f", rLat, rLon);
    } else if (lastBridgeData > 0) {
      Serial.print("STALE — bridge timeout, relay stopped");
    } else {
      Serial.print("Waiting for bridge data");
    }
    Serial.println();
  }
}
