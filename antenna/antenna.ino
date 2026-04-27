#include "hardware_interface.h"
#include "tracking_math.h"

double prev_error = 0;
static unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  hardware_init();
}

void loop() {

  double my_lat, my_lon;
  double rover_lat, rover_lon;

  // 1. Read inputs (ALWAYS — keeps serial buffers drained and caches updated)
  get_antenna_coordinates(&my_lat, &my_lon);
  get_rover_coordinates(&rover_lat, &rover_lon);

  double heading = get_antenna_heading();

  // 2. Coordinate validity — do NOT compute or drive servo on (0,0)
  //    GPS returns (0,0) before satellite lock (~30-60s after boot).
  //    Telemetry returns (0,0) before the rover sends its first position.
  //    Computing bearing on (0,0)→(0,0) produces garbage that slams the servo.
  //    Servo holds its current position (90° after init sweep) until both acquire.
  bool antenna_ready = (my_lat != 0.0 || my_lon != 0.0);
  bool rover_ready = (rover_lat != 0.0 || rover_lon != 0.0);

  double bearing = 0.0;
  double error = 0.0;
  double servo_angle = 90.0;

  if (antenna_ready && rover_ready) {

    // 3. Compute navigation
    bearing = compute_bearing(my_lat, my_lon, rover_lat, rover_lon);
    double distance = compute_distance(my_lat, my_lon, rover_lat, rover_lon);

    // 4. Compute error
    error = compute_heading_error(bearing, heading);

    // 5. Stability
    double deadband = compute_dynamic_deadband(distance);
    error = apply_deadband(error, deadband);
    error = low_pass_filter(error, prev_error, 0.2);

    // 6. Servo output
    servo_angle = 90.0 + error;

    if (servo_angle < 0)
      servo_angle = 0;
    if (servo_angle > 180)
      servo_angle = 180;

    set_servo_angle(servo_angle);

    prev_error = error;
  }

  // 7. Serial monitor — print once per second
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    // Own GPS
    Serial.print("Base: ");
    if (antenna_ready) {
      Serial.printf("%.7f,%.7f", my_lat, my_lon);
    } else {
      Serial.print("Waiting for GPS");
    }

    // Received rover telemetry
    Serial.print(" | Rover: ");
    if (rover_ready) {
      Serial.printf("%.7f,%.7f", rover_lat, rover_lon);
    } else {
      Serial.print("Waiting for Telemetry");
    }

    // Compass heading
    Serial.printf(" | H: %.1f", heading);

    // Bearing & error (only when both positions are valid)
    if (antenna_ready && rover_ready) {
      Serial.printf(" | B: %.1f | Err: %.1f | Servo: %.1f", bearing, error,
                    servo_angle);
    }

    Serial.println();
  }

  delay(100); // ~10 Hz
}