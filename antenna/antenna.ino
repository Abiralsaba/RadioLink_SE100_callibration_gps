#include "hardware_interface.h"
#include "tracking_math.h"

// Tracking threshold — stop motor when heading is within this of target
#define HEADING_THRESHOLD 3.0

// Motor position safety margins (don't allow closer than 2° to limits)
#define POS_LIMIT_MARGIN 2.0

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

  // 2. Update motor position tracking (compass delta accumulation)
  update_motor_position();
  double motor_pos = get_motor_position();

  // 3. Safety: if compass fails, STOP motor immediately
  //    Without compass feedback the motor would spin blindly.
  if (!is_compass_healthy()) {
    stop_motor();
    // Still print diagnostics, but skip all computation
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.printf("[SAFETY] Compass failed — motor stopped | Pos: %.1f\n",
                    motor_pos);
    }
    delay(20);
    return;
  }

  // 4. Coordinate validity — don't track until both positions are valid
  bool antenna_ready = (my_lat != 0.0 || my_lon != 0.0);
  bool rover_ready = (rover_lat != 0.0 || rover_lon != 0.0);

  double bearing = 0.0;
  double error = 0.0;

  if (antenna_ready && rover_ready) {

    // 5. Compute navigation
    bearing = compute_bearing(my_lat, my_lon, rover_lat, rover_lon);

    // 6. Compute heading error [-180, +180]
    //    Positive = rover is clockwise from current heading
    //    Negative = rover is counter-clockwise
    error = compute_heading_error(bearing, heading);

    // 7. Motor control with wire-tangle prevention
    if (fabs(error) > HEADING_THRESHOLD) {

      // Determine natural direction
      bool want_right = (error > 0); // positive error → go CW (right)

      // Wire-tangle check: prevent crossing 0°/360° boundary
      if (want_right && motor_pos >= (360.0 - POS_LIMIT_MARGIN)) {
        // At right limit — MUST go left (long way back)
        want_right = false;
      } else if (!want_right && motor_pos <= POS_LIMIT_MARGIN) {
        // At left limit — MUST go right (long way back)
        want_right = true;
      }

      // Proportional speed based on error magnitude
      int speed;
      double abs_error = fabs(error);
      if (abs_error > 30.0)
        speed = 40; // Fast approach
      else if (abs_error > 10.0)
        speed = 20; // Medium
      else
        speed = 10; // Slow, precise

      // Drive motor
      if (want_right) {
        rotate_motor_right(speed);
      } else {
        rotate_motor_left(speed);
      }

    } else {
      // Within threshold — aligned with rover, stop
      stop_motor();
    }

  } else {
    // No valid coordinates — stop motor, wait for GPS/telemetry
    stop_motor();
  }

  // 8. Serial monitor — print once per second
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.print("Base: ");
    if (antenna_ready) {
      Serial.printf("%.7f,%.7f", my_lat, my_lon);
    } else {
      Serial.print("Waiting for GPS");
    }

    Serial.print(" | Rover: ");
    if (rover_ready) {
      Serial.printf("%.7f,%.7f", rover_lat, rover_lon);
    } else {
      Serial.print("Waiting for Telemetry");
    }

    Serial.printf(" | H: %.1f | Pos: %.1f", heading, motor_pos);

    if (antenna_ready && rover_ready) {
      Serial.printf(" | B: %.1f | Err: %.1f", bearing, error);
    }

    Serial.println();
  }

  delay(20); // ~50 Hz — required for continuous Sabertooth pulse updates
}