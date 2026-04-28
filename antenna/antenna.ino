#include "hardware_interface.h"
#include "tracking_math.h"

// ═══════════════════════════════════════════════════════
//  CONTROL PARAMETERS
//  Tuned for slow, smooth, pinpoint tracking.
// ═══════════════════════════════════════════════════════

// Hysteresis deadband — prevents motor chattering near target.
// Motor stops when error drops below ALIGN_THRESHOLD.
// Motor restarts only when error exceeds MOVE_THRESHOLD.
#define ALIGN_THRESHOLD 2.0 // degrees — precision stop
#define MOVE_THRESHOLD 5.0  // degrees — restart margin

// Motor speed limits (0–100 scale → Sabertooth pulse width)
#define MIN_MOTOR_SPEED 5  // Minimum that overcomes motor stiction
#define MAX_MOTOR_SPEED 8  // Capped very low for slow, smooth tracking

// Proportional gain: maps heading error (degrees) → motor speed.
// At 40° error: speed = 0.15 * 40 = 6 → moderate.
// At 10° error: speed = 0.15 * 10 = 1.5 → clamped to MIN (5).
#define KP 0.15

// Motor position safety margins (wire-tangle prevention)
// Must be wider than compass noise (~2-3°) to prevent direction-flip
// oscillation.
#define POS_LIMIT_MARGIN 5.0

// Speed ramp rate — max speed change per loop iteration (~50Hz).
// Prevents abrupt motor speed transitions for smooth operation.
#define SPEED_RAMP_STEP 1

// ═══════════════════════════════════════════════════════
//  CONTROL STATE
// ═══════════════════════════════════════════════════════
static unsigned long lastPrint = 0;
static double prev_error = 0.0;
static bool aligned = true; // True = antenna is on target, motor idle
static int current_speed = 0; // Ramped motor speed (smoothed)

void setup() {
  Serial.begin(115200);
  hardware_init();
}

void loop() {

  double my_lat, my_lon;
  double rover_lat, rover_lon;

  // ── 1. READ ALL INPUTS (non-blocking, drains serial buffers) ──
  get_antenna_coordinates(&my_lat, &my_lon);
  get_rover_coordinates(&rover_lat, &rover_lon);

  double heading = get_antenna_heading(); // Filtered (EMA) heading

  // ── 2. UPDATE MOTOR POSITION (compass delta accumulation) ──
  update_motor_position();
  double motor_pos = get_motor_position();

  // ── 3. SAFETY: compass failure → immediate stop ──
  if (!is_compass_healthy()) {
    stop_motor();
    aligned = true;
    current_speed = 0;
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.printf("[SAFETY] Compass failed — motor stopped | Pos: %.1f\n",
                    motor_pos);
    }
    delay(20);
    return;
  }

  // ── 4. COORDINATE VALIDITY ──
  bool antenna_ready = (my_lat != 0.0 || my_lon != 0.0);
  bool rover_ready = (rover_lat != 0.0 || rover_lon != 0.0);

  double bearing = 0.0;
  double error = 0.0;

  if (antenna_ready && rover_ready) {

    // ── 5. NAVIGATION ──
    bearing = compute_bearing(my_lat, my_lon, rover_lat, rover_lon);

    // Heading error: positive = rover is CW from current heading
    error = compute_heading_error(bearing, heading);

    double abs_error = fabs(error);

    // ── 5b. MINIMUM DISTANCE GUARD ──
    // If rover is very close, GPS noise dominates bearing → stop tracking.
    double distance =
        compute_distance(my_lat, my_lon, rover_lat, rover_lon);
    if (distance < 30.0) {
      stop_motor();
      aligned = true;
      current_speed = 0;
      prev_error = error;
      if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        Serial.printf("Base: %.7f,%.7f | Rover: %.7f,%.7f | Dist: %.0fm | "
                      "TOO CLOSE — holding\n",
                      my_lat, my_lon, rover_lat, rover_lon, distance);
      }
      delay(20);
      return;
    }

    // ── 6. HYSTERESIS DEADBAND ──
    if (aligned) {
      if (abs_error > MOVE_THRESHOLD) {
        aligned = false;
      }
    } else {
      if (abs_error <= ALIGN_THRESHOLD) {
        aligned = true;
      }
    }

    if (!aligned) {
      // ── 7. PROPORTIONAL SPEED CONTROLLER ──
      int target_speed;

      bool approaching = (abs_error < fabs(prev_error));

      if (abs_error < 10.0 && approaching) {
        target_speed = MIN_MOTOR_SPEED; // Crawl near target
      } else {
        target_speed = (int)(KP * abs_error);
        target_speed = constrain(target_speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
      }

      // ── 7b. SPEED RAMP LIMITER ──
      // Smoothly ramp current_speed toward target_speed.
      // Max change = SPEED_RAMP_STEP per loop (~50Hz).
      if (current_speed < target_speed) {
        current_speed = min(current_speed + SPEED_RAMP_STEP, target_speed);
      } else if (current_speed > target_speed) {
        current_speed = max(current_speed - SPEED_RAMP_STEP, target_speed);
      }
      current_speed = constrain(current_speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

      // ── 8. DIRECTION WITH WIRE-TANGLE PREVENTION ──
      bool want_right = (error > 0); // Positive error → rotate CW

      if (want_right && motor_pos >= (360.0 - POS_LIMIT_MARGIN)) {
        want_right = false; // At right limit → take long way left
      } else if (!want_right && motor_pos <= POS_LIMIT_MARGIN) {
        want_right = true; // At left limit → take long way right
      }

      // ── 9. DRIVE MOTOR ──
      if (want_right) {
        rotate_motor_right(current_speed);
      } else {
        rotate_motor_left(current_speed);
      }

    } else {
      // Aligned — hold position
      stop_motor();
      current_speed = 0;
    }

    prev_error = error;

  } else {
    // No valid coordinates — stop motor, wait for data
    stop_motor();
    aligned = true;
    current_speed = 0;
    prev_error = 0.0;
  }

  // ── 10. SERIAL DIAGNOSTICS (1 Hz) ──
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.print("Base: ");
    if (antenna_ready) {
      Serial.printf("%.7f,%.7f", my_lat, my_lon);
    } else {
      Serial.print("Waiting GPS");
    }

    Serial.print(" | Rover: ");
    if (rover_ready) {
      Serial.printf("%.7f,%.7f", rover_lat, rover_lon);
    } else {
      Serial.print("Waiting Telem");
    }

    Serial.printf(" | H: %.1f | Pos: %.1f", heading, motor_pos);

    if (antenna_ready && rover_ready) {
      Serial.printf(" | B: %.1f | Err: %.1f | %s", bearing, error,
                    aligned ? "ALIGNED" : "TRACKING");
    }

    Serial.println();
  }

  delay(20); // ~50 Hz — continuous Sabertooth pulse cadence
}