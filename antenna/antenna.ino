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
#define MAX_MOTOR_SPEED 12 // Capped low for smooth, controlled movement

// Proportional gain: maps heading error (degrees) → motor speed.
// At 40° error: speed = 0.25 * 40 = 10 → near max.
// At 10° error: speed = 0.25 * 10 = 2.5 → clamped to MIN (5).
#define KP 0.25

// Motor position safety margins (wire-tangle prevention)
#define POS_LIMIT_MARGIN 2.0

// ═══════════════════════════════════════════════════════
//  CONTROL STATE
// ═══════════════════════════════════════════════════════
static unsigned long lastPrint = 0;
static double prev_error = 0.0;
static bool aligned = true; // True = antenna is on target, motor idle

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

    // ── 6. HYSTERESIS DEADBAND ──
    // Prevents the motor from chattering when heading hovers near target.
    // Once aligned, the motor stays stopped until error grows past MOVE_THRESHOLD.
    if (aligned) {
      if (abs_error > MOVE_THRESHOLD) {
        aligned = false; // Error has grown — begin tracking
      }
    } else {
      if (abs_error <= ALIGN_THRESHOLD) {
        aligned = true; // Reached target — lock on
      }
    }

    if (!aligned) {
      // ── 7. PROPORTIONAL SPEED CONTROLLER ──
      // Continuous linear mapping from error to speed.
      // Approach braking: when error is small and decreasing,
      // force minimum speed to prevent overshoot from motor inertia.
      int speed;

      bool approaching =
          (abs_error < fabs(prev_error)); // Error is getting smaller

      if (abs_error < 10.0 && approaching) {
        // Close to target AND closing in — crawl to prevent overshoot
        speed = MIN_MOTOR_SPEED;
      } else {
        // Standard proportional response
        speed = (int)(KP * abs_error);
        speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
      }

      // ── 8. DIRECTION WITH WIRE-TANGLE PREVENTION ──
      bool want_right = (error > 0); // Positive error → rotate CW

      if (want_right && motor_pos >= (360.0 - POS_LIMIT_MARGIN)) {
        want_right = false; // At right limit → take long way left
      } else if (!want_right && motor_pos <= POS_LIMIT_MARGIN) {
        want_right = true; // At left limit → take long way right
      }

      // ── 9. DRIVE MOTOR ──
      if (want_right) {
        rotate_motor_right(speed);
      } else {
        rotate_motor_left(speed);
      }

    } else {
      // Aligned — hold position
      stop_motor();
    }

    prev_error = error;

  } else {
    // No valid coordinates — stop motor, wait for data
    stop_motor();
    aligned = true;
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