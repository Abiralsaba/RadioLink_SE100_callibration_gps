#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

// ===============================
// HARDWARE INIT (call once in setup)
// ===============================
void hardware_init();

// ===============================
// SENSOR FUNCTIONS
// ===============================

// GPS (antenna)
void get_antenna_coordinates(double* lat, double* lon);

// GPS (rover via telemetry)
void get_rover_coordinates(double* lat, double* lon);

// Returns true once when new rover telemetry arrives, then resets
bool rover_coordinate_updated();

// Compass / heading (0–360)
double get_antenna_heading();

// Returns true if the last compass read succeeded
bool is_compass_healthy();

// ===============================
// MOTOR CONTROL (Sabertooth DC motor)
// ===============================

// Rotate motor clockwise (right) at given speed (0–100)
void rotate_motor_right(int speed);

// Rotate motor counter-clockwise (left) at given speed (0–100)
void rotate_motor_left(int speed);

// Stop motor immediately
void stop_motor();

// ===============================
// MOTOR POSITION TRACKING
// ===============================

// Update motor position from compass delta — call every loop iteration
void update_motor_position();

// Get current virtual motor position (0–360°, boots at 180°)
double get_motor_position();

#endif