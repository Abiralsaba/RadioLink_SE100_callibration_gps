#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

// ===============================
// HARDWARE INIT (call once in setup)
// ===============================
void hardware_init();

// ===============================
// USER HARDWARE FUNCTIONS
// ===============================

// GPS (antenna)
void get_antenna_coordinates(double* lat, double* lon);

// GPS (rover via telemetry)
void get_rover_coordinates(double* lat, double* lon);

// Compass / heading (0–360)
double get_antenna_heading();

// Servo feedback
double get_servo_angle();

// Servo control
void set_servo_angle(double angle);

#endif