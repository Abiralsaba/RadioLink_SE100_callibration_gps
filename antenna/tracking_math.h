#ifndef TRACKING_MATH_H
#define TRACKING_MATH_H

#include <math.h>

#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232
#define EARTH_RADIUS 6371000.0

double compute_bearing(double lat1, double lon1,
                       double lat2, double lon2);

double compute_distance(double lat1, double lon1,
                        double lat2, double lon2);

double normalize_angle(double angle);

double compute_heading_error(double bearing, double heading);

double low_pass_filter(double current,
                       double previous,
                       double alpha);

double apply_deadband(double value, double threshold);

double compute_dynamic_deadband(double distance);

#endif