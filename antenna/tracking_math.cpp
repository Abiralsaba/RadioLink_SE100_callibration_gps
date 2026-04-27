#include "tracking_math.h"

static double deg2rad(double deg) { return deg * DEG_TO_RAD; }

static double rad2deg(double rad) { return rad * RAD_TO_DEG; }

double compute_bearing(double lat1, double lon1, double lat2, double lon2) {

  double lat1r = deg2rad(lat1);
  double lat2r = deg2rad(lat2);
  double dlon = deg2rad(lon2 - lon1);

  double x = sin(dlon) * cos(lat2r);
  double y = cos(lat1r) * sin(lat2r) - sin(lat1r) * cos(lat2r) * cos(dlon);

  double bearing = atan2(x, y);
  double deg = rad2deg(bearing);

  if (deg < 0)
    deg += 360.0;
  return deg;
}

double compute_distance(double lat1, double lon1, double lat2, double lon2) {

  double lat1r = deg2rad(lat1);
  double lat2r = deg2rad(lat2);

  double dlat = lat2r - lat1r;
  double dlon = deg2rad(lon2 - lon1);

  double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
             cos(lat1r) * cos(lat2r) * sin(dlon / 2.0) * sin(dlon / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return EARTH_RADIUS * c;
}

double normalize_angle(double angle) {
  while (angle > 180.0)
    angle -= 360.0;
  while (angle < -180.0)
    angle += 360.0;
  return angle;
}

double compute_heading_error(double bearing, double heading) {
  return normalize_angle(bearing - heading);
}

double low_pass_filter(double current, double previous, double alpha) {
  return alpha * current + (1.0 - alpha) * previous;
}

/*
double apply_deadband(double value, double threshold) {
    if (value > -threshold && value < threshold)
        return 0.0;
    return value;
}
*/
double apply_deadband(double value, double threshold) {
  if (fabs(value) < threshold)
    return value * 0.3; // small correction instead of zero
  return value;
}

double compute_dynamic_deadband(double distance) {
  double min_db = 1.0; // 2.0
  double max_db = 4.0; // 8.0
  double scale = 0.01; // 0.01

  double db = distance * scale;

  if (db < min_db)
    db = min_db;
  if (db > max_db)
    db = max_db;

  return db;
}
