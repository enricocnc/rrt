#pragma once

namespace utility {

// compute the signed shortest angular distance between the given angles
double shortestAngularDistance(const double &current_angle, const double &target_angle);

// convert the given angle from radians to degrees
double rad2deg(const double &angle);

// convert the given angle from degrees to radians
double deg2rad(const double &angle);

// return 1 if input is greater than 0, 0 if input is equal to 0, -1 if input is smaller than 0
int sign(const double &val);

// normalize the given angle between -pi and pi
double normalizeAngle(double angle);

// return true if the difference between a and b is smaller than tol
bool approxEqual(const double &a, const double &b, const double &tol = 1e-6);

} // namespace utility