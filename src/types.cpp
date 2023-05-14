#include <rrt/types.h>
#include <rrt/utils.h>

#include <cmath>
#include <stdexcept>

using namespace utility;

namespace types {

double computeSquaredDistance(const WorldPosition &p1, const WorldPosition &p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

double computeDistance(const WorldPosition &p1, const WorldPosition &p2) {
	return sqrt(computeSquaredDistance(p1, p2));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

bool Pose2D::operator==(const Pose2D& conf) const {
	return approxEqual(position.x, conf.position.x) &&
				 approxEqual(position.y, conf.position.y) &&
				 approxEqual(normalizeAngle(yaw), normalizeAngle(conf.yaw));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

Pose2D steeringFunction(const Pose2D &q_nearest,
												const Pose2D &q_rand,
												double steering_dist,
												double steering_angular_dist) {
	if (steering_dist <= 0.0 || steering_angular_dist <= 0.0)
		throw std::invalid_argument("Can't steer, invalid arguments received!");

	Pose2D q_steer = q_rand;
  double squared_dist = computeSquaredDistance(q_nearest.position, q_rand.position);
  if (squared_dist > steering_dist * steering_dist) {
    double dist = std::sqrt(squared_dist);
    double tmp1 = steering_dist / dist;
    double tmp2 = (1.0 / tmp1) - 1.0;
    q_steer.position.x = tmp1 * (tmp2 * q_nearest.position.x + q_rand.position.x);
    q_steer.position.y = tmp1 * (tmp2 * q_nearest.position.y + q_rand.position.y);
  }
  double angular_dist = utility::shortestAngularDistance(q_nearest.yaw, q_rand.yaw);
  if (fabs(angular_dist) > steering_angular_dist)
    q_steer.yaw = utility::normalizeAngle(q_nearest.yaw + utility::sign(angular_dist) * steering_angular_dist);
  return q_steer;
}

} // namespace types