#include <rrt/types.h>
#include <rrt/utils.h>

#include <cmath>

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

} // namespace types