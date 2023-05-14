#pragma once

#include <rrt/types.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace grid {

struct Map {
	enum class Cell : uint8_t {
		FREE = 0,
		OCCUPIED = 1
	};
	std::vector<Cell> occupancy; // occupancy state of each map cell. In row-major order, starting with (0,0)
	double resolution; // m / pixel
	size_t width, height; // pixel
	types::WorldPosition origin; // world coordinates of the bottom-left point in the map
};

class Grid {
public:
	Grid(const Map &map);

	// return true if the given position corresponds to a free map cell
	[[nodiscard]] bool isValidPosition(const types::WorldPosition &position) const;

	// return true if the map cells crossed by the given path are free. NOTE: each point of the path is approximated with the center of the corresponding map cell
	[[nodiscard]] bool collisionFree(const types::Path &path) const;

private:
	const Map map_;

	types::GridPosition fromWorldToGrid_(const types::WorldPosition &world_position) const;

	size_t fromGridToMap_(const types::GridPosition &grid_position) const;

	bool isWithinGrid_(const types::WorldPosition &position) const;

	bool isFreeMapCell_(const types::GridPosition &p) const;

	bool straightLineFeasible_(const types::WorldPosition &p1, const types::WorldPosition &p2) const;
};

} // namespace grid