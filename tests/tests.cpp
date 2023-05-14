#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

#include <limits>
#include <rrt/dubins.h>
#include <rrt/posq.h>
#include <rrt/cubic_spline.h>
#include <rrt/grid.h>
#include <rrt/utils.h>
#include <rrt/types.h>
#include <rrt/rrt_core.h>
#include <rrt/tree.h>
#include <rrt/random_pose_generator.h>
#include <math.h>
#include <stdexcept>
#include <vector>

std::ostream& operator<<(std::ostream& os, const types::Pose2D& pose) {
  os << "(" << pose.position.x << ", " << pose.position.y << ", " << pose.yaw << ")";
  return os;
}

TEST(UtilsTest, TestShortestAngularDistance) {
  std::vector<double> vec;
  double a = M_PI / 4, b = M_PI / 6;
  EXPECT_DOUBLE_EQ(utility::shortestAngularDistance(a, b), -M_PI / 12);
  EXPECT_DOUBLE_EQ(utility::shortestAngularDistance(b, a), M_PI / 12);

  EXPECT_DOUBLE_EQ(utility::shortestAngularDistance(-a, -b), M_PI / 12);
  EXPECT_DOUBLE_EQ(utility::shortestAngularDistance(-b, -a), -M_PI / 12);


  double c = 0.75 * M_PI, d = -0.9 * M_PI;
  EXPECT_DOUBLE_EQ(utility::shortestAngularDistance(c, d), 0.35 * M_PI);
  EXPECT_DOUBLE_EQ(utility::shortestAngularDistance(d, c), -0.35 * M_PI);

  double e = 0.4 * M_PI;
  EXPECT_DOUBLE_EQ(utility::shortestAngularDistance(e, e), 0.0);
}


class SteeringTest : public testing::Test {
protected:
  types::Pose2D q_nearest_{.position = types::WorldPosition{.x = 1.0, .y = 2.0},
                           .yaw = M_PI_4};
  double steering_dist_ = 2.0;
  double steering_angular_dist_ = M_PI_4;
};

TEST_F(SteeringTest, HandleIdenticalPoses) {
  EXPECT_EQ(q_nearest_, types::steeringFunction(q_nearest_, q_nearest_, steering_dist_, steering_angular_dist_));
}

TEST_F(SteeringTest, HandleInPlaceRotations) {
  auto q_rand = q_nearest_;
  // dummy rotation
  q_rand.yaw += 2 * M_PI;
  EXPECT_EQ(q_nearest_, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));

  // big rotation, steering needed
  q_rand.yaw = q_nearest_.yaw - 1.5 * steering_angular_dist_;
  types::Pose2D q_expected{.position = q_nearest_.position, .yaw = q_nearest_.yaw - steering_angular_dist_};
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));

  // small rotation, steering not needed
  q_rand.yaw = q_nearest_.yaw + 0.5 * steering_angular_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
}

TEST_F(SteeringTest, HandleTranslations) {
  types::Pose2D q_rand;

  // small translations, steering not needed
  // E
  q_rand = q_nearest_;
  q_rand.position.x += 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // NE
  q_rand = q_nearest_;
  q_rand.position.x += 0.5 * steering_dist_;
  q_rand.position.y += 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // N
  q_rand = q_nearest_;
  q_rand.position.y += 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // NO
  q_rand = q_nearest_;
  q_rand.position.x -= 0.5 * steering_dist_;
  q_rand.position.y += 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // O
  q_rand = q_nearest_;
  q_rand.position.x -= 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // SO
  q_rand = q_nearest_;
  q_rand.position.x -= 0.5 * steering_dist_;
  q_rand.position.y -= 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // S
  q_rand = q_nearest_;
  q_rand.position.y -= 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // SE
  q_rand = q_nearest_;
  q_rand.position.x += 0.5 * steering_dist_;
  q_rand.position.y -= 0.5 * steering_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));

  // big translations, steering needed
  types::Pose2D q_expected;
  // double dx, dy, theta;
  // E
  q_rand = q_nearest_;
  q_rand.position.x += 4.0 * steering_dist_;
  q_expected = q_nearest_;
  q_expected.position.x += steering_dist_;
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // N
  q_rand = q_nearest_;
  q_rand.position.y += 2.0 * steering_dist_;
  q_expected = q_nearest_;
  q_expected.position.y += steering_dist_;
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // O
  q_rand = q_nearest_;
  q_rand.position.x -= 8.0 * steering_dist_;
  q_expected = q_nearest_;
  q_expected.position.x -= steering_dist_;
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // S
  q_rand = q_nearest_;
  q_rand.position.y -= 14.0 * steering_dist_;
  q_expected = q_nearest_;
  q_expected.position.y -= steering_dist_;
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  
  auto set_configurations = [&](double dx_factor, double dy_factor) {
    q_rand = q_nearest_;
    double dx = dx_factor * steering_dist_;
    double dy = dy_factor * steering_dist_;
    q_rand.position.x += dx;
    q_rand.position.y += dy;
    q_expected = q_nearest_;
    double theta = atan2(dy, dx);
    q_expected.position.x += steering_dist_ * cos(theta);
    q_expected.position.y += steering_dist_ * sin(theta);
  };
  // NE
  set_configurations(4.0, 3.0);
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // NO
  set_configurations(2.0, -7.0);
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // SO
  set_configurations(-14.0, -0.7);
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
  // SE
  set_configurations(-8.0, 13.0);
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));  
}

TEST_F(SteeringTest, HandleRototranslations) {
  types::Pose2D q_rand;

  // steering not needed
  q_rand = q_nearest_;
  q_rand.position.x += 0.3 * steering_dist_;
  q_rand.position.y -= 0.4 * steering_dist_;
  q_rand.yaw += 0.7 *steering_angular_dist_;
  EXPECT_EQ(q_rand, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));

  // steering needed
  q_rand = q_nearest_;
  double dx = -1.3 * steering_dist_;
  double dy = 0.4 * steering_dist_;
  q_rand.position.x += dx;
  q_rand.position.y += dy;
  q_rand.yaw -= 1.7 *steering_angular_dist_;
  auto q_expected = q_nearest_;
  double theta = atan2(dy, dx);
  q_expected.position.x += steering_dist_ * cos(theta);
  q_expected.position.y += steering_dist_ * sin(theta);
  q_expected.yaw -= steering_angular_dist_;
  EXPECT_EQ(q_expected, types::steeringFunction(q_nearest_, q_rand, steering_dist_, steering_angular_dist_));
}

TEST_F(SteeringTest, HandleInvalidArguments) {
  auto q_rand = q_nearest_;
  EXPECT_ANY_THROW(types::steeringFunction(q_nearest_, q_rand, 0.0, steering_angular_dist_));
  EXPECT_ANY_THROW(types::steeringFunction(q_nearest_, q_rand, -3.0, steering_angular_dist_));
  EXPECT_ANY_THROW(types::steeringFunction(q_nearest_, q_rand, steering_dist_, 0.0));
  EXPECT_ANY_THROW(types::steeringFunction(q_nearest_, q_rand, steering_dist_, -M_PI_2));
}

TEST(RandomPoseGeneratorTest, TestInvalidBoundaries) {
  EXPECT_ANY_THROW(RandomPoseGenerator(0.0, 0.2, 3.0, 0.1));
  EXPECT_ANY_THROW(RandomPoseGenerator(0.9, 0.2, 3.0, 0.1));
  EXPECT_ANY_THROW(RandomPoseGenerator(0.9, 0.2, 0.0, 0.1));
  EXPECT_NO_THROW(RandomPoseGenerator(0.9, 1.2, 3.0, 5.1));
}

TEST(TreeTest, TreeInitializationAndExpansion) {
  tree::Tree tree;
  EXPECT_EQ(tree.getTreeSize(), 0);
  EXPECT_ANY_THROW(auto invalid_node = tree.getNodeById({}));
  
  types::Pose2D init_pose{};
  auto init_node_id = tree.initializeTree(init_pose);
  EXPECT_EQ(tree.getTreeSize(), 1);
  
  auto init_node = tree.getNodeById(init_node_id);
  EXPECT_EQ(init_node.id, init_node_id);
  EXPECT_EQ(init_node.configuration, init_pose);
  EXPECT_EQ(init_node.parent, init_node.id);
  EXPECT_EQ(init_node.cost, 0.0);

  EXPECT_ANY_THROW(tree.insertNode({}, init_node_id + 1, 0.0, {}));

  types::Pose2D new_pose{.position{.x = 2.0, .y = 0.0}, .yaw = 0.0};
  double new_node_cost = types::computeDistance(init_node.configuration.position, new_pose.position);
  auto new_node_id = tree.insertNode(new_pose, init_node_id, new_node_cost, {init_node.configuration, new_pose});
  EXPECT_EQ(tree.getTreeSize(), 2);

  auto new_node = tree.getNodeById(new_node_id);
  EXPECT_EQ(new_node.id, new_node_id);
  EXPECT_EQ(new_node.configuration, new_pose);
  EXPECT_EQ(new_node.parent, init_node.id);
  EXPECT_EQ(new_node.cost, new_node_cost);
}

TEST(TreeTest, TreeInspection) {
  tree::Tree tree;
  EXPECT_ANY_THROW(auto invalid_request = tree.findNearestNode({}));
  EXPECT_TRUE(tree.findNodesWithinRadius({}, std::numeric_limits<double>::max()).empty());

  types::Pose2D init_pose{};
  auto init_node_id = tree.initializeTree(init_pose);

  types::Pose2D pose1{.position{.x = 1.0, .y = 0.0}, .yaw = 0.0};
  double cost1 = types::computeDistance(init_pose.position, pose1.position);
  auto id1 = tree.insertNode(pose1, init_node_id, cost1, {init_pose, pose1});

  types::Pose2D pose2{.position{.x = 0.0, .y = 1.0}, .yaw = 0.0};
  double cost2 = types::computeDistance(init_pose.position, pose2.position);
  auto id2 = tree.insertNode(pose2, init_node_id, cost2, {init_pose, pose2});

  EXPECT_EQ(tree.findNearestNode(pose1), id1);
  EXPECT_EQ(tree.findNearestNode({.position{.x = -3.0, .y = -4.0}, .yaw = 0.0}), init_node_id);

  types::Pose2D tmp{.position{.x = 1.1, .y = 0.0}, .yaw = M_PI_2};
  auto neighbors = tree.findNodesWithinRadius(tmp, 0.2);
  ASSERT_EQ(neighbors.size(), 1);
  EXPECT_EQ(neighbors[0], id1);

  neighbors = tree.findNodesWithinRadius(tmp, 1.109);
  std::array<node_id, 2> expected_neighbors{init_node_id, id1};
  EXPECT_EQ(neighbors.size(), expected_neighbors.size());
  EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), expected_neighbors[0]) != neighbors.end());
  EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), expected_neighbors[1]) != neighbors.end());
}


class GridTest : public testing::Test {
protected:
  size_t h_ = 2, w_ = 3;
  double res_ = 1.0;
  types::WorldPosition origin_{.x = 0.0, .y = 0.0};
  std::vector<grid::Map::Cell> occ_{grid::Map::Cell::FREE, grid::Map::Cell::OCCUPIED, grid::Map::Cell::FREE, grid::Map::Cell::FREE, grid::Map::Cell::FREE, grid::Map::Cell::FREE};
  grid::Grid grid_{grid::Map{.occupancy = occ_, .resolution = res_, .width = w_, .height = h_, .origin = origin_}};
};

TEST_F(GridTest, GridCreationThrowsOnlyIfSizesDoNotAgree) {
  size_t height = 3, width = 4;
  std::vector<grid::Map::Cell> occupancy(height * width + 1, grid::Map::Cell::FREE);
  EXPECT_ANY_THROW(grid::Grid(grid::Map{.occupancy = occupancy, .resolution = 0.05, .width = width, .height = height, .origin = {}}));

  occupancy.pop_back();
  EXPECT_NO_THROW(grid::Grid(grid::Map{.occupancy = occupancy, .resolution = 0.05, .width = width, .height = height, .origin = {}}));
}

TEST_F(GridTest, HandleOutOfMapPositions) {
  EXPECT_FALSE(grid_.isValidPosition({.x = origin_.x - 0.1, .y = origin_.y}));
  EXPECT_FALSE(grid_.isValidPosition({.x = std::numeric_limits<double>::max(), .y = 0.0}));
}

TEST_F(GridTest, OccupancyStateCheck) {
  EXPECT_TRUE(grid_.isValidPosition({.x = origin_.x + 0.3, .y = origin_.y + 1.2}));
  EXPECT_FALSE(grid_.isValidPosition({.x = origin_.x + 1.3, .y = origin_.y + 0.8}));
}

TEST_F(GridTest, CheckCollisionOverPaths) {
  types::WorldPosition p1{.x = 0.5, .y = 0.5}, p2{.x = 1.5, .y = 1.5}, p3{.x = 2.5, .y = 0.5};
  double yaw1_2 = atan2(p2.y - p1.y, p2.x - p1.x);
  double yaw1_3 = atan2(p3.y - p1.y, p3.x - p1.x);
  EXPECT_TRUE(grid_.collisionFree({{.position = p1, .yaw = yaw1_2}, {.position = p2, .yaw = yaw1_2}}));
  EXPECT_FALSE(grid_.collisionFree({{.position = p1, .yaw = yaw1_3}, {.position = p3, .yaw = yaw1_3}}));
}

TEST_F(GridTest, CollisionCheckOnInvalidPathThrows) {
  types::Pose2D pose{};

  EXPECT_ANY_THROW(grid_.collisionFree({pose}));
}

TEST(DubinsTest, StartAndGoalOverlapped) {
  types::Pose2D pose{.position{.x = 1.0, .y = 2.0}, .yaw = M_PI_2};
  dubins::DubinsParams params{.turning_radius = 1.0};
  auto dubins_path = dubins::computeShortestDubinsPath(pose, pose, params);
  EXPECT_DOUBLE_EQ(dubins_path.second, 0.0);
  ASSERT_EQ(dubins_path.first.size(), 2);
  EXPECT_EQ(dubins_path.first.front(), pose);
  EXPECT_EQ(dubins_path.first.back(), pose);
}

TEST(DubinsTest, StraightLine) {
  types::Pose2D start{.position{.x = 1.0, .y = 2.0}, .yaw = M_PI_2};
  types::Pose2D goal{.position{.x = 1.0, .y = 4.0}, .yaw = M_PI_2};
  dubins::DubinsParams params{.turning_radius = 1.0};
  auto dubins_path = dubins::computeShortestDubinsPath(start, goal, params);

  EXPECT_DOUBLE_EQ(dubins_path.second, 2.0);
  ASSERT_GT(dubins_path.first.size(), 1);
  EXPECT_EQ(dubins_path.first.front(), start);
  EXPECT_EQ(dubins_path.first.back(), goal);
  for (auto curr = dubins_path.first.cbegin() + 1, prev = dubins_path.first.cbegin(); curr != dubins_path.first.cend(); prev = curr++) {
    EXPECT_DOUBLE_EQ(curr->position.x, 1.0);
    EXPECT_DOUBLE_EQ(curr->yaw, M_PI_2);
    EXPECT_GE(curr->position.y, 2.0);
    EXPECT_LE(curr->position.y, 4.0);
    EXPECT_GT(curr->position.y, prev->position.y);
  }
}

TEST(POSQTest, StartAndGoalOverlapped) {
  types::Pose2D pose{.position{.x = 1.0, .y = 2.0}, .yaw = M_PI_2};
  posq::POSQParams params{.K_rho = 0.6, .K_phi = -2.0, .K_alpha = 5.0, .K_v = 2.0,};
  auto posq_path = posq::computePOSQPath(pose, pose, params);
  EXPECT_DOUBLE_EQ(posq_path.second, 0.0);
  ASSERT_EQ(posq_path.first.size(), 2);
  EXPECT_EQ(posq_path.first.front(), pose);
  EXPECT_EQ(posq_path.first.back(), pose);
}

TEST(SplineTest, StartAndGoalOverlapped) {
  types::Pose2D pose{.position{.x = 1.0, .y = 2.0}, .yaw = M_PI_2};
  spline::SplineParams params{.tf = 4.0, .v = 1.6};
  auto spline_path = spline::computeSplinePath(pose, pose, params);
  EXPECT_DOUBLE_EQ(spline_path.second, 0.0);
  ASSERT_EQ(spline_path.first.size(), 2);
  EXPECT_EQ(spline_path.first.front(), pose);
  EXPECT_EQ(spline_path.first.back(), pose);
}

class RRTTest : public testing::Test {
protected:
  void SetUp() override {
    std::vector<grid::Map::Cell> occ_free(h_ * w_, grid::Map::Cell::FREE);
    map_free_ = grid::Map{.occupancy = occ_free, .resolution = res_, .width = w_, .height = h_, .origin = origin_};

    auto occ_splitted = occ_free;
    for (size_t i = w_ * h_ / 3; i < 2 * w_ * h_ / 3; ++i)
      occ_splitted.at(i) = grid::Map::Cell::OCCUPIED;
    map_splitted_ = grid::Map{.occupancy = occ_splitted, .resolution = res_, .width = w_, .height = h_, .origin = origin_};
  }
  size_t h_ = 500, w_ = 500;
  double res_ = 0.1;
  types::WorldPosition origin_{.x = 0.0, .y = 0.0};
  grid::Map map_free_, map_splitted_;
  types::Pose2D p_out_{.position{.x = -3.0, .y = 5.0}, .yaw = M_PI_2};
  types::Pose2D p_low_{.position{.x = 13.0, .y = 5.0}, .yaw = M_PI_2};
  types::Pose2D p_high_{.position{.x = 33.0, .y = 45.0}, .yaw = -M_PI_2};

  core::SearchParams search_params_{.min_iterations = 2,
                                    .max_iterations = 200,
                                    .steering_distance = 3.0,
                                    .steering_angular_distance = utility::deg2rad(90.0),
                                    .radius_constant = 18.0,
                                    .ancestors_depth = 0};

  dubins::DubinsParams dubins_params_{.turning_radius = 0.5};
  posq::POSQParams posq_params_{.K_rho = 0.6,
                                .K_phi = -2.0,
                                .K_alpha = 5.0,
                                .K_v = 2.0};
  spline::SplineParams spline_params_{.tf = 4.0,
		                                  .v = 1.6};
};

TEST_F(RRTTest, UninitializedMapThrows) {
  core::RRTCore rrt;
  EXPECT_ANY_THROW(rrt.searchPath({}));
}

TEST_F(RRTTest, SetExtendFunction) {
  core::RRTCore rrt;
  EXPECT_TRUE(rrt.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::DUBINS));
  EXPECT_TRUE(rrt.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::POSQ));
  EXPECT_TRUE(rrt.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::SPLINE));
}

TEST_F(RRTTest, SetSearchParams) {
  core::RRTCore rrt;
  EXPECT_TRUE(rrt.setSearchParams(search_params_));
  
  search_params_.steering_distance = 0.0;
  EXPECT_FALSE(rrt.setSearchParams(search_params_));
}

TEST_F(RRTTest, SetDubinsParams) {
  core::RRTCore rrt;
  EXPECT_TRUE(rrt.setDubinsParams(dubins_params_));
  
  dubins_params_.turning_radius = 0.0;
  EXPECT_FALSE(rrt.setDubinsParams(dubins_params_));
}

TEST_F(RRTTest, SetPOSQParams) {
  core::RRTCore rrt;
  EXPECT_TRUE(rrt.setPOSQParams(posq_params_));
  
  posq_params_.K_rho = 0.0;
  EXPECT_FALSE(rrt.setPOSQParams(posq_params_));
}

TEST_F(RRTTest, SetSplineParams) {
  core::RRTCore rrt;
  EXPECT_TRUE(rrt.setSplineParams(spline_params_));
  
  spline_params_.tf = 0.0;
  EXPECT_FALSE(rrt.setSplineParams(spline_params_));
}

TEST_F(RRTTest, HandleOutOfMapStart) {
  core::RRTCore rrt;
  rrt.setMap(map_free_);

  auto output = rrt.searchPath({.start = p_out_, .goal = p_low_});
  EXPECT_TRUE(output.path.empty());
  EXPECT_EQ(output.path_cost, std::numeric_limits<double>::infinity());
  EXPECT_EQ(output.iterations, 0);
}

TEST_F(RRTTest, DubinsSearchFindsPath) {
  core::RRTCore rrt;
  rrt.setMap(map_free_);
  rrt.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::DUBINS);
  rrt.setSearchParams(search_params_);

  auto start = p_low_;
  auto goal = start;
  goal.position.x += search_params_.steering_distance / 2;
  
  auto output = rrt.searchPath({.start = start, .goal = goal});
  EXPECT_LE(output.path_cost, std::numeric_limits<double>::infinity());
  ASSERT_FALSE(output.path.empty());
  EXPECT_EQ(output.path.front(), start);
  EXPECT_EQ(output.path.back(), goal);
}

TEST_F(RRTTest, POSQSearchDoesNotFindPath) {
  core::RRTCore rrt;
  rrt.setMap(map_splitted_);
  rrt.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::POSQ);
  rrt.setSearchParams(search_params_);

  core::RRTInput input{.start = p_low_, .goal = p_high_};
  
  auto output = rrt.searchPath(input);
  EXPECT_EQ(output.path_cost, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(output.path.empty());
  EXPECT_EQ(output.iterations, search_params_.max_iterations);
}

TEST_F(RRTTest, SplineSearchDoesNotFindPath) {
  core::RRTCore rrt;
  rrt.setMap(map_splitted_);
  rrt.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::SPLINE);
  rrt.setSearchParams(search_params_);

  core::RRTInput input{.start = p_low_, .goal = p_high_};
  
  auto output = rrt.searchPath(input);
  EXPECT_EQ(output.path_cost, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(output.path.empty());
  EXPECT_EQ(output.iterations, search_params_.max_iterations);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}