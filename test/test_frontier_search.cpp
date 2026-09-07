// Copyright 2026 jcfurey
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "auto_mapper/frontier_search.hpp"
#include "auto_mapper/search_worker.hpp"

namespace
{
using auto_mapper::FrontierSearch;
using auto_mapper::Grid;
using auto_mapper::GridGeometry;
using auto_mapper::Pose2;
using auto_mapper::SearchParams;

struct Scene
{
  explicit Scene(uint32_t width = 20, uint32_t height = 20, double resolution = 1.0)
  : data(std::make_shared<std::vector<int8_t>>(static_cast<std::size_t>(width) * height, -1)),
    grid(std::make_shared<Grid>(Grid{{width, height, resolution, 0, 0, 0}, data}))
  {}
  void cell(uint32_t x, uint32_t y, int8_t value) {(*data)[y * grid->geometry.width + x] = value;}
  std::shared_ptr<std::vector<int8_t>> data;
  std::shared_ptr<Grid> grid;
};

Scene straight()
{
  Scene scene;
  for (uint32_t y = 0; y < 20; ++y) {
    for (uint32_t x = 0; x < 10; ++x) {
      scene.cell(x, y, 0);
    }
  }
  return scene;
}

TEST(GridValidation, RejectsMalformedDimensionsResolutionAndCells)
{
  const GridGeometry valid{2, 2, 1, 0, 0, 0};
  EXPECT_TRUE(auto_mapper::validate_grid(valid, {0, -1, 99, 100}, 4).empty());
  EXPECT_FALSE(auto_mapper::validate_grid(valid, {0}, 4).empty());
  EXPECT_FALSE(auto_mapper::validate_grid(valid, {0, 0, 0, 0}, 3).empty());
  EXPECT_FALSE(auto_mapper::validate_grid(valid, {0, 0, -2, 0}, 4).empty());
  EXPECT_FALSE(auto_mapper::validate_grid(valid, {0, 0, 101, 0}, 4).empty());
  auto bad = valid;
  bad.width = bad.height = 65536;
  EXPECT_FALSE(auto_mapper::validate_grid(bad, {}, 16000000).empty());
  bad = valid; bad.width = 0;
  EXPECT_FALSE(auto_mapper::validate_grid(bad, {}, 4).empty());
  for (double resolution : {0.0, -1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    bad = valid; bad.resolution = resolution;
    EXPECT_FALSE(auto_mapper::validate_grid(bad, {0, 0, 0, 0}, 4).empty());
  }
  bad = valid; bad.origin_yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(auto_mapper::validate_grid(bad, {0, 0, 0, 0}, 4).empty());
}

TEST(GridGeometry, RotatedCellCentersRoundTripAndOutsideCoordinatesAreRejected)
{
  GridGeometry geometry{10, 10, 1, 10, 20, std::acos(-1.0) / 2};
  const auto center = geometry.cell_center(0);
  EXPECT_NEAR(center.x, 9.5, 1e-12);
  EXPECT_NEAR(center.y, 20.5, 1e-12);
  for (uint32_t cell = 0; cell < 100; ++cell) {
    EXPECT_EQ(geometry.world_to_cell(geometry.cell_center(cell)), cell);
  }
  EXPECT_FALSE(geometry.world_to_cell({11, 20}));
  EXPECT_FALSE(geometry.world_to_cell({std::numeric_limits<double>::quiet_NaN(), 20}));
  EXPECT_FALSE(geometry.world_to_cell({1e100, 1e100}));
}

TEST(Frontiers, DefaultsFindStraightBoundaryAndChooseReachableApproach)
{
  auto scene = straight();
  FrontierSearch search;
  const Pose2 robot{{2.5, 10.5}, 0};
  const auto result = search.search(scene.grid, robot, SearchParams{});
  ASSERT_TRUE(result.selected);
  EXPECT_EQ(result.frontier_count, 1u);
  EXPECT_EQ(result.selected->size, 20u);
  EXPECT_LT(result.selected->goal.x, 10);
  EXPECT_DOUBLE_EQ(result.selected->boundary.x, 10.5);
  EXPECT_TRUE(auto_mapper::path_still_valid(result, *scene.grid, *scene.grid, robot,
      SearchParams{}));
}

TEST(Frontiers, ClosedBoundaryDoesNotUseItsCentroidAsGoalOrDistanceFilter)
{
  Scene scene(30, 30);
  for (uint32_t y = 10; y < 20; ++y) {
    for (uint32_t x = 10; x < 20; ++x) {
      scene.cell(x, y, 0);
    }
  }
  FrontierSearch search;
  const Pose2 robot{{15.5, 15.5}, 0};
  const auto result = search.search(scene.grid, robot, SearchParams{});
  ASSERT_TRUE(result.selected);
  EXPECT_EQ(result.frontier_count, 1u);
  EXPECT_EQ(result.selected->size, 40u);
  EXPECT_NEAR(result.selected->centroid.x, 15, 1e-12);
  EXPECT_GT(std::hypot(result.selected->goal.x - robot.position.x,
    result.selected->goal.y - robot.position.y), 3.0);
  EXPECT_LE(std::hypot(result.selected->goal.x - result.selected->boundary.x,
    result.selected->goal.y - result.selected->boundary.y), std::sqrt(2.0));
}

TEST(Frontiers, RefinementCannotCrossSolidWallToLowerCostDisconnectedRoom)
{
  Scene scene(15, 15, 0.1);
  for (uint32_t y = 0; y < 15; ++y) {
    for (uint32_t x = 0; x < 15; ++x) {
      scene.cell(x, y, x < 5 ? 50 : x == 5 ? -1 : x == 6 ? 100 : 0);
    }
  }
  SearchParams params;
  params.min_distance = 0;
  FrontierSearch search;
  const auto result = search.search(scene.grid, {{0.25, 0.75}, 0}, params);
  ASSERT_TRUE(result.selected);
  EXPECT_LT(result.selected->goal.x, 0.5);
  for (uint32_t cell : result.path) {
    EXPECT_LT(cell % 15, 5u);
                                                              }
}

TEST(Frontiers, ReachabilityDoesNotCutBlockedDiagonalCorners)
{
  Scene scene(5, 5);
  std::fill(scene.data->begin(), scene.data->end(), 100);
  for (uint32_t y = 1; y < 5; ++y) {
    for (uint32_t x = 1; x < 5; ++x) {
      scene.cell(x, y, x == 4 ? -1 : 0);
    }
  }
  scene.cell(0, 0, 0);
  FrontierSearch search;
  const auto result = search.search(scene.grid, {{0.5, 0.5}, 0}, SearchParams{});
  EXPECT_TRUE(result.valid_seed);
  EXPECT_EQ(result.frontier_count, 0u);
  EXPECT_FALSE(result.selected);
}

TEST(Frontiers, SingleCellFrontierIncludesSeedAndHasFiniteCoordinates)
{
  Scene scene(5, 5);
  std::fill(scene.data->begin(), scene.data->end(), 0);
  scene.cell(3, 3, -1);
  FrontierSearch search;
  const auto result = search.search(scene.grid, {{0.5, 0.5}, 0}, SearchParams{});
  ASSERT_TRUE(result.selected);
  EXPECT_EQ(result.selected->size, 1u);
  EXPECT_DOUBLE_EQ(result.selected->centroid.x, 3.5);
  EXPECT_DOUBLE_EQ(result.selected->centroid.y, 3.5);
}

TEST(Frontiers, BlacklistingOneBoundaryPointLeavesOtherPartsAvailable)
{
  auto scene = straight();
  FrontierSearch search;
  const Pose2 robot{{2.5, 10.5}, 0};
  const auto first = search.search(scene.grid, robot, SearchParams{});
  ASSERT_TRUE(first.selected);
  std::vector<auto_mapper::BlacklistEntry> blacklist;
  auto_mapper::blacklist_rejected_goal(blacklist,
    first.selected->goal.x, first.selected->goal.y,
    first.selected->boundary.x, first.selected->boundary.y, 100.0);
  const auto second = search.search(scene.grid, robot, SearchParams{}, blacklist);
  ASSERT_TRUE(second.selected);
  EXPECT_FALSE(auto_mapper::is_blacklisted(blacklist,
    second.selected->goal.x, second.selected->goal.y, 1));
  EXPECT_FALSE(auto_mapper::is_blacklisted(blacklist,
    second.selected->boundary.x, second.selected->boundary.y, 1));
}

TEST(Frontiers, FootprintRejectsNarrowCorridorAndUnknownGoalOverlap)
{
  Scene scene(20, 20, 0.1);
  std::fill(scene.data->begin(), scene.data->end(), 100);
  for (uint32_t y = 1; y < 19; ++y) {
    for (uint32_t x = 9; x <= 10; ++x) {
      scene.cell(x, y, y < 15 ? 0 : -1);
    }
  }
  const uint32_t center = 10 * 20 + 10;
  EXPECT_TRUE(auto_mapper::footprint_free(*scene.grid, center, 0, false));
  EXPECT_FALSE(auto_mapper::footprint_free(*scene.grid, center, 0.2, false));
  auto open = straight();
  EXPECT_FALSE(auto_mapper::footprint_free(*open.grid, 10 * 20 + 9, 0.6, false));
  EXPECT_TRUE(auto_mapper::footprint_free(*open.grid, 10 * 20 + 9, 0.6, true));
}

TEST(Frontiers, RefinementUsesCircularRadiusAndActualObstacleClearance)
{
  auto scene = straight();
  for (uint32_t x = 0; x < 10; ++x) {
    scene.cell(x, 0, 100);
                                                           }
  SearchParams params;
  params.clearance_radius = 1.0;
  params.min_distance = 0;
  FrontierSearch search;
  const auto result = search.search(scene.grid, {{8.5, 1.5}, 0}, params);
  ASSERT_TRUE(result.selected);
  // The nearest approach anchor is (9.5,1.5), next to the bottom wall.
  EXPECT_NEAR(result.selected->goal.x, 9.5, 1e-12);
  EXPECT_NEAR(result.selected->goal.y, 2.5, 1e-12);
}

TEST(Frontiers, NewMapPathObstacleOrObservedBoundaryInvalidatesPendingGoal)
{
  auto scene = straight();
  FrontierSearch search;
  const Pose2 robot{{2.5, 10.5}, 0};
  const auto result = search.search(scene.grid, robot, SearchParams{});
  ASSERT_TRUE(result.selected);
  auto newer = straight();
  ASSERT_GT(result.path.size(), 2u);
  (*newer.data)[result.path[1]] = 100;
  EXPECT_FALSE(auto_mapper::path_still_valid(result, *scene.grid, *newer.grid, robot,
      SearchParams{}));
  (*newer.data)[result.path[1]] = 0;
  EXPECT_TRUE(auto_mapper::path_still_valid(result, *scene.grid, *newer.grid, robot,
      SearchParams{}));
  EXPECT_FALSE(auto_mapper::path_still_valid(result, *scene.grid, *newer.grid,
    robot, SearchParams{}, [] {return true;}));
  (*newer.data)[*newer.grid->geometry.world_to_cell(result.selected->boundary)] = 0;
  EXPECT_FALSE(auto_mapper::path_still_valid(result, *scene.grid, *newer.grid, robot,
      SearchParams{}));
}

TEST(Frontiers, SeedRecoveryIsBoundedAndCannotTunnelThroughWall)
{
  auto scene = straight();
  SearchParams params;
  params.seed_search_radius = 0.4;
  FrontierSearch search;
  EXPECT_FALSE(search.search(scene.grid, {{10.5, 10.5}, 0}, params).valid_seed);
  params.seed_search_radius = 1.1;
  EXPECT_TRUE(search.search(scene.grid, {{10.5, 10.5}, 0}, params).valid_seed);
  auto blocked = straight();
  for (uint32_t y = 0; y < 20; ++y) {
    blocked.cell(10, y, 100);
                                                              }
  params.seed_search_radius = 5.0;
  EXPECT_FALSE(search.search(blocked.grid, {{11.5, 10.5}, 0}, params).valid_seed);
}

TEST(Frontiers, CancellationStopsSearchAndBuffersCanBeReused)
{
  auto scene = straight();
  FrontierSearch search;
  EXPECT_TRUE(search.search(scene.grid, {{2.5, 10.5}, 0}, SearchParams{}, {}, [] {
      return true;
      }).canceled);
  EXPECT_TRUE(search.search(scene.grid, {{2.5, 10.5}, 0}, SearchParams{}).selected);
  EXPECT_TRUE(search.search(scene.grid, {{2.5, 10.5}, 0},
    SearchParams{}, {}, [] {return true;}).canceled);
  Scene small(4, 4);
  std::fill(small.data->begin(), small.data->end(), 0);
  EXPECT_EQ(search.search(small.grid, {{1.5, 1.5}, 0}, SearchParams{}).frontier_count, 0u);
}

TEST(SearchParameters, RejectsInvalidValuesAndDistanceRelationships)
{
  SearchParams params;
  EXPECT_TRUE(auto_mapper::validate_search_params(params).empty());
  params.min_free_neighbors = 0;
  EXPECT_FALSE(auto_mapper::validate_search_params(params).empty());
  params.min_free_neighbors = 9;
  EXPECT_FALSE(auto_mapper::validate_search_params(params).empty());
  params = SearchParams{}; params.min_distance = params.max_distance + 1;
  EXPECT_FALSE(auto_mapper::validate_search_params(params).empty());
  params = SearchParams{}; params.robot_radius = -0.1;
  EXPECT_FALSE(auto_mapper::validate_search_params(params).empty());
  params = SearchParams{}; params.clearance_radius = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(auto_mapper::validate_search_params(params).empty());
}

TEST(SearchWorker, LatestSubmissionWinsAndShutdownJoins)
{
  auto large = std::make_shared<Scene>(2048, 2048);
  std::fill(large->data->begin(), large->data->end(), 0);
  auto small = straight();
  const auto begin = std::chrono::steady_clock::now();
  {
    auto_mapper::SearchWorker worker;
    auto_mapper::SearchJob job;
    job.grid = large->grid;
    job.robot = {{2.5, 10.5}, 0};
    worker.submit(job);
    job.grid = small.grid;
    const auto id = worker.submit(job);
    std::optional<auto_mapper::SearchCompletion> completion;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!completion && std::chrono::steady_clock::now() < deadline) {
      completion = worker.poll();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_TRUE(completion);
    EXPECT_EQ(completion->job.id, id);
    EXPECT_TRUE(completion->result.selected);
    job.grid = large->grid;
    worker.submit(job);
  }
  EXPECT_LT(std::chrono::steady_clock::now() - begin, std::chrono::seconds(4));
}

}  // namespace
