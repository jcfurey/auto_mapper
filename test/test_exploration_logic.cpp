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
#include <vector>

#include "auto_mapper/exploration_logic.hpp"

namespace
{

using auto_mapper::BlacklistEntry;
using auto_mapper::FrontierScoreParams;
using std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;

TEST(TranslationTable, PinsSpecialOccupancyValues)
{
  const auto table = auto_mapper::init_translation_table();
  EXPECT_EQ(table[0], auto_mapper::kFreeSpace);
  EXPECT_EQ(table[99], auto_mapper::kInscribedInflatedObstacle);
  EXPECT_EQ(table[100], auto_mapper::kLethalObstacle);
  // OccupancyGrid's -1 (unknown) arrives as 255 after the unsigned cast.
  EXPECT_EQ(table[255], auto_mapper::kNoInformation);
}

TEST(TranslationTable, MapsOccupancyLinearlyOntoCostRange)
{
  const auto table = auto_mapper::init_translation_table();
  EXPECT_EQ(table[1], 1);     // lowest non-zero occupancy -> lowest non-zero cost
  EXPECT_EQ(table[98], 252);  // highest linear occupancy -> kMaxAcceptableCost
  for (int i = 1; i <= 98; ++i) {
    EXPECT_GE(table[i], 1) << "occupancy " << i;
    EXPECT_LE(table[i], 252) << "occupancy " << i;
  }
}

TEST(TranslationTable, IsMonotonicOverValidOccupancy)
{
  const auto table = auto_mapper::init_translation_table();
  for (int i = 1; i <= 100; ++i) {
    EXPECT_LE(table[i - 1], table[i]) << "occupancy " << i;
  }
}

TEST(Traversability, AcceptsFreeAndInflatedCells)
{
  EXPECT_TRUE(auto_mapper::is_traversable(auto_mapper::kFreeSpace));
  EXPECT_TRUE(auto_mapper::is_traversable(1));
  EXPECT_TRUE(auto_mapper::is_traversable(128));
  EXPECT_TRUE(auto_mapper::is_traversable(auto_mapper::kMaxAcceptableCost));
}

TEST(Traversability, RejectsInscribedLethalAndUnknownCells)
{
  // kInscribedInflatedObstacle is a cell the robot center cannot occupy;
  // treating it as traversable let the reachability BFS pass through
  // near-certain obstacles (occupancy 99 maps to 253).
  EXPECT_FALSE(auto_mapper::is_traversable(auto_mapper::kInscribedInflatedObstacle));
  EXPECT_FALSE(auto_mapper::is_traversable(auto_mapper::kLethalObstacle));
  EXPECT_FALSE(auto_mapper::is_traversable(auto_mapper::kNoInformation));
}

TEST(Traversability, MatchesGoalAcceptanceThreshold)
{
  // Traversability and goal validation share kMaxAcceptableCost, so a cell
  // the BFS walks through is always also a legal goal cell and vice versa.
  for (int cost = 0; cost <= 255; ++cost) {
    EXPECT_EQ(
      auto_mapper::is_traversable(static_cast<unsigned char>(cost)),
      cost <= auto_mapper::kMaxAcceptableCost) << "cost " << cost;
  }
}

TEST(YawFromQuaternion, RecoversYawFromUnitQuaternions)
{
  EXPECT_DOUBLE_EQ(auto_mapper::yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0);

  const double s = std::sin(kPi / 4.0);
  const double c = std::cos(kPi / 4.0);
  EXPECT_NEAR(auto_mapper::yaw_from_quaternion(0.0, 0.0, s, c), kPi / 2.0, 1e-12);
  EXPECT_NEAR(auto_mapper::yaw_from_quaternion(0.0, 0.0, -s, c), -kPi / 2.0, 1e-12);
  EXPECT_NEAR(auto_mapper::yaw_from_quaternion(0.0, 0.0, 1.0, 0.0), kPi, 1e-12);
}

TEST(ScoreFrontier, ReducesToSizeTermWhenOtherWeightsAreZero)
{
  FrontierScoreParams params;
  params.size_weight = 2.0;
  params.distance_weight = 0.0;
  params.forward_weight = 0.0;
  const double score = auto_mapper::score_frontier(params, 3.0, 10.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(score, 6.0);
}

TEST(ScoreFrontier, DistanceActsAsBonusUpToCap)
{
  // Documents the intentional sign convention: with the default positive
  // distance_weight, farther frontiers score higher (depth-first behavior),
  // saturating at distance_cap_m.
  FrontierScoreParams params;
  params.forward_weight = 0.0;  // isolate the distance term
  const double near = auto_mapper::score_frontier(params, 1.0, 2.0, 0.0, 0.0);
  const double far = auto_mapper::score_frontier(params, 1.0, 10.0, 0.0, 0.0);
  EXPECT_GT(far, near);

  const double at_cap =
    auto_mapper::score_frontier(params, 1.0, params.distance_cap_m, 0.0, 0.0);
  const double beyond_cap =
    auto_mapper::score_frontier(params, 1.0, params.distance_cap_m + 15.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(beyond_cap, at_cap);
}

TEST(ScoreFrontier, PrefersFrontiersAheadOfTheRobot)
{
  const FrontierScoreParams params;  // node defaults: forward_weight = 2.0
  const double yaw = kPi / 2.0;      // robot facing +Y
  const double ahead = auto_mapper::score_frontier(params, 1.0, 0.0, 5.0, yaw);
  const double side = auto_mapper::score_frontier(params, 1.0, 5.0, 0.0, yaw);
  const double behind = auto_mapper::score_frontier(params, 1.0, 0.0, -5.0, yaw);
  EXPECT_GT(ahead, side);
  EXPECT_GT(side, behind);
}

TEST(ScoreFrontier, SuppressesForwardBonusAtCloseRange)
{
  FrontierScoreParams params;
  params.size_weight = 0.0;
  params.distance_weight = 0.0;
  params.forward_weight = 5.0;
  // Directly ahead but within the 0.5 m gate: no forward bonus, score 0.
  EXPECT_DOUBLE_EQ(auto_mapper::score_frontier(params, 1.0, 0.4, 0.0, 0.0), 0.0);
  // Zero distance must not divide by zero.
  EXPECT_DOUBLE_EQ(auto_mapper::score_frontier(params, 1.0, 0.0, 0.0, 0.0), 0.0);
  // Just beyond the gate the bonus kicks in.
  EXPECT_GT(auto_mapper::score_frontier(params, 1.0, 0.6, 0.0, 0.0), 0.0);
}

TEST(ScoreFrontier, MatchesHandComputedExample)
{
  // size 1.0 * 2.0 m + distance 0.35 * 4.0 m + forward 2.0 * 1.0 * 4.0 m
  // (directly ahead: forward_factor = 1) = 2.0 + 1.4 + 8.0
  const FrontierScoreParams params;
  const double score = auto_mapper::score_frontier(params, 2.0, 4.0, 0.0, 0.0);
  EXPECT_NEAR(score, 11.4, 1e-12);
}

TEST(Blacklist, EmptyListBlocksNothing)
{
  const std::vector<BlacklistEntry> entries;
  EXPECT_FALSE(auto_mapper::is_blacklisted(entries, 0.0, 0.0, 1.0));
}

TEST(Blacklist, BlocksStrictlyWithinRadius)
{
  const auto now = steady_clock::now();
  const std::vector<BlacklistEntry> entries = {{10.0, -5.0, now}};
  EXPECT_TRUE(auto_mapper::is_blacklisted(entries, 10.0, -5.0, 1.0));
  EXPECT_TRUE(auto_mapper::is_blacklisted(entries, 10.5, -5.0, 1.0));
  // The comparison is strict: a point exactly on the radius is allowed.
  EXPECT_FALSE(auto_mapper::is_blacklisted(entries, 11.0, -5.0, 1.0));
  EXPECT_FALSE(auto_mapper::is_blacklisted(entries, 12.0, -5.0, 1.0));
}

TEST(Blacklist, ChecksEveryEntry)
{
  const auto now = steady_clock::now();
  const std::vector<BlacklistEntry> entries = {{0.0, 0.0, now}, {5.0, 5.0, now}};
  EXPECT_TRUE(auto_mapper::is_blacklisted(entries, 5.2, 5.2, 1.0));
  EXPECT_FALSE(auto_mapper::is_blacklisted(entries, 2.5, 2.5, 1.0));
}

TEST(Blacklist, PruneDropsOnlyExpiredEntries)
{
  const auto now = steady_clock::now();
  std::vector<BlacklistEntry> entries = {
    {1.0, 1.0, now - std::chrono::seconds(120)},  // expired
    {2.0, 2.0, now - std::chrono::seconds(30)},   // fresh
    {3.0, 3.0, now},                              // fresh
  };
  auto_mapper::prune_blacklist(entries, now, 60.0);
  ASSERT_EQ(entries.size(), 2u);
  EXPECT_DOUBLE_EQ(entries[0].x, 2.0);
  EXPECT_DOUBLE_EQ(entries[1].x, 3.0);
}

TEST(Blacklist, PruneKeepsEntryExactlyAtExpiry)
{
  // (now - when) > duration is strict, so an entry exactly at the boundary
  // survives this prune and is dropped on the next one.
  const auto now = steady_clock::now();
  std::vector<BlacklistEntry> entries = {{1.0, 1.0, now - std::chrono::seconds(60)}};
  auto_mapper::prune_blacklist(entries, now, 60.0);
  EXPECT_EQ(entries.size(), 1u);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
