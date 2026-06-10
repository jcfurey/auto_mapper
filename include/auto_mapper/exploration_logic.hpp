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

#ifndef AUTO_MAPPER__EXPLORATION_LOGIC_HPP_
#define AUTO_MAPPER__EXPLORATION_LOGIC_HPP_

// Pure exploration logic shared by the auto_mapper node and its unit tests.
// This header is deliberately ROS-free (standard library only) so the logic
// can be compiled and tested without a ROS 2 installation. The node
// translation unit static_asserts that the cost constants below stay in sync
// with nav2_costmap_2d/cost_values.hpp.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <vector>

namespace auto_mapper
{

// Costmap cost values, mirroring nav2_costmap_2d/cost_values.hpp.
inline constexpr unsigned char kFreeSpace = 0;
inline constexpr unsigned char kInscribedInflatedObstacle = 253;
inline constexpr unsigned char kLethalObstacle = 254;
inline constexpr unsigned char kNoInformation = 255;

// Maximum costmap cost accepted for a goal cell. Both goal refinement and
// goal validation check against this same threshold so a borderline cell
// can't pass one and fail the other. Strictly below
// kInscribedInflatedObstacle (253), so cells in [253, 254] are always
// rejected; kNoInformation (255) is also above this threshold, so a
// `cost > kMaxAcceptableCost` comparison rejects unknown cells too.
inline constexpr unsigned char kMaxAcceptableCost = 252;

static_assert(
  kMaxAcceptableCost < kInscribedInflatedObstacle,
  "goal cells must never sit inside the inscribed-obstacle band");

/// Map OccupancyGrid values onto costmap costs.
///
/// OccupancyGrid values are int8 in [-1, 100]. After a
/// static_cast<unsigned char>, valid inputs are [0..100] and 255 (= -1,
/// unknown). Indices 101..254 are unreachable but cheap to fill.
/// [1..98] maps linearly onto [1..252]; 0, 99, 100 and 255 are pinned by
/// the OccupancyGrid -> costmap convention.
inline std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table{};

  for (std::size_t i = 1; i < 256; ++i) {
    cost_translation_table[i] =
      static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  cost_translation_table[0] = kFreeSpace;
  cost_translation_table[99] = kInscribedInflatedObstacle;
  cost_translation_table[100] = kLethalObstacle;
  cost_translation_table[255] = kNoInformation;

  return cost_translation_table;
}

/// A cell is traversable if it is not lethal and not unknown — this includes
/// kFreeSpace (0) and inflated cells (1-252). Using kFreeSpace alone misses
/// narrow passages where inflation zones from both walls overlap, leaving no
/// cost-0 cells even though the robot can physically fit.
inline constexpr bool is_traversable(unsigned char cost)
{
  return cost != kNoInformation && cost != kLethalObstacle;
}

/// Yaw (rotation about +Z, radians in (-pi, pi]) of a unit quaternion.
inline double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/// Weights for frontier scoring; defaults match the node's ROS parameter
/// defaults (frontier_size_weight, frontier_distance_weight,
/// frontier_distance_cap_m, forward_weight).
struct FrontierScoreParams
{
  double size_weight{1.0};
  double distance_weight{0.35};
  double distance_cap_m{20.0};
  double forward_weight{2.0};
};

/// Score a frontier; higher is better.
///
/// \param params scoring weights
/// \param frontier_length_m frontier size in meters
/// \param dx, dy world-frame vector from the robot to the frontier centroid
/// \param yaw robot heading, radians
///
/// Distance acts as a *bonus* (clamped to distance_cap_m), not a penalty,
/// and the forward term rewards frontiers in the robot's heading direction:
/// together they produce depth-first behavior in tunnels — continue forward
/// before turning around to explore side branches. The forward term is
/// suppressed within 0.5 m, where heading alignment is mostly noise.
inline double score_frontier(
  const FrontierScoreParams & params,
  double frontier_length_m, double dx, double dy, double yaw)
{
  const double distance = std::sqrt(dx * dx + dy * dy);
  const double clamped_distance = std::min(distance, params.distance_cap_m);

  double forward_bonus = 0.0;
  if (params.forward_weight > 0.0 && distance > 0.5) {
    // dot in [-1, 1]: +1 = directly ahead, -1 = directly behind;
    // mapped to [0, 1] so behind = 0, ahead = 1, side = 0.5.
    const double dot =
      (dx * std::cos(yaw) + dy * std::sin(yaw)) / distance;
    const double forward_factor = (dot + 1.0) / 2.0;
    forward_bonus = params.forward_weight * forward_factor * clamped_distance;
  }

  return params.size_weight * frontier_length_m +
         params.distance_weight * clamped_distance +
         forward_bonus;
}

/// Rejected goal locations are remembered so the same frontier centroid is
/// not re-selected on the next map update. Entries expire after a
/// configurable duration so that previously inaccessible areas can be
/// retried once the map has changed.
struct BlacklistEntry
{
  double x;
  double y;
  std::chrono::steady_clock::time_point when;
};

/// True if (x, y) lies strictly within radius_m of any blacklist entry.
inline bool is_blacklisted(
  const std::vector<BlacklistEntry> & entries,
  double x, double y, double radius_m)
{
  return std::any_of(
    entries.begin(), entries.end(),
    [x, y, radius_m](const BlacklistEntry & entry) {
      const double dx = x - entry.x;
      const double dy = y - entry.y;
      return dx * dx + dy * dy < radius_m * radius_m;
    });
}

/// Drop entries older than duration_sec (measured against `now`).
inline void prune_blacklist(
  std::vector<BlacklistEntry> & entries,
  std::chrono::steady_clock::time_point now, double duration_sec)
{
  const auto duration =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(duration_sec));
  entries.erase(
    std::remove_if(
      entries.begin(), entries.end(),
      [now, duration](const BlacklistEntry & e) {return (now - e.when) > duration;}),
    entries.end());
}

}  // namespace auto_mapper

#endif  // AUTO_MAPPER__EXPLORATION_LOGIC_HPP_
