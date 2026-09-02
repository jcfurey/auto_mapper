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

/// A cell is traversable if its cost is at most kMaxAcceptableCost — this
/// includes kFreeSpace (0) and inflated cells (1-252). Using kFreeSpace
/// alone would miss narrow passages where inflation zones from both walls
/// overlap, leaving no cost-0 cells even though the robot can physically
/// fit. kInscribedInflatedObstacle (253) is rejected along with
/// kLethalObstacle (254) and kNoInformation (255): the robot center cannot
/// occupy an inscribed-obstacle cell, so counting it as traversable let the
/// reachability BFS expand through cells the robot can't actually pass and
/// overcounted "free" neighbors when qualifying frontier cells.
inline constexpr bool is_traversable(unsigned char cost)
{
  return cost <= kMaxAcceptableCost;
}

/// Yaw (rotation about +Z, radians in (-pi, pi]) of a unit quaternion.
inline double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/// Choose a frontier goal heading that looks from the refined free-space goal
/// into the unknown frontier. If refinement did not displace the centroid,
/// use the approach bearing; if every point coincides, retain the robot yaw.
inline double frontier_goal_yaw(
  double goal_x, double goal_y,
  double centroid_x, double centroid_y,
  double robot_x, double robot_y,
  double robot_yaw)
{
  constexpr double kDirectionEpsilonSquared = 1e-12;
  double dx = centroid_x - goal_x;
  double dy = centroid_y - goal_y;
  if (dx * dx + dy * dy > kDirectionEpsilonSquared) {
    return std::atan2(dy, dx);
  }

  dx = centroid_x - robot_x;
  dy = centroid_y - robot_y;
  if (dx * dx + dy * dy > kDirectionEpsilonSquared) {
    return std::atan2(dy, dx);
  }

  return robot_yaw;
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
/// Distance is clamped before weighting and applied as a travel penalty. The
/// bounded forward term only rewards frontiers in the robot's forward half
/// plane, so a remote sideways target cannot dominate nearby work. The
/// forward term is suppressed within 0.5 m, where heading alignment is noise.
inline double score_frontier(
  const FrontierScoreParams & params,
  double frontier_length_m, double dx, double dy, double yaw)
{
  const double distance = std::sqrt(dx * dx + dy * dy);
  const double clamped_distance = std::min(distance, params.distance_cap_m);

  double forward_bonus = 0.0;
  if (params.forward_weight > 0.0 && distance > 0.5) {
    // dot in [-1, 1]: +1 = directly ahead, 0 = sideways, -1 = behind.
    // Clamp at zero so only the forward half-plane receives a bonus.
    const double dot =
      (dx * std::cos(yaw) + dy * std::sin(yaw)) / distance;
    forward_bonus = params.forward_weight * std::max(0.0, dot);
  }

  return params.size_weight * frontier_length_m +
         -params.distance_weight * clamped_distance +
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

/// Blacklist a rejected/aborted goal together with the frontier centroid it
/// was refined from.
///
/// Goal refinement can displace the dispatched goal by up to
/// goal_clearance_radius_m from the centroid, which by default exceeds
/// blacklist_radius_m. Blacklisting only the refined goal point then fails
/// to cover the centroid: the same frontier survives the blacklist filter
/// on the next map update, refines to the same rejected goal, and is
/// rejected again — a livelock until the map happens to change. Recording
/// both points closes that gap.
inline void blacklist_rejected_goal(
  std::vector<BlacklistEntry> & entries,
  double goal_x, double goal_y,
  double centroid_x, double centroid_y,
  std::chrono::steady_clock::time_point now)
{
  entries.push_back({goal_x, goal_y, now});
  if (goal_x != centroid_x || goal_y != centroid_y) {
    entries.push_back({centroid_x, centroid_y, now});
  }
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
