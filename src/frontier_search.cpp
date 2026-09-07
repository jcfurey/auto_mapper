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

#include "auto_mapper/frontier_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <utility>

namespace auto_mapper
{
namespace
{
constexpr uint8_t kVisited = 1;
constexpr uint8_t kFrontier = 2;
constexpr uint8_t kSeedVisited = 4;
constexpr uint8_t kQueuedUnknown = 8;
constexpr float kInfinity = 1e20F;
constexpr double kHalfDiagonal = 0.7071067811865476;

template<class Function>
void neighbors(const GridGeometry & geometry, uint32_t cell, Function function)
{
  const int x = static_cast<int>(cell % geometry.width);
  const int y = static_cast<int>(cell / geometry.width);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if ((dx == 0 && dy == 0) || x + dx < 0 || y + dy < 0 ||
        x + dx >= static_cast<int>(geometry.width) ||
        y + dy >= static_cast<int>(geometry.height))
      {
        continue;
      }
      function(static_cast<uint32_t>((y + dy) * geometry.width + x + dx), dx, dy);
    }
  }
}

double squared_distance(Point2 a, Point2 b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}
}  // namespace

std::size_t GridGeometry::size() const
{
  return static_cast<std::size_t>(width) * height;
}

bool GridGeometry::operator==(const GridGeometry & other) const
{
  return width == other.width && height == other.height && resolution == other.resolution &&
         origin_x == other.origin_x && origin_y == other.origin_y && origin_yaw == other.origin_yaw;
}

std::optional<uint32_t> GridGeometry::world_to_cell(Point2 point) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(resolution) ||
    resolution <= 0.0 || !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(origin_yaw))
  {
    return std::nullopt;
  }
  const double dx = point.x - origin_x;
  const double dy = point.y - origin_y;
  const double c = std::cos(origin_yaw), s = std::sin(origin_yaw);
  const double x = (c * dx + s * dy) / resolution;
  const double y = (-s * dx + c * dy) / resolution;
  if (!(x >= 0.0 && y >= 0.0 && x < width && y < height)) {
    return std::nullopt;
  }
  return static_cast<uint32_t>(y) * width + static_cast<uint32_t>(x);
}

Point2 GridGeometry::cell_center(uint32_t cell) const
{
  const double x = (cell % width + 0.5) * resolution;
  const double y = (cell / width + 0.5) * resolution;
  const double c = std::cos(origin_yaw), s = std::sin(origin_yaw);
  return {origin_x + c * x - s * y, origin_y + s * x + c * y};
}

unsigned char Grid::cost(uint32_t cell) const
{
  static const auto translation = init_translation_table();
  return translation[static_cast<unsigned char>((*data)[cell])];
}

std::string validate_grid(
  const GridGeometry & geometry, const std::vector<int8_t> & data, std::size_t max_cells)
{
  const uint64_t cells = static_cast<uint64_t>(geometry.width) * geometry.height;
  if (cells == 0 || cells > max_cells || cells >= std::numeric_limits<uint32_t>::max() ||
    geometry.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    geometry.height > static_cast<uint32_t>(std::numeric_limits<int>::max()))
  {
    return "map dimensions are empty or exceed max_map_cells/index limits";
  }
  if (!std::isfinite(geometry.resolution) || geometry.resolution <= 0.0 ||
    !std::isfinite(geometry.origin_x) || !std::isfinite(geometry.origin_y) ||
    !std::isfinite(geometry.origin_yaw))
  {
    return "map resolution/origin must be finite, with positive resolution";
  }
  if (cells != data.size()) {
    return "map data length must equal width * height";
  }
  if (std::any_of(data.begin(), data.end(), [](int8_t value) {return value < -1 || value > 100;})) {
    return "map occupancy must be -1 or in [0, 100]";
  }
  return {};
}

std::string validate_search_params(const SearchParams & p)
{
  const std::array<double, 11> nonnegative = {
    p.min_length, p.min_distance, p.max_distance, p.clearance_radius, p.robot_radius,
    p.seed_search_radius, p.blacklist_radius, p.score.size_weight, p.score.distance_weight,
    p.score.distance_cap_m, p.score.forward_weight};
  for (double value : nonnegative) {
    if (!std::isfinite(value) || value < 0.0 || value > 1e6) {
      return "search distances and weights must be finite and in [0, 1000000]";
    }
  }
  if (p.min_distance > p.max_distance || p.max_distance == 0.0) {
    return "require 0 <= min_distance_to_frontier_m <= max_distance_to_frontier_m, with max > 0";
  }
  if (p.min_free_neighbors < 1 || p.min_free_neighbors > 8) {
    return "min_free_threshold must be in [1, 8]";
  }
  return {};
}

bool footprint_free(
  const Grid & grid, uint32_t cell, double radius, bool allow_unknown,
  const std::function<bool()> & canceled)
{
  if (cell >= grid.geometry.size() || !is_traversable(grid.cost(cell))) {
    return false;
  }
  if (radius <= 0.0) {
    return true;
  }
  const auto & g = grid.geometry;
  const int x = static_cast<int>(cell % g.width), y = static_cast<int>(cell / g.width);
  const double cells = radius / g.resolution;
  if (!allow_unknown && (cells > x + 0.5 || cells > y + 0.5 ||
    cells > g.width - x - 0.5 || cells > g.height - y - 0.5))
  {
    return false;
  }
  const int extent = static_cast<int>(std::min(
      std::ceil(cells + 0.5), static_cast<double>(std::max(g.width, g.height))));
  const int max_y = y + std::min(extent, static_cast<int>(g.height) - 1 - y);
  const int max_x = x + std::min(extent, static_cast<int>(g.width) - 1 - x);
  for (int ny = std::max(0, y - extent); ny <= max_y;
    ++ny)
  {
    if (canceled()) {return false;}
    for (int nx = std::max(0, x - extent); nx <= max_x;
      ++nx)
    {
      const double dx = std::max(0.0, std::abs(nx - x) - 0.5);
      const double dy = std::max(0.0, std::abs(ny - y) - 0.5);
      if (dx * dx + dy * dy > cells * cells) {
        continue;
      }
      const auto cost = grid.cost(static_cast<uint32_t>(ny * g.width + nx));
      if (!is_traversable(cost) && !(allow_unknown && cost == kNoInformation)) {
        return false;
      }
    }
  }
  return true;
}

bool path_still_valid(
  const SearchResult & result, const Grid & searched, const Grid & latest,
  Pose2 robot, const SearchParams & params, const std::function<bool()> & canceled)
{
  if (canceled() || !result.selected || result.path.empty() ||
    !(searched.geometry == latest.geometry))
  {
    return false;
  }
  const auto current_cell = latest.geometry.world_to_cell(robot.position);
  const auto boundary = latest.geometry.world_to_cell(result.selected->boundary);
  if (!current_cell || *current_cell != result.robot_cell || !boundary ||
    latest.cost(*boundary) != kNoInformation)
  {
    return false;
  }
  const double distance = std::sqrt(squared_distance(robot.position, result.selected->goal));
  if (distance < params.min_distance || distance > params.max_distance) {
    return false;
  }
  if (&searched == &latest) {return true;}  // Already checked on this immutable snapshot.
  if (!footprint_free(latest, result.path.front(), params.robot_radius, false, canceled)) {
    return false;
  }
  uint32_t previous = result.path.front();
  for (uint32_t cell : result.path) {
    if (canceled() || !footprint_free(latest, cell, params.robot_radius, true, canceled)) {
      return false;
    }
    const auto width = latest.geometry.width;
    if (previous % width != cell % width && previous / width != cell / width) {
      if (!footprint_free(latest, (previous / width) * width + cell % width,
          params.robot_radius, true, canceled) ||
        !footprint_free(latest, (cell / width) * width + previous % width,
          params.robot_radius, true, canceled))
      {
        return false;
      }
    }
    previous = cell;
  }
  return true;
}

// Exact squared Euclidean distance transform: lower envelope of parabolas in
// each dimension. Finite infinity avoids inf-inf, and double intermediates
// avoid integer overflow on long, thin grids.
void FrontierSearch::distance_transform(std::size_t length)
{
  std::size_t k = 0;
  envelope_[0] = 0;
  intersections_[0] = -std::numeric_limits<double>::infinity();
  intersections_[1] = std::numeric_limits<double>::infinity();
  for (std::size_t q = 1; q < length; ++q) {
    double cross;
    do {
      const double v = static_cast<double>(envelope_[k]);
      const double x = static_cast<double>(q);
      cross = ((static_cast<double>(line_[q]) - line_[envelope_[k]]) + x * x - v * v) /
        (2.0 * (x - v));
      if (cross > intersections_[k] || k == 0) {break;}
      --k;
    } while (true);
    ++k;
    envelope_[k] = q;
    intersections_[k] = cross;
    intersections_[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (std::size_t q = 0; q < length; ++q) {
    while (intersections_[k + 1] < static_cast<double>(q)) {++k;}
    const double delta = static_cast<double>(q) - envelope_[k];
    transformed_[q] = static_cast<float>(delta * delta + line_[envelope_[k]]);
  }
}

bool FrontierSearch::compute_clearance(
  const std::shared_ptr<const Grid> & grid, const std::function<bool()> & canceled)
{
  if (clearance_grid_ == grid) {return true;}
  clearance_grid_.reset();
  const auto & g = grid->geometry;
  clearance_.resize(g.size());
  const std::size_t length = std::max(g.width, g.height);
  line_.resize(length);
  transformed_.resize(length);
  envelope_.resize(length);
  intersections_.resize(length + 1);
  for (uint32_t y = 0; y < g.height; ++y) {
    if (canceled()) {return false;}
    for (uint32_t x = 0; x < g.width; ++x) {
      const auto cost = grid->cost(y * g.width + x);
      line_[x] = cost == kLethalObstacle || cost == kInscribedInflatedObstacle ? 0.0F : kInfinity;
    }
    distance_transform(g.width);
    std::copy_n(transformed_.begin(), g.width, clearance_.begin() + y * g.width);
  }
  for (uint32_t x = 0; x < g.width; ++x) {
    if (canceled()) {return false;}
    for (uint32_t y = 0; y < g.height; ++y) {
      line_[y] = clearance_[y * g.width + x];
    }
    distance_transform(g.height);
    for (uint32_t y = 0; y < g.height; ++y) {
      clearance_[y * g.width + x] = transformed_[y];
    }
  }
  clearance_grid_ = grid;
  return true;
}

SearchResult FrontierSearch::search(
  const std::shared_ptr<const Grid> & grid, Pose2 robot, const SearchParams & p,
  const std::vector<BlacklistEntry> & blacklist, const std::function<bool()> & canceled)
{
  SearchResult result;
  if (canceled()) {result.canceled = true; return result;}
  if (!grid || !grid->data || !validate_search_params(p).empty()) {return result;}
  const auto & g = grid->geometry;
  const auto robot_cell = g.world_to_cell(robot.position);
  if (!robot_cell || grid->data->size() != g.size()) {return result;}
  result.robot_cell = *robot_cell;
  if (!compute_clearance(grid, canceled)) {result.canceled = true; return result;}
  flags_.assign(g.size(), 0);
  parents_.resize(g.size());
  const auto traversable = [&](uint32_t cell) {
      if (!is_traversable(grid->cost(cell))) {return false;}
      if (p.robot_radius == 0.0) {return true;}
      const double clearance = std::sqrt(clearance_[cell]) * g.resolution;
      return clearance >= p.robot_radius + kHalfDiagonal * g.resolution;
    };
  const auto connected = [&](uint32_t from, uint32_t to) {
      if (!traversable(to)) {return false;}
      if (from % g.width != to % g.width && from / g.width != to / g.width) {
        return traversable((from / g.width) * g.width + to % g.width) &&
               traversable((to / g.width) * g.width + from % g.width);
      }
      return true;
    };
  uint32_t seed = *robot_cell;
  std::deque<uint32_t> queue;
  if (!traversable(seed)) {
    // Recover only through a bounded unknown patch under the robot, never
    // through obstacles or diagonally between them.
    queue.push_back(seed);
    flags_[seed] |= kSeedVisited;
    bool found = false;
    while (!queue.empty() && !found) {
      if (canceled()) {result.canceled = true; return result;}
      const uint32_t cell = queue.front(); queue.pop_front();
      neighbors(g, cell, [&](uint32_t next, int dx, int dy) {
          if (found || (dx != 0 && dy != 0) || (flags_[next] & kSeedVisited)) {return;}
          flags_[next] |= kSeedVisited;
          if (squared_distance(robot.position, g.cell_center(next)) >
          p.seed_search_radius * p.seed_search_radius) {return;}
          if (traversable(next)) {
            seed = next; found = true;
          } else if (grid->cost(next) == kNoInformation) {queue.push_back(next);}
      });
    }
    if (!found) {return result;}
    queue.clear();
  }
  result.valid_seed = true;
  std::vector<uint32_t> boundary_seeds;
  queue.push_back(seed);
  flags_[seed] |= kVisited;
  parents_[seed] = seed;
  std::size_t iterations = 0;
  while (!queue.empty()) {
    if ((++iterations % 256 == 0) && canceled()) {result.canceled = true; return result;}
    const uint32_t cell = queue.front(); queue.pop_front();
    neighbors(g, cell, [&](uint32_t next, int, int) {
        if (!(flags_[next] & kVisited) && connected(cell, next)) {
          flags_[next] |= kVisited;
          parents_[next] = cell;
          queue.push_back(next);
        } else if (grid->cost(next) == kNoInformation && !(flags_[next] & kQueuedUnknown)) {
          flags_[next] |= kQueuedUnknown;
          boundary_seeds.push_back(next);
        }
    });
  }
  const auto frontier_cell = [&](uint32_t cell) {
      if (grid->cost(cell) != kNoInformation || (flags_[cell] & kFrontier)) {return false;}
      int count = 0;
      neighbors(g, cell, [&](uint32_t next, int, int) {
          if (flags_[next] & kVisited) {++count;}
      });
      return count >= p.min_free_neighbors;
    };
  const auto blocked = [&](Point2 point) {
      return is_blacklisted(blacklist, point.x, point.y, p.blacklist_radius);
    };
  std::optional<uint32_t> selected_cell;
  for (uint32_t boundary_seed : boundary_seeds) {
    if (canceled()) {result.canceled = true; return result;}
    if (!frontier_cell(boundary_seed)) {continue;}
    Frontier frontier;
    std::optional<uint32_t> anchor;
    double anchor_distance = std::numeric_limits<double>::infinity();
    flags_[boundary_seed] |= kFrontier;
    queue.push_back(boundary_seed);
    while (!queue.empty()) {
      if ((++iterations % 256 == 0) && canceled()) {result.canceled = true; return result;}
      const uint32_t cell = queue.front(); queue.pop_front();
      const Point2 point = g.cell_center(cell);
      frontier.centroid.x += point.x;
      frontier.centroid.y += point.y;
      ++frontier.size;
      const double distance = std::sqrt(squared_distance(robot.position, point));
      neighbors(g, cell, [&](uint32_t next, int, int) {
          if (frontier_cell(next)) {
            flags_[next] |= kFrontier;
            queue.push_back(next);
          }
          if ((flags_[next] & kVisited) && distance >= p.min_distance &&
          distance <= p.max_distance && !blocked(point))
          {
            const auto approach = g.cell_center(next);
            const double d2 = squared_distance(robot.position, approach);
            if (d2 < anchor_distance && !blocked(approach)) {
              anchor = next;
              anchor_distance = d2;
              frontier.boundary = point;
            }
          }
      });
    }
    frontier.centroid.x /= frontier.size;
    frontier.centroid.y /= frontier.size;
    if (frontier.size * g.resolution < p.min_length) {continue;}
    ++result.frontier_count;
    if (!anchor) {continue;}
    const int ax = static_cast<int>(*anchor % g.width), ay = static_cast<int>(*anchor / g.width);
    const double radius = p.clearance_radius / g.resolution;
    const int extent = static_cast<int>(std::min(
        std::ceil(radius), static_cast<double>(std::max(g.width, g.height))));
    std::optional<uint32_t> goal;
    unsigned char best_cost = kNoInformation;
    float best_clearance = -1.0F;
    double best_offset = std::numeric_limits<double>::infinity();
    const int max_y = ay + std::min(extent, static_cast<int>(g.height) - 1 - ay);
    const int max_x = ax + std::min(extent, static_cast<int>(g.width) - 1 - ax);
    for (int y = std::max(0, ay - extent); y <= max_y; ++y) {
      if (canceled()) {result.canceled = true; return result;}
      for (int x = std::max(0, ax - extent); x <= max_x; ++x) {
        const double dx = x - ax, dy = y - ay;
        const double offset = dx * dx + dy * dy;
        if (offset > radius * radius) {continue;}
        const uint32_t cell = static_cast<uint32_t>(y * g.width + x);
        if (!(flags_[cell] & kVisited)) {continue;}
        const Point2 point = g.cell_center(cell);
        const double distance = std::sqrt(squared_distance(robot.position, point));
        if (distance < p.min_distance || distance > p.max_distance || blocked(point) ||
          !footprint_free(*grid, cell, p.robot_radius, false, canceled)) {continue;}
        const auto cost = grid->cost(cell);
        if (cost < best_cost || (cost == best_cost &&
          (clearance_[cell] > best_clearance ||
          (clearance_[cell] == best_clearance && offset < best_offset))))
        {
          goal = cell;
          best_cost = cost;
          best_clearance = clearance_[cell];
          best_offset = offset;
        }
      }
    }
    if (!goal) {continue;}
    frontier.goal = g.cell_center(*goal);
    frontier.score = score_frontier(
      p.score, frontier.size * g.resolution, frontier.goal.x - robot.position.x,
      frontier.goal.y - robot.position.y, robot.yaw);
    result.frontiers.push_back(frontier);
    if (!result.selected || frontier.score > result.selected->score) {
      result.selected = frontier;
      selected_cell = goal;
    }
  }
  if (selected_cell) {
    uint32_t cell = *selected_cell;
    while (true) {
      if ((++iterations % 256 == 0) && canceled()) {result.canceled = true; return result;}
      result.path.push_back(cell);
      if (parents_[cell] == cell) {break;}
      cell = parents_[cell];
    }
  }
  return result;
}

}  // namespace auto_mapper
