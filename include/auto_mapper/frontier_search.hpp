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

#ifndef AUTO_MAPPER__FRONTIER_SEARCH_HPP_
#define AUTO_MAPPER__FRONTIER_SEARCH_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "auto_mapper/exploration_logic.hpp"

namespace auto_mapper
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2
{
  Point2 position;
  double yaw{0.0};
};

struct GridGeometry
{
  uint32_t width{0};
  uint32_t height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};

  std::size_t size() const;
  bool operator==(const GridGeometry & other) const;
  std::optional<uint32_t> world_to_cell(Point2 point) const;
  Point2 cell_center(uint32_t cell) const;
};

// Shared immutable occupancy storage, including an aliasing shared_ptr to a ROS
// message's data vector. No costmap resize, reset, or second full-map copy.
struct Grid
{
  GridGeometry geometry;
  std::shared_ptr<const std::vector<int8_t>> data;

  unsigned char cost(uint32_t cell) const;
};

std::string validate_grid(
  const GridGeometry & geometry, const std::vector<int8_t> & data,
  std::size_t max_cells);

struct SearchParams
{
  FrontierScoreParams score;
  double min_length{0.25};
  double min_distance{0.75};
  double max_distance{40.0};
  int min_free_neighbors{2};
  double clearance_radius{1.5};
  double robot_radius{0.0};
  double seed_search_radius{1.0};
  double blacklist_radius{1.0};
};

std::string validate_search_params(const SearchParams & params);

struct Frontier
{
  Point2 centroid;  // Visualization only; never a navigation target.
  Point2 boundary;
  Point2 goal;
  std::size_t size{0};
  double score{0.0};
};

struct SearchResult
{
  bool canceled{false};
  bool valid_seed{false};
  std::size_t frontier_count{0};
  std::vector<Frontier> frontiers;
  std::optional<Frontier> selected;
  uint32_t robot_cell{0};
  // Goal-to-seed path in the searched grid, used to revalidate a result when a
  // newer map arrives while the worker is searching.
  std::vector<uint32_t> path;
};

bool footprint_free(
  const Grid & grid, uint32_t cell, double radius, bool allow_unknown,
  const std::function<bool()> & canceled = [] {return false;});
// Use the same parameters as the search. A canceled validation returns false
// so the scheduler can plan again using its latest snapshot.
bool path_still_valid(
  const SearchResult & result, const Grid & searched, const Grid & latest,
  Pose2 robot, const SearchParams & params,
  const std::function<bool()> & canceled = [] {return false;});

class FrontierSearch
{
public:
  SearchResult search(
    const std::shared_ptr<const Grid> & grid, Pose2 robot, const SearchParams & params,
    const std::vector<BlacklistEntry> & blacklist = {},
    const std::function<bool()> & canceled = [] {return false;});

private:
  bool compute_clearance(
    const std::shared_ptr<const Grid> & grid, const std::function<bool()> & canceled);
  void distance_transform(std::size_t length);

  // Reused by the single search worker, never accessed by ROS callbacks.
  std::vector<uint8_t> flags_;
  std::vector<uint32_t> parents_;
  std::vector<float> clearance_;
  std::vector<float> line_, transformed_;
  std::vector<std::size_t> envelope_;
  std::vector<double> intersections_;
  std::shared_ptr<const Grid> clearance_grid_;
};

}  // namespace auto_mapper

#endif  // AUTO_MAPPER__FRONTIER_SEARCH_HPP_
