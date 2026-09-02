# auto_mapper

Frontier-based autonomous exploration node. Subscribes to a 2D
`OccupancyGrid` and a robot pose, finds reachable frontiers between
known-free and unknown space, and dispatches the highest-scoring
frontier as a Nav2 `NavigateToPose` goal.

This is a fork of [Omar-Salem/auto_mapper](https://github.com/Omar-Salem/auto_mapper)
with substantial divergence — see **Differences from upstream** below.

## Topics

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Sub | `<map_topic>` | `nav_msgs/OccupancyGrid` | The map to find frontiers in. Default `/map`. |
| Sub | `<odom_topic>` | `nav_msgs/Odometry` | Robot pose source. Default `/localization/odometry/odom`. |
| Sub | `<pose_topic>` | `geometry_msgs/PoseStamped` | Optional alternative to `odom_topic`. Default empty (disabled). |
| Pub | `/frontiers` | `visualization_msgs/MarkerArray` | Frontier centroids for RViz/Trillium. |
| Action client | `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Goal dispatch. Remap as needed. |
| Service client | `/map_server/save_map` | `nav2_msgs/SaveMap` | Map snapshot on every goal completion + on frontier exhaustion. |

## Services

| Service | Type | Behavior |
| --- | --- | --- |
| `~/set_enabled` | `std_srvs/SetBool` | `true` resumes exploration, `false` cancels the active goal and pauses. The node never auto-shuts-down on frontier exhaustion — toggle via this service. |

## Parameters

| Param | Default | Notes |
| --- | --- | --- |
| `map_topic` | `/map` | OccupancyGrid input. |
| `odom_topic` | `/localization/odometry/odom` | Primary pose source (Odometry). |
| `pose_topic` | `""` | Alternative pose source (PoseStamped). |
| `map_path` | `/tmp/maps` | Where `nav2_map_saver` writes the snapshot. |
| `start_enabled` | `true` | If false, sit idle until `~/set_enabled` is called. |
| `startup_delay_sec` | `0.0` | Wait this many seconds before first goal so localization/mapping can warm up. |
| `min_frontier_length_m` | `0.25` | Reject frontiers shorter than this (filters single-cell holes). |
| `min_distance_to_frontier_m` | `0.75` | Skip frontiers closer than this. |
| `max_distance_to_frontier_m` | `40.0` | Skip frontiers farther than this. |
| `frontier_size_weight` | `1.0` | Score multiplier for frontier length. |
| `frontier_distance_weight` | `0.35` | Travel penalty for clamped distance (nearer frontiers score higher). |
| `frontier_distance_cap_m` | `20.0` | Distance is clamped to this before scoring. |
| `forward_weight` | `2.0` | Bounded bonus for frontiers in the robot's forward half-plane. 0 disables. |
| `min_free_threshold` | `4` | Number of traversable 8-neighbors required for a cell to count as a frontier. |
| `goal_clearance_radius_m` | `1.5` | Search this radius around the centroid for the lowest-cost cell to use as the goal. 0 disables. |
| `blacklist_radius_m` | `1.0` | Re-reject frontiers whose centroid is within this radius of a recently rejected goal. |
| `blacklist_duration_sec` | `60.0` | Blacklist entries expire after this many seconds. |
| `goal_timeout_sec` | `300.0` | Watchdog: cancel a goal whose result hasn't arrived within this time (e.g. Nav2 died mid-goal) and resume exploring. `0` disables. |

## Differences from upstream

- Pose source can be `Odometry` (preferred) or `PoseStamped`; upstream
  was PoseStamped only.
- Bundled `nav2`/`slam_toolbox` launch glue removed — this package
  ships only the node, and is launched from a parent workspace.
- Frontier scoring balances information gain against travel distance, with a
  bounded forward bias instead of unbounded cross-map attraction.
- Goal-clearance refinement: shifts the dispatched goal to the lowest
  cost cell within a radius of the centroid (pulls goals away from
  walls).
- Frontier-facing goal headings replace a fixed map-frame yaw, avoiding
  unnecessary arrival rotations while aiming the sensor into unknown space.
- BFS expands through inflated cells, not only `FREE_SPACE`, so
  corridors narrower than 2× inflation_radius are reachable.
- Seed-cell BFS recovery for cases where the robot's vicinity is
  entirely unknown/lethal (common with VDB + ground-segmentation).
- Per-goal blacklist (radius + expiry) prevents repeatedly retrying
  inaccessible frontiers.
- `~/set_enabled` runtime soft-toggle.
- Re-arms automatically after frontier exhaustion (no `stop()`
  latching).

## Development

The pure exploration logic (cost translation table, traversability,
frontier scoring, goal blacklist) lives in
`include/auto_mapper/exploration_logic.hpp`, which is ROS-free and
unit-tested in `test/test_exploration_logic.cpp`. Build and run the
tests, plus the ament linters (copyright, cpplint, cppcheck, xmllint,
lint_cmake), with:

```bash
colcon build --packages-select auto_mapper
colcon test --packages-select auto_mapper
colcon test-result --verbose
```

The header has no ROS dependencies, so the tests also build standalone:

```bash
g++ -std=c++17 -Wall -Wextra -Iinclude test/test_exploration_logic.cpp \
  -lgtest -o /tmp/test_logic && /tmp/test_logic
```

`ament_uncrustify` is excluded from the lint set — the codebase keeps
its original formatting. Run `ament_uncrustify --reformat src include
test` and drop the exclusion in `CMakeLists.txt` to adopt the ament
style.

## Acknowledgements

- Original [auto_mapper](https://github.com/Omar-Salem/auto_mapper) by Omar Salem.
- Frontier BFS pattern from [m-explore](https://github.com/hrnr/m-explore).
