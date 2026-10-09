# auto_mapper

Autonomous frontier exploration for ROS 2 and Nav2. The node takes a 2D
`OccupancyGrid` and a robot pose, finds reachable boundaries between known and
unknown space, and sends a `NavigateToPose` goal to a free approach cell beside
an actual boundary point. Goal headings face the boundary; an arithmetic
frontier centroid is never used as a navigation target.

This fork derives from [Omar-Salem/auto_mapper](https://github.com/Omar-Salem/auto_mapper).
It ships the node and an RViz configuration. Bring up localization, mapping,
Nav2, and the Nav2 map saver separately.

## Interfaces

Default names are relative to the node namespace and support normal ROS
remapping. Explicit absolute parameter values are also supported.

| Direction | Default name | ROS interface |
| --- | --- | --- |
| Subscribe | `map` | `nav_msgs/msg/OccupancyGrid` |
| Subscribe | `localization/odometry/odom` | `nav_msgs/msg/Odometry` |
| Subscribe, optional | Set `pose_topic` to enable | `geometry_msgs/msg/PoseStamped` |
| Publish | `frontiers` | `visualization_msgs/msg/MarkerArray`, approach goals |
| Action client | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| Service client | `map_saver/save_map` | `nav2_msgs/srv/SaveMap` |
| Service server | `~/set_enabled` | `std_srvs/srv/SetBool` |

`~/set_enabled` with `false` pauses exploration and requests cancellation of the
node's own goal, including a goal accepted after the disable request. With
`true`, exploration resumes from cached inputs. The node stays alive after
frontier exhaustion and searches again when the map, robot cell/heading,
blacklist, or tuning changes. Nav2 discovery never blocks construction.

Maps are cached even before the first pose arrives. Map QoS defaults to reliable,
transient local, depth one for latched sources. Set `map_durability` to `volatile`
for publishers offering volatile durability. Pose subscriptions default to best
effort, volatile, depth one, compatible with reliable and best-effort publishers.
See the [ROS 2 QoS compatibility rules](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html).

Fresh, transformable primary odometry takes precedence over the optional stamped
pose. Each source must provide a valid frame, positive timestamp, finite position,
and nonzero finite quaternion. Poses are transformed into the map frame at their
timestamp; missing TF or stale inputs defer navigation. This supports distinct
`map` and `odom` frames as described by [REP 105](https://www.ros.org/reps/rep-0105.html).

## Search and execution

The search traverses known cells with occupancy values 0–98. Values 99 and 100
are treated as inscribed/lethal obstacles, and -1 is unknown. Diagonal steps
require both intervening orthogonal cells to be traversable. Frontier cells
must border the robot's reachable component; the default two-neighbor threshold
detects ordinary straight boundaries. Length is approximated by cell count times
map resolution.

A reachable cell beside an actual frontier member anchors refinement. Candidates
remain in the reachable component and within a circular `goal_clearance_radius_m`
of that anchor. Lower map cost wins, then greater Euclidean clearance from known
obstacles, then proximity to the anchor. Scoring combines boundary length, a
bounded distance penalty, and a forward-heading bonus. Both the boundary
representative and final goal must satisfy the robot-distance bounds. Blacklisting
records the goal and boundary point, leaving other parts of a large frontier
available for another attempt.

`robot_radius_m=0` performs cell-level checks and assumes the input already
accounts for robot clearance. For a raw occupancy map, set a suitable circular
radius. A nonzero radius conservatively excludes paths close to known obstacles
and rejects goal footprints touching unknown cells or extending outside the map.
This circular approximation does not replace Nav2's footprint and path collision
checks. Bounded seed recovery can cross an unknown patch under the robot, but
cannot tunnel through an intervening obstacle.

Map dimensions, data length, occupancy range, resolution, origin, and frame are
validated before replacing the active map. Origin yaw is supported; tilted grids
are rejected. One joined worker owns reusable search buffers and reads immutable
snapshots. At most one active job, one replacement, and one result are retained.
Unchanged maps reuse clearance buffers and cached search decisions. New maps are
coalesced while a search runs. Before dispatch, the path and boundary are checked
against the latest map with a 10 ms validation budget; invalid or over-budget
results are replanned.

Goals have explicit sending, active, and canceling states, an owned handle, and
generation/mission identifiers. Stale callbacks cannot alter a newer goal. Only
the owned goal is canceled. Cancellation acceptance is not terminal under the
[ROS action contract](https://design.ros2.org/articles/actions.html); a replacement
waits for a terminal result/status. Rejected or unanswered cancellation keeps the
goal tracked. An unanswered send to a still-discovered server retains one pending
request so late acceptance can be canceled. Loss of server discovery allows
retirement after the communication timeout. ROS graph discovery does not provide
a network-level liveness guarantee.

Startup delays, goal timeouts, retry backoff, pose freshness, and blacklist expiry
use ROS time, including `use_sim_time`. Pausing simulation does not consume these
deadlines. A backward clock jump or clock-source change cancels the old mission,
clears cached inputs and blacklists, and requires a fresh map and pose.
Communication cleanup uses steady time and remains responsive during a pause.

Successful navigation and frontier exhaustion request map snapshots. Saves are
coalesced into one in-flight request, use the subscription's resolved map topic,
and require a successful response. An unavailable or failing saver is retried;
timed-out requests are removed. An old response cannot acknowledge a newer save
or a different exhaustion episode. Save failures do not stop exploration.

## Parameters

These parameters are read-only after construction. Restart to change them; use
the enable service to pause/resume instead of changing `start_enabled`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `map_topic` | `map` | OccupancyGrid input. |
| `odom_topic` | `localization/odometry/odom` | Primary pose source; empty disables. |
| `pose_topic` | `""` | Optional fallback; at least one pose source is required. |
| `navigation_action` | `navigate_to_pose` | Nav2 action endpoint. |
| `map_saver_service` | `map_saver/save_map` | SaveMap endpoint. |
| `map_path` | `/tmp/maps` | Output filename prefix passed to the map saver. |
| `start_enabled` | `true` | Initial exploration state. |
| `startup_delay_sec` | `0.0` | ROS-time delay before the first search. |
| `max_map_cells` | `16000000` | Bound on width × height and search memory. |
| `map_durability` | `transient_local` | `transient_local` or `volatile`. |
| `map_reliability` | `reliable` | `reliable` or `best_effort`. |
| `pose_reliability` | `best_effort` | `reliable` or `best_effort`. |

These tuning parameters support validated runtime updates. Use an atomic update
when changing related bounds together. Distances, durations, and weights must be
finite, nonnegative, and at most 1,000,000. Minimum distance must not exceed the
positive maximum. Pose/communication timeouts and save retry intervals must be
positive.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `min_frontier_length_m` | `0.25` | Minimum estimated boundary length. |
| `min_distance_to_frontier_m` | `0.75` | Minimum distance to representative and goal. |
| `max_distance_to_frontier_m` | `40.0` | Maximum distance to representative and goal. |
| `frontier_size_weight` | `1.0` | Boundary length score multiplier. |
| `frontier_distance_weight` | `0.35` | Distance penalty weight. |
| `frontier_distance_cap_m` | `20.0` | Scoring distance clamp. |
| `forward_weight` | `2.0` | Bounded forward-heading bonus; 0 disables. |
| `min_free_threshold` | `2` | Required reachable eight-neighbors; integer in [1, 8]. |
| `goal_clearance_radius_m` | `1.5` | Circular refinement radius around the approach anchor. |
| `robot_radius_m` | `0.0` | Circular footprint radius; see collision assumptions above. |
| `seed_search_radius_m` | `1.0` | Recovery radius for an invalid robot seed cell. |
| `blacklist_radius_m` | `1.0` | Exclusion radius around failed goals and boundary points. |
| `blacklist_duration_sec` | `60.0` | ROS-time lifetime; at most 1024 entries retained. |
| `max_goal_failures` | `0` | Failures near one place before it is blacklisted for the rest of the mission; 0 never. |
| `goal_timeout_sec` | `300.0` | ROS-time deadline to cancel/blacklist an overdue goal; 0 disables. |
| `pose_timeout_sec` | `1.0` | Maximum ROS-time pose age. |
| `retry_delay_sec` | `5.0` | ROS-time backoff after a goal finishes or is rejected. |
| `action_response_timeout_sec` | `10.0` | Steady-time goal-response/discovery-loss deadline. |
| `cancel_timeout_sec` | `2.0` | Cancel retry interval and terminal-status/result grace period. |
| `save_timeout_sec` | `5.0` | Steady-time SaveMap response deadline. |
| `save_retry_sec` | `2.0` | Steady-time delay between save attempts. |

### Migration from earlier versions of this fork

- Default endpoints are relative. A node in `/tb1` uses `/tb1/map`,
  `/tb1/navigate_to_pose`, `/tb1/map_saver/save_map`, and `/tb1/frontiers`.
- The saver default changed from `map_server/save_map` to `map_saver/save_map`.
  Set `map_saver_service` explicitly for a differently named server.
- Streaming volatile map publishers require `map_durability:=volatile`.
- Pose timestamps and TF are enforced. Frame mismatches, stale poses, and
  zero-stamped poses no longer start navigation.
- `min_free_threshold` defaults to two instead of four, and refinement starts
  beside the actual boundary instead of its centroid.

## Development

Source a ROS 2 installation with Nav2 dependencies, then run from the workspace:

```bash
colcon build --packages-select auto_mapper
colcon test --packages-select auto_mapper
colcon test-result --verbose
```

The default build type is `RelWithDebInfo` unless explicitly set. Tests include
ROS-free scoring, grid, reachability, footprint, cancellation, and worker-lifetime
checks, plus in-process ROS tests with fake action/save servers. ROS tests use
domain 178 and unique namespaces; they do not navigate hardware or write maps.
Ament formatting and lint checks are enabled.

To run the core search tests with memory and undefined-behavior instrumentation,
from the package directory with GoogleTest installed:

```bash
g++ -std=c++17 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Iinclude src/frontier_search.cpp src/search_worker.cpp test/test_frontier_search.cpp \
  -lgtest -lgtest_main -pthread -o /tmp/test_auto_mapper_search
/tmp/test_auto_mapper_search
```

The public `AutoMapper` class supports in-process testing. ROS callbacks use the
default mutually exclusive callback group, including clock and TF updates. The
node disables the private ROS clock executor; the worker never accesses node state.
Remove the node from its executor before destruction.

## Acknowledgements

- Original [auto_mapper](https://github.com/Omar-Salem/auto_mapper) by Omar Salem.
- Frontier BFS pattern from [m-explore](https://github.com/hrnr/m-explore).
