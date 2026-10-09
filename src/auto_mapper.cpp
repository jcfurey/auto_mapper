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
//
// Derived from auto_mapper by Omar Salem (Apache-2.0),
// https://github.com/Omar-Salem/auto_mapper

#include "auto_mapper/auto_mapper.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "auto_mapper/search_worker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace auto_mapper
{
namespace
{
using SteadyClock = std::chrono::steady_clock;
using Navigate = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
using Navigator = rclcpp_action::Client<Navigate>;
using SaveMap = nav2_msgs::srv::SaveMap;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Marker = visualization_msgs::msg::Marker;

static_assert(kFreeSpace == nav2_costmap_2d::FREE_SPACE);
static_assert(kInscribedInflatedObstacle == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
static_assert(kLethalObstacle == nav2_costmap_2d::LETHAL_OBSTACLE);
static_assert(kNoInformation == nav2_costmap_2d::NO_INFORMATION);

SteadyClock::time_point after(double seconds)
{
  return SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(seconds));
}

struct Config
{
  SearchParams search;
  double blacklist_duration{60.0};
  // Failures near one place before it is blacklisted for the mission; 0 never.
  double max_goal_failures{0.0};
  double goal_timeout{300.0};
  double pose_timeout{1.0};
  double retry_delay{5.0};
  double response_timeout{10.0};
  double cancel_timeout{2.0};
  double save_timeout{5.0};
  double save_retry{2.0};
};

template<class Visitor>
void config_parameters(Config & config, Visitor visit)
{
  visit("min_frontier_length_m", config.search.min_length);
  visit("min_distance_to_frontier_m", config.search.min_distance);
  visit("max_distance_to_frontier_m", config.search.max_distance);
  visit("frontier_size_weight", config.search.score.size_weight);
  visit("frontier_distance_weight", config.search.score.distance_weight);
  visit("frontier_distance_cap_m", config.search.score.distance_cap_m);
  visit("forward_weight", config.search.score.forward_weight);
  visit("min_free_threshold", config.search.min_free_neighbors);
  visit("goal_clearance_radius_m", config.search.clearance_radius);
  visit("robot_radius_m", config.search.robot_radius);
  visit("seed_search_radius_m", config.search.seed_search_radius);
  visit("blacklist_radius_m", config.search.blacklist_radius);
  visit("blacklist_duration_sec", config.blacklist_duration);
  visit("max_goal_failures", config.max_goal_failures);
  visit("goal_timeout_sec", config.goal_timeout);
  visit("pose_timeout_sec", config.pose_timeout);
  visit("retry_delay_sec", config.retry_delay);
  visit("action_response_timeout_sec", config.response_timeout);
  visit("cancel_timeout_sec", config.cancel_timeout);
  visit("save_timeout_sec", config.save_timeout);
  visit("save_retry_sec", config.save_retry);
}

std::string validate_config(const Config & config)
{
  const auto search_error = validate_search_params(config.search);
  if (!search_error.empty()) {return search_error;}
  const std::array<double, 8> durations = {
    config.blacklist_duration, config.goal_timeout, config.pose_timeout, config.retry_delay,
    config.response_timeout, config.cancel_timeout, config.save_timeout, config.save_retry};
  for (double duration : durations) {
    if (!std::isfinite(duration) || duration < 0.0 || duration > 1e6) {
      return "durations must be finite and in [0, 1000000] seconds";
    }
  }
  if (!std::isfinite(config.max_goal_failures) || config.max_goal_failures < 0.0 ||
    config.max_goal_failures > 1000.0 ||
    config.max_goal_failures != std::floor(config.max_goal_failures))
  {
    return "max_goal_failures must be a whole number in [0, 1000]";
  }
  if (config.pose_timeout == 0.0 || config.response_timeout == 0.0 ||
    config.cancel_timeout == 0.0 || config.save_timeout == 0.0 || config.save_retry == 0.0)
  {
    return "pose, response, cancellation, and save timeouts/retries must be positive";
  }
  return {};
}

bool apply_parameters(Config & config, const std::vector<rclcpp::Parameter> & parameters)
{
  bool changed = false;
  config_parameters(config, [&](const char * name, auto & field) {
      for (const auto & parameter : parameters) {
        if (parameter.get_name() != name) {continue;}
        if constexpr (std::is_integral_v<std::decay_t<decltype(field)>>) {
          const auto value = parameter.as_int();
          if (value < 1 || value > 8) {
            throw std::invalid_argument("min_free_threshold must be in [1, 8]");
          }
          field = static_cast<int>(value);
        } else {
          field = parameter.as_double();
        }
        changed = true;
      }
  });
  return changed;
}

bool normalize_pose(geometry_msgs::msg::Pose & pose)
{
  auto & p = pose.position;
  auto & q = pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    return false;
  }
  const double length = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(length) || length < 1e-6) {return false;}
  q.x /= length; q.y /= length; q.z /= length; q.w /= length;
  return true;
}

bool valid_frame(const std::string & frame)
{
  return !frame.empty() && frame.front() != '/' &&
         frame.find_first_of(" \t\r\n") == std::string::npos;
}

rclcpp::NodeOptions executor_clock_options(rclcpp::NodeOptions options)
{
  // Search never blocks the node executor, so /clock can share it with the
  // mission callbacks. This also avoids starting a private clock executor in
  // the base constructor before startup parameters have been validated.
  options.use_clock_thread(false);
  return options;
}
}  // namespace

class AutoMapper::Impl
{
public:
  explicit Impl(AutoMapper & node)
  : node_(node)
  {
    const auto map_topic = read_only<std::string>("map_topic", "map");
    const auto odom_topic = read_only<std::string>("odom_topic", "localization/odometry/odom");
    const auto pose_topic = read_only<std::string>("pose_topic", "");
    navigation_action_ = read_only<std::string>("navigation_action", "navigate_to_pose");
    const auto save_service = read_only<std::string>("map_saver_service", "map_saver/save_map");
    map_path_ = read_only<std::string>("map_path", "/tmp/maps");
    enabled_ = read_only<bool>("start_enabled", true);
    startup_delay_ = read_only<double>("startup_delay_sec", 0.0);
    const int64_t max_cells = read_only<int64_t>("max_map_cells", 16000000);
    const auto durability = read_only<std::string>("map_durability", "transient_local");
    const auto map_reliability = read_only<std::string>("map_reliability", "reliable");
    const auto pose_reliability = read_only<std::string>("pose_reliability", "best_effort");
    if (map_topic.empty() || (odom_topic.empty() && pose_topic.empty()) ||
      navigation_action_.empty() || save_service.empty() || map_path_.empty())
    {
      throw std::invalid_argument(
          "map, navigation, saver, output path and at least one pose source are required");
    }
    if (!std::isfinite(startup_delay_) || startup_delay_ < 0.0 || startup_delay_ > 1e6 ||
      max_cells <= 0 || static_cast<uint64_t>(max_cells) >= std::numeric_limits<uint32_t>::max())
    {
      throw std::invalid_argument("invalid startup_delay_sec or max_map_cells");
    }
    max_map_cells_ = static_cast<std::size_t>(max_cells);
    config_parameters(config_, [&](const char * name, auto & field) {
        using Type = std::decay_t<decltype(field)>;
        if constexpr (std::is_integral_v<Type>) {
          const auto value = node_.declare_parameter<int64_t>(name, field);
          if (value < 1 || value > 8) {
            throw std::invalid_argument("min_free_threshold must be in [1, 8]");
          }
          field = static_cast<Type>(value);
        } else {
          field = node_.declare_parameter<Type>(name, field);
        }
    });
    const auto error = validate_config(config_);
    if (!error.empty()) {throw std::invalid_argument(error);}
    use_sim_time_ = node_.get_parameter("use_sim_time").as_bool();

    parameter_validator_ = node_.add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
          auto candidate = config_;
          apply_parameters(candidate, parameters);
          result.reason = validate_config(candidate);
          result.successful = result.reason.empty();
        } catch (const std::exception & exception) {
          result.successful = false;
          result.reason = exception.what();
        }
        return result;
      });
    parameter_applier_ = node_.add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        if (apply_parameters(config_, parameters)) {
          ++config_revision_;
          invalidate_search();
        }
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "use_sim_time" && parameter.as_bool() != use_sim_time_) {
            use_sim_time_ = parameter.as_bool();
            clock_changed_ = true;
          }
        }
      });

    rclcpp::QoS map_qos(1);
    if (durability == "transient_local") {
      map_qos.transient_local();
    } else if (durability != "volatile") {throw std::invalid_argument("invalid map_durability");}
    set_reliability(map_qos, map_reliability);
    rclcpp::QoS pose_qos(1);
    set_reliability(pose_qos, pose_reliability);
    map_subscription_ = node_.create_subscription<OccupancyGrid>(
      map_topic, map_qos,
      [this](OccupancyGrid::ConstSharedPtr message) {
        receive_map(std::move(message));
      });
    if (!odom_topic.empty()) {
      odom_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, pose_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          PoseStamped pose;
          pose.header = message->header;
          pose.pose = message->pose.pose;
          receive_pose(std::move(pose), odom_pose_);
        });
    }
    if (!pose_topic.empty()) {
      pose_subscription_ = node_.create_subscription<PoseStamped>(
        pose_topic, pose_qos, [this](PoseStamped::ConstSharedPtr message) {
          receive_pose(*message, alternative_pose_);
        });
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
    // Use this node's executor; transform queries below never block waiting for TF.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, &node_, false);
    markers_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 1);
    navigator_ = rclcpp_action::create_client<Navigate>(&node_, navigation_action_);
    saver_ = node_.create_client<SaveMap>(save_service);
    enable_service_ = node_.create_service<std_srvs::srv::SetBool>(
      "~/set_enabled", [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        const bool changed = enabled_ != request->data;
        enabled_ = request->data;
        if (changed) {
          invalidate_search();
          if (!enabled_) {
            request_cancel();
            publish_markers({});
          } else {
            next_explore_ = node_.now().seconds();
          }
        }
        response->success = true;
        response->message =
        enabled_ ? "exploration enabled" :
        "exploration disabled; owned goal cancellation requested";
      });
    worker_ = std::make_unique<SearchWorker>();
    // Wall polling keeps cancellation/service cleanup responsive when simulation
    // is paused. Mission deadlines and blacklist expiry use node_.now() only.
    timer_ = node_.create_wall_timer(std::chrono::milliseconds(50), [this] {tick();});
    RCLCPP_INFO(node_.get_logger(), "Exploration %s; waiting asynchronously for map, pose and Nav2",
      enabled_ ? "enabled" : "disabled");
  }

  ~Impl()
  {
    timer_.reset();
    worker_.reset();  // Cooperative stop and join before releasing snapshot storage.
    if (goal_ && goal_->handle && rclcpp::ok(node_.get_node_base_interface()->get_context())) {
      try {navigator_->async_cancel_goal(goal_->handle);} catch (const std::exception &) {}
    }
    if (save_request_) {saver_->remove_pending_request(save_request_->id);}
    navigator_.reset();
  }

private:
  template<class T>
  T read_only(const char * name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Set at construction; restart the node to change this parameter.";
    return node_.declare_parameter<T>(name, value, descriptor);
  }

  static void set_reliability(rclcpp::QoS & qos, const std::string & reliability)
  {
    if (reliability == "best_effort") {qos.best_effort();} else if (reliability == "reliable") {
      qos.reliable();
    } else {throw std::invalid_argument("reliability must be reliable or best_effort");}
  }

  void invalidate_search()
  {
    if (worker_) {worker_->cancel();}
    planning_id_.reset();
    last_search_.reset();
  }

  void receive_map(OccupancyGrid::ConstSharedPtr message)
  {
    auto origin = message->info.origin;
    if (!valid_frame(message->header.frame_id) || !normalize_pose(origin)) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected map with invalid frame or origin pose");
      return;
    }
    const auto & q = origin.orientation;
    // A rotated XY grid is supported. A tilted grid cannot represent a planar
    // Nav2 surface without resampling, so reject roll/pitch explicitly.
    if (std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected nonplanar OccupancyGrid origin");
      return;
    }
    GridGeometry geometry{message->info.width, message->info.height, message->info.resolution,
      origin.position.x, origin.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)};
    const auto error = validate_grid(geometry, message->data, max_map_cells_);
    if (!error.empty()) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected OccupancyGrid: %s", error.c_str());
      return;
    }
    if (!map_frame_.empty() && message->header.frame_id != map_frame_) {
      reset_mission(node_.now().seconds(), false);
    }
    if (grid_ && map_frame_ == message->header.frame_id && grid_->geometry == geometry &&
      *grid_->data == message->data)
    {
      return;  // Header-only republishing does not invalidate an exhausted-map cache.
    }
    map_frame_ = message->header.frame_id;
    auto storage = std::shared_ptr<const std::vector<int8_t>>(message, &message->data);
    grid_ = std::make_shared<const Grid>(Grid{geometry, std::move(storage)});
    ++map_revision_;
    // Let an active search finish. Its approach path is checked against this
    // newer grid before dispatch; a high-rate publisher cannot starve planning.
  }

  void receive_pose(PoseStamped pose, std::optional<PoseStamped> & destination)
  {
    if (pose.header.stamp.sec < 0 || pose.header.stamp.nanosec >= 1000000000u) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected pose with invalid timestamp fields");
      return;
    }
    const double stamp = rclcpp::Time(pose.header.stamp).seconds();
    if (!valid_frame(pose.header.frame_id) || stamp <= 0.0 || !normalize_pose(pose.pose)) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected pose with invalid frame, timestamp or numeric values");
      return;
    }
    if (destination && stamp < rclcpp::Time(destination->header.stamp).seconds()) {return;}
    destination = std::move(pose);
  }

  std::optional<Pose2> map_pose(double time)
  {
    for (const auto * source : {&odom_pose_, &alternative_pose_}) {
      if (!*source) {continue;}
      const auto & pose = **source;
      const double age = time - rclcpp::Time(pose.header.stamp).seconds();
      if (age < -0.1 || age > config_.pose_timeout) {continue;}
      PoseStamped transformed;
      try {
        if (pose.header.frame_id == map_frame_) {transformed = pose;} else {
          transformed = tf_buffer_->transform(pose, map_frame_, tf2::durationFromSec(0.0));
        }
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
          "Waiting for pose transform into '%s': %s", map_frame_.c_str(), error.what());
        continue;
      }
      if (!normalize_pose(transformed.pose)) {continue;}
      const auto & p = transformed.pose.position;
      const auto & q = transformed.pose.orientation;
      return Pose2{{p.x, p.y}, yaw_from_quaternion(q.x, q.y, q.z, q.w)};
    }
    return std::nullopt;
  }

  void reset_mission(double time, bool clear_pose)
  {
    ++mission_epoch_;
    request_cancel();
    invalidate_search();
    blacklist_.clear();
    failure_history_.clear();
    ++blacklist_revision_;
    if (clear_pose) {odom_pose_.reset(); alternative_pose_.reset();}
    grid_.reset();
    has_navigated_ = false;
    exhausted_ = false;
    exhaustion_saved_ = false;
    ++exhaustion_episode_;
    save_wanted_ = false;
    save_for_exhaustion_ = false;
    requested_exhaustion_episode_.reset();
    if (save_request_) {saver_->remove_pending_request(save_request_->id); save_request_.reset();}
    ++save_generation_;
    mission_initialized_ = time > 0.0;
    next_explore_ = time + startup_delay_;
    last_time_ = time;
    publish_markers({});
  }

  struct SearchKey
  {
    uint64_t map, config, blacklist;
    uint32_t cell;
    int heading;
    bool operator==(const SearchKey & other) const
    {
      return map == other.map && config == other.config && blacklist == other.blacklist &&
             cell == other.cell && heading == other.heading;
    }
  };

  void tick()
  {
    const double time = node_.now().seconds();
    if (clock_changed_ || (mission_initialized_ && time < last_time_)) {
      clock_changed_ = false;
      reset_mission(time, true);
    }
    if (!mission_initialized_ && time > 0.0) {
      mission_initialized_ = true;
      next_explore_ = time + startup_delay_;
    }
    last_time_ = time;
    update_goal(time);
    update_save();
    if (!mission_initialized_ || !enabled_ || goal_ || time < next_explore_ || !grid_) {return;}
    const auto robot = map_pose(time);
    if (!robot) {return;}
    const std::size_t previous_size = blacklist_.size();
    prune_blacklist(blacklist_, time, config_.blacklist_duration);
    if (blacklist_.size() != previous_size) {++blacklist_revision_; invalidate_search();}
    if (planning_id_) {
      auto completion = worker_->poll();
      if (!completion) {return;}
      if (completion->job.id != *planning_id_) {return;}
      planning_id_.reset();
      if (!completion->error.empty()) {
        RCLCPP_ERROR(node_.get_logger(), "Frontier search failed: %s", completion->error.c_str());
        last_search_.reset();
        next_explore_ = time + config_.retry_delay;
        return;
      }
      auto & result = completion->result;
      if (result.canceled) {last_search_.reset(); return;}
      if (!result.valid_seed) {return;}
      if (result.selected) {
        const auto validation_deadline = after(0.01);
        if (!path_still_valid(result, *completion->job.grid, *grid_, *robot, config_.search,
          [validation_deadline] {return SteadyClock::now() >= validation_deadline;}))
        {
          last_search_.reset();
          return;
        }
        exhausted_ = false;
        exhaustion_saved_ = false;
        publish_markers(result.frontiers);
        send_goal(*result.selected, *robot, time);
        return;
      }
      publish_markers({});
      if (result.frontier_count == 0 && completion->job.map_revision == map_revision_) {
        if (!exhausted_) {exhausted_ = true; exhaustion_saved_ = false; ++exhaustion_episode_;}
        if (has_navigated_ && !exhaustion_saved_) {want_save(true);}
      } else if (result.frontier_count > 0) {
        exhausted_ = false;
        exhaustion_saved_ = false;
      }
    }
    if (!navigator_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "Waiting for navigation action server");
      return;
    }
    const auto cell = grid_->geometry.world_to_cell(robot->position);
    if (!cell) {return;}
    const SearchKey key{map_revision_, config_revision_, blacklist_revision_, *cell,
      static_cast<int>(std::floor(robot->yaw / 0.2))};
    if (last_search_ && *last_search_ == key) {return;}
    SearchJob job;
    job.map_revision = map_revision_;
    job.grid = grid_;
    job.robot = *robot;
    job.params = config_.search;
    job.blacklist = blacklist_;
    planning_id_ = worker_->submit(std::move(job));
    last_search_ = key;
  }

  enum class GoalPhase {Sending, Active, Canceling};
  struct ActiveGoal
  {
    uint64_t generation{0};
    uint64_t epoch{0};
    GoalPhase phase{GoalPhase::Sending};
    GoalHandle::SharedPtr handle;
    Frontier frontier;
    double started{0.0};
    SteadyClock::time_point response_deadline;
    SteadyClock::time_point cancel_deadline;
    SteadyClock::time_point cancel_retry;
    bool cancel_requested{false};
    bool cancel_sent{false};
    bool cancel_confirmed{false};
    bool terminal_confirmed{false};
    bool blacklisted{false};
  };

  void blacklist_goal()
  {
    if (!goal_ || goal_->blacklisted || goal_->epoch != mission_epoch_) {return;}
    goal_->blacklisted = true;
    const auto & f = goal_->frontier;
    const double now = node_.now().seconds();
    // A place that keeps failing (e.g. an unreachable frontier) would otherwise
    // be retried each time its blacklist entry expires, and never let the
    // mission reach exhaustion.
    const auto failures = count_failures(failure_history_, f.goal.x, f.goal.y,
        config_.search.blacklist_radius);
    const bool permanent = config_.max_goal_failures > 0.0 &&
      static_cast<double>(failures) >= config_.max_goal_failures;
    failure_history_.push_back({f.goal.x, f.goal.y, now});
    if (failure_history_.size() > 1024) {failure_history_.erase(failure_history_.begin());}
    if (permanent) {
      RCLCPP_WARN(node_.get_logger(), "Frontier near (%.2f, %.2f) failed %zu times; skipping it",
        f.goal.x, f.goal.y, failures);
    }
    blacklist_rejected_goal(blacklist_, f.goal.x, f.goal.y, f.boundary.x, f.boundary.y,
      now, permanent);
    constexpr std::size_t max_entries = 1024;
    if (blacklist_.size() > max_entries) {
      blacklist_.erase(blacklist_.begin(), blacklist_.begin() + (blacklist_.size() - max_entries));
    }
    ++blacklist_revision_;
  }

  void send_goal(const Frontier & frontier, Pose2 robot, double time)
  {
    if (!enabled_ || goal_ || !navigator_->action_server_is_ready()) {last_search_.reset(); return;}
    ActiveGoal state;
    state.generation = ++goal_generation_;
    state.epoch = mission_epoch_;
    state.frontier = frontier;
    state.started = time;
    state.response_deadline = after(config_.response_timeout);
    state.cancel_retry = SteadyClock::now();
    goal_ = state;
    Navigate::Goal message;
    message.pose.header.frame_id = map_frame_;
    message.pose.header.stamp = node_.now();
    message.pose.pose.position.x = frontier.goal.x;
    message.pose.pose.position.y = frontier.goal.y;
    const double yaw = frontier_goal_yaw(frontier.goal.x, frontier.goal.y,
      frontier.boundary.x, frontier.boundary.y, robot.position.x, robot.position.y, robot.yaw);
    message.pose.pose.orientation.z = std::sin(yaw / 2.0);
    message.pose.pose.orientation.w = std::cos(yaw / 2.0);
    const auto generation = state.generation;
    std::weak_ptr<Navigator> sending_client = navigator_;
    Navigator::SendGoalOptions options;
    options.goal_response_callback = [this, generation,
        sending_client](GoalHandle::SharedPtr handle) {
        if (!goal_ || goal_->generation != generation) {
          if (handle) {
            if (auto client = sending_client.lock()) {
              try {client->async_cancel_goal(handle);} catch (const std::exception &) {}
            }
          }
          return;
        }
        if (!handle) {
          const bool current_mission = goal_->epoch == mission_epoch_;
          goal_.reset();
          if (current_mission) {
            next_explore_ = std::max(next_explore_, node_.now().seconds() + config_.retry_delay);
          }
          last_search_.reset();
          publish_markers({});
          return;
        }
        goal_->handle = std::move(handle);
        goal_->phase = GoalPhase::Active;
        if (goal_->epoch == mission_epoch_) {has_navigated_ = true;}
        if (!enabled_ || goal_->cancel_requested || goal_->epoch != mission_epoch_) {
          request_cancel();
        }
      };
    options.feedback_callback = [this, generation](
      GoalHandle::SharedPtr, const std::shared_ptr<const Navigate::Feedback> feedback)
      {
        if (!goal_ || goal_->generation != generation) {return;}
        RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
          "Distance remaining: %.2f m", feedback->distance_remaining);
      };
    options.result_callback = [this, generation](const GoalHandle::WrappedResult & result) {
        if (!goal_ || goal_->generation != generation ||
          (goal_->handle && goal_->handle->get_goal_id() != result.goal_id)) {return;}
        const bool current_mission = goal_->epoch == mission_epoch_;
        if (current_mission && !goal_->cancel_requested) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            want_save(false);
          } else if (result.code == rclcpp_action::ResultCode::ABORTED) {blacklist_goal();}
        }
        goal_.reset();
        publish_markers({});
        invalidate_search();
        if (current_mission) {
          next_explore_ = std::max(next_explore_, node_.now().seconds() + config_.retry_delay);
        }
      };
    try {
      navigator_->async_send_goal(message, options);
      RCLCPP_INFO(node_.get_logger(), "Navigating to frontier approach (%.2f, %.2f)",
        frontier.goal.x, frontier.goal.y);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_.get_logger(), "Failed to send goal: %s", error.what());
      retire_goal();
    }
  }

  void request_cancel()
  {
    if (!goal_) {return;}
    goal_->cancel_requested = true;
    if (!goal_->handle || goal_->cancel_sent || SteadyClock::now() < goal_->cancel_retry) {return;}
    goal_->phase = GoalPhase::Canceling;
    goal_->cancel_sent = true;
    goal_->cancel_deadline = after(config_.cancel_timeout);
    const auto generation = goal_->generation;
    const auto id = goal_->handle->get_goal_id();
    try {
      navigator_->async_cancel_goal(goal_->handle,
        [this, generation, id](Navigator::CancelResponse::SharedPtr response) {
          if (!goal_ || goal_->generation != generation) {return;}
          const bool accepted = std::any_of(
            response->goals_canceling.begin(), response->goals_canceling.end(),
            [&id](const auto & info) {return info.goal_id.uuid == id;});
          goal_->terminal_confirmed =
          response->return_code == Navigator::CancelResponse::ERROR_GOAL_TERMINATED ||
          response->return_code == Navigator::CancelResponse::ERROR_UNKNOWN_GOAL_ID;
          if (accepted || goal_->terminal_confirmed) {
            goal_->cancel_confirmed = true;
            goal_->cancel_deadline = after(config_.cancel_timeout);
          } else {
            // A rejected cancel leaves the owned goal active. Do not issue a
            // replacement; retry only after this request has returned.
            goal_->cancel_sent = false;
            goal_->cancel_retry = after(config_.cancel_timeout);
          }
        });
    } catch (const std::exception & error) {
      goal_->cancel_sent = false;
      goal_->cancel_retry = after(config_.cancel_timeout);
      RCLCPP_WARN(node_.get_logger(), "Cancellation request failed: %s", error.what());
    }
  }

  void retire_goal()
  {
    // Called by the scheduler, outside client callbacks. Replacing the client
    // releases unanswered action requests as well as stopping old callbacks.
    const bool current_mission = goal_ && goal_->epoch == mission_epoch_;
    ++goal_generation_;
    goal_.reset();
    navigator_ = rclcpp_action::create_client<Navigate>(&node_, navigation_action_);
    invalidate_search();
    publish_markers({});
    if (current_mission) {
      next_explore_ = std::max(next_explore_, node_.now().seconds() + config_.retry_delay);
    }
  }

  void update_goal(double time)
  {
    if (!goal_) {return;}
    const auto real_time = SteadyClock::now();
    if (!navigator_->action_server_is_ready() && real_time >= goal_->response_deadline) {
      blacklist_goal();
      retire_goal();
      return;
    }
    if (goal_->phase == GoalPhase::Sending && real_time >= goal_->response_deadline) {
      // With a still-discovered server the request might yet be accepted. Keep
      // its one bounded pending response so a late acceptance can be canceled.
      goal_->cancel_requested = true;
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "Waiting for outstanding goal response before replacing the goal");
      // A response lost for good would stall exploration for the node's life.
      // After another full deadline give up on it: a server that accepted it
      // late preempts or rejects the next goal, so goals still can't overlap.
      if (real_time >= goal_->response_deadline +
        std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(config_.response_timeout)))
      {
        RCLCPP_WARN(node_.get_logger(), "Goal response lost; replacing the goal");
        retire_goal();
        return;
      }
    }
    if (config_.goal_timeout > 0.0 && time >= goal_->started + config_.goal_timeout &&
      !goal_->cancel_requested)
    {
      blacklist_goal();
      request_cancel();
    }
    if (!enabled_ || goal_->epoch != mission_epoch_ ||
      (goal_->phase == GoalPhase::Active && !map_pose(time)))
    {
      request_cancel();
    }
    if (goal_->cancel_requested) {request_cancel();}
    if (goal_->cancel_confirmed && real_time >= goal_->cancel_deadline) {
      const auto status = goal_->handle->get_status();
      using Status = action_msgs::msg::GoalStatus;
      if (goal_->terminal_confirmed || status == Status::STATUS_CANCELED ||
        status == Status::STATUS_SUCCEEDED || status == Status::STATUS_ABORTED)
      {
        retire_goal();
      } else {
        // CANCELING is an active ROS action state. Acceptance of cancellation
        // alone does not authorize dispatching a replacement goal.
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
          "Cancellation accepted; waiting for navigation to finish stopping");
      }
    }
  }

  struct SaveRequest
  {
    int64_t id;
    uint64_t generation;
    uint64_t ticket;
    uint64_t epoch;
    uint64_t episode;
    bool exhaustion;
    SteadyClock::time_point deadline;
  };

  void want_save(bool exhaustion)
  {
    if (exhaustion) {
      if (exhaustion_saved_ || requested_exhaustion_episode_ == exhaustion_episode_) {return;}
      requested_exhaustion_episode_ = exhaustion_episode_;
    }
    ++save_ticket_;
    save_wanted_ = true;
    save_for_exhaustion_ = save_for_exhaustion_ || exhaustion;
  }

  void update_save()
  {
    const auto real_time = SteadyClock::now();
    if (save_request_ && real_time >= save_request_->deadline) {
      saver_->remove_pending_request(save_request_->id);
      save_request_.reset();
      ++save_generation_;
      next_save_ = after(config_.save_retry);
      RCLCPP_WARN(node_.get_logger(), "Map save timed out; request removed and retry scheduled");
    }
    if (!save_wanted_ || save_request_ || real_time < next_save_ || !saver_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<SaveMap::Request>();
    request->map_topic = map_subscription_->get_topic_name();
    request->map_url = map_path_;
    request->image_format = "pgm";
    request->map_mode = "trinary";
    const auto generation = ++save_generation_;
    const auto ticket = save_ticket_;
    try {
      auto future = saver_->async_send_request(request,
          [this, generation](rclcpp::Client<SaveMap>::SharedFuture response) {
            if (!save_request_ || save_request_->generation != generation) {return;}
            const auto completed = *save_request_;
            save_request_.reset();
            bool success = false;
            try {success = response.get()->result;} catch (const std::exception &) {}
            if (success && completed.epoch == mission_epoch_) {
              if (completed.exhaustion && exhausted_ && completed.episode == exhaustion_episode_) {
                exhaustion_saved_ = true;
              }
              if (completed.ticket == save_ticket_) {
                save_wanted_ = false;
                save_for_exhaustion_ = false;
              }
              RCLCPP_INFO(node_.get_logger(), "Map saved to %s", map_path_.c_str());
            } else {
              RCLCPP_WARN(node_.get_logger(), "Map save failed; retry scheduled");
            }
            next_save_ = after(config_.save_retry);
        });
      save_request_ = SaveRequest{future.request_id, generation, ticket, mission_epoch_,
        exhaustion_episode_, save_for_exhaustion_, after(config_.save_timeout)};
    } catch (const std::exception & error) {
      next_save_ = after(config_.save_retry);
      RCLCPP_WARN(node_.get_logger(), "Map save request failed: %s", error.what());
    }
  }

  void publish_markers(const std::vector<Frontier> & frontiers)
  {
    if (!markers_) {return;}
    visualization_msgs::msg::MarkerArray message;
    Marker clear;
    clear.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
    clear.header.stamp = node_.now();
    clear.ns = "frontiers";
    clear.action = Marker::DELETEALL;
    message.markers.push_back(clear);
    int32_t id = 0;
    for (const auto & frontier : frontiers) {
      Marker marker;
      marker.header = clear.header;
      marker.ns = "frontiers";
      marker.id = id++;
      marker.type = Marker::SPHERE;
      marker.action = Marker::ADD;
      marker.pose.position.x = frontier.goal.x;
      marker.pose.position.y = frontier.goal.y;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
      marker.color.g = marker.color.a = 1.0;
      message.markers.push_back(std::move(marker));
    }
    markers_->publish(message);
  }

  AutoMapper & node_;
  Config config_;
  std::string navigation_action_, map_path_, map_frame_;
  std::size_t max_map_cells_{0};
  double startup_delay_{0.0};
  bool enabled_{true};
  bool mission_initialized_{false};
  bool clock_changed_{false};
  bool use_sim_time_{false};
  double last_time_{0.0};
  double next_explore_{0.0};
  uint64_t mission_epoch_{0};
  uint64_t map_revision_{0}, config_revision_{0}, blacklist_revision_{0};
  std::shared_ptr<const Grid> grid_;
  std::optional<PoseStamped> odom_pose_, alternative_pose_;
  std::vector<BlacklistEntry> blacklist_;
  std::vector<BlacklistEntry> failure_history_;
  std::optional<SearchKey> last_search_;
  std::optional<uint64_t> planning_id_;
  std::unique_ptr<SearchWorker> worker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  Navigator::SharedPtr navigator_;
  std::optional<ActiveGoal> goal_;
  uint64_t goal_generation_{0};
  bool has_navigated_{false};
  bool exhausted_{false}, exhaustion_saved_{false};
  uint64_t exhaustion_episode_{0};
  std::optional<uint64_t> requested_exhaustion_episode_;
  bool save_wanted_{false}, save_for_exhaustion_{false};
  uint64_t save_generation_{0}, save_ticket_{0};
  SteadyClock::time_point next_save_ = SteadyClock::now();
  std::optional<SaveRequest> save_request_;
  rclcpp::Client<SaveMap>::SharedPtr saver_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_validator_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_applier_;
};

AutoMapper::AutoMapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("auto_mapper", executor_clock_options(options)),
  impl_(std::make_unique<Impl>(*this))
{}

AutoMapper::~AutoMapper() = default;

}  // namespace auto_mapper
