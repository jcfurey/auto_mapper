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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "auto_mapper/auto_mapper.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
using Navigate = nav2_msgs::action::NavigateToPose;
using ServerGoal = rclcpp_action::ServerGoalHandle<Navigate>;
using SaveMap = nav2_msgs::srv::SaveMap;
using namespace std::chrono_literals;

class AutoMapperNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static int sequence = 0;
    ns_ = "/auto_mapper_test_" + std::to_string(++sequence);
    helper_ = std::make_shared<rclcpp::Node>("harness", ns_);
    nav_node_ = std::make_shared<rclcpp::Node>("fake_navigation", ns_);
    executor_.add_node(helper_);
    executor_.add_node(nav_node_);
    clock_ = helper_->create_publisher<rosgraph_msgs::msg::Clock>("clock",
        rclcpp::QoS(1).best_effort());
    maps_ = helper_->create_publisher<nav_msgs::msg::OccupancyGrid>("map",
        rclcpp::QoS(1).transient_local());
    poses_ = helper_->create_publisher<geometry_msgs::msg::PoseStamped>("pose",
        rclcpp::SensorDataQoS());
    odometry_ = helper_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
    transforms_ = helper_->create_publisher<tf2_msgs::msg::TFMessage>("tf_static",
        rclcpp::QoS(1).transient_local());
    markers_ = helper_->create_subscription<visualization_msgs::msg::MarkerArray>("frontiers", 1,
        [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) {
          if (message->markers.size() > 1) {++marker_updates_;}
      });
    enable_ = helper_->create_client<std_srvs::srv::SetBool>("auto_mapper/set_enabled");
  }

  void TearDown() override
  {
    for (auto & goal : goals_) {
      if (goal->is_active()) {
        auto result = std::make_shared<Navigate::Result>();
        if (goal->is_canceling()) {goal->canceled(result);} else {goal->abort(result);}
      }
    }
    if (node_) {executor_.remove_node(node_); node_.reset();}
    executor_.remove_node(helper_);
    executor_.remove_node(nav_node_);
    held_.clear();
    goals_.clear();
    server_.reset();
    saver_.reset();
  }

  void make_node(std::vector<rclcpp::Parameter> extra = {}, std::vector<std::string> remaps = {})
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("odom_topic", ""), rclcpp::Parameter("pose_topic", "pose"),
      rclcpp::Parameter("retry_delay_sec", 0.2),
      rclcpp::Parameter("action_response_timeout_sec", 0.3),
      rclcpp::Parameter("cancel_timeout_sec", 0.15),
      rclcpp::Parameter("save_timeout_sec", 0.15),
      rclcpp::Parameter("save_retry_sec", 0.1)};
    for (auto & parameter : extra) {
      parameters.erase(std::remove_if(parameters.begin(), parameters.end(),
        [&](const auto & current) {
          return current.get_name() == parameter.get_name();
          }), parameters.end());
      parameters.push_back(std::move(parameter));
    }
    std::vector<std::string> arguments = {"--ros-args", "--log-level", "error", "-r",
      "__ns:=" + ns_,
      "-r", "/clock:=" + ns_ + "/clock", "-r", "/tf:=" + ns_ + "/tf",
      "-r", "/tf_static:=" + ns_ + "/tf_static"};
    for (const auto & remap : remaps) {arguments.push_back("-r"); arguments.push_back(remap);}
    node_ = std::make_shared<auto_mapper::AutoMapper>(
      rclcpp::NodeOptions().arguments(arguments).parameter_overrides(parameters));
    executor_.add_node(node_);
    ASSERT_TRUE(wait_for([&] {return node_->now().seconds() == time_;}));
  }

  void make_navigation()
  {
    server_ = rclcpp_action::create_server<Navigate>(nav_node_, "navigate_to_pose",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal>) {
          ++goal_requests_;
          return accept_goals_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                 rclcpp_action::GoalResponse::REJECT;
      },
        [this](std::shared_ptr<ServerGoal>) {
          ++cancel_requests_;
          return accept_cancels_ ? rclcpp_action::CancelResponse::ACCEPT :
                 rclcpp_action::CancelResponse::REJECT;
      },
        [this](std::shared_ptr<ServerGoal> goal) {goals_.push_back(std::move(goal));});
  }

  builtin_interfaces::msg::Time stamp() const
  {
    return rclcpp::Time(static_cast<int64_t>(time_ * 1e9), RCL_ROS_TIME);
  }

  void publish_clock()
  {
    rosgraph_msgs::msg::Clock message;
    message.clock = stamp();
    clock_->publish(message);
  }

  void spin_for(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    do {
      publish_clock();
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < end);
  }

  bool wait_for(const std::function<bool()> & predicate, double seconds = 3.0)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!predicate() && std::chrono::steady_clock::now() < end) {spin_for(0.01);}
    return predicate();
  }

  void publish_pose(double x = 2.5, double y = 10.5, const std::string & frame = "map")
  {
    latest_pose_.header.frame_id = frame;
    latest_pose_.header.stamp = stamp();
    latest_pose_.pose.position.x = x;
    latest_pose_.pose.position.y = y;
    latest_pose_.pose.orientation.w = 1.0;
    poses_->publish(latest_pose_);
  }

  nav_msgs::msg::OccupancyGrid scene(bool boundary = true)
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = stamp();
    map.info.width = map.info.height = 20;
    map.info.resolution = 1.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(400, 0);
    if (boundary) {
      for (unsigned y = 0; y < 20; ++y) {
        for (unsigned x = 10; x < 20; ++x) {map.data[y * 20 + x] = -1;}
      }
    }
    return map;
  }

  void advance(double seconds, bool refresh_pose = true)
  {
    time_ += seconds;
    publish_clock();
    if (refresh_pose && !latest_pose_.header.frame_id.empty()) {
      latest_pose_.header.stamp = stamp();
      poses_->publish(latest_pose_);
    }
    spin_for(0.06);
  }

  bool set_enabled(bool enabled)
  {
    if (!wait_for([&] {return enable_->service_is_ready();})) {return false;}
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = enable_->async_send_request(request);
    return wait_for([&] {
               return future.wait_for(0s) == std::future_status::ready;
        }) && future.get()->success;
  }

  void make_saver()
  {
    saver_ = helper_->create_service<SaveMap>("map_saver/save_map",
        [this](std::shared_ptr<rclcpp::Service<SaveMap>> service,
        std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<SaveMap::Request> request)
        {
          saved_topics_.push_back(request->map_topic);
          if (hold_saves_) {held_.push_back({service, header}); return;}
          SaveMap::Response response;
          response.result = !fail_saves_;
          service->send_response(*header, response);
      });
  }

  struct HeldResponse
  {
    std::shared_ptr<rclcpp::Service<SaveMap>> service;
    std::shared_ptr<rmw_request_id_t> header;
  };
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::string ns_;
  rclcpp::Node::SharedPtr helper_, nav_node_;
  std::shared_ptr<auto_mapper::AutoMapper> node_;
  rclcpp_action::Server<Navigate>::SharedPtr server_;
  std::vector<std::shared_ptr<ServerGoal>> goals_;
  rclcpp::Service<SaveMap>::SharedPtr saver_;
  std::vector<HeldResponse> held_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poses_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr transforms_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  std::vector<std::string> saved_topics_;
  double time_{100.0};
  int marker_updates_{0};
  int goal_requests_{0}, cancel_requests_{0};
  bool accept_goals_{true}, accept_cancels_{true};
  bool hold_saves_{false}, fail_saves_{false};
};

TEST_F(AutoMapperNode, StartsWithoutNavigationAndServicesRemainResponsive)
{
  const auto before = std::chrono::steady_clock::now();
  make_node();
  EXPECT_LT(std::chrono::steady_clock::now() - before, 1s);
  EXPECT_TRUE(set_enabled(false));
  EXPECT_TRUE(set_enabled(true));
  make_navigation();
  maps_->publish(scene());
  publish_pose();
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 1;}));
}

TEST_F(AutoMapperNode, RetainsLatchedMapBeforeFirstPoseAndReceivesBestEffortPose)
{
  maps_->publish(scene());  // Published before the map subscriber exists.
  make_navigation();
  make_node();
  spin_for(0.15);
  EXPECT_TRUE(goals_.empty());
  publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  EXPECT_EQ(goals_[0]->get_goal()->pose.header.frame_id, "map");
  EXPECT_GT(goals_[0]->get_goal()->pose.pose.position.x, 2.5);
}

TEST_F(AutoMapperNode, ReenableAndRejectedGoalRetryDoNotRequireAnotherMap)
{
  make_navigation();
  make_node({rclcpp::Parameter("start_enabled", false)});
  maps_->publish(scene()); publish_pose();
  spin_for(0.15);
  accept_goals_ = false;
  ASSERT_TRUE(set_enabled(true));
  ASSERT_TRUE(wait_for([&] {return goal_requests_ == 1;}));
  spin_for(0.05);
  accept_goals_ = true;
  advance(0.3);
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  EXPECT_EQ(goal_requests_, 2);
}

TEST_F(AutoMapperNode, ConfigurableMapQosAcceptsVolatileBestEffortStream)
{
  maps_ = helper_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).best_effort());
  make_navigation();
  make_node({rclcpp::Parameter("map_durability", "volatile"),
      rclcpp::Parameter("map_reliability", "best_effort")});
  maps_->publish(scene()); publish_pose();
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 1;}));
}

TEST_F(AutoMapperNode, DisableDoesNotCancelAnotherClientsGoal)
{
  make_navigation(); make_node();
  auto peer = rclcpp_action::create_client<Navigate>(helper_, "navigate_to_pose");
  ASSERT_TRUE(peer->wait_for_action_server(1s));
  auto future = peer->async_send_goal(Navigate::Goal{});
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  ASSERT_TRUE(set_enabled(false));
  spin_for(0.2);
  EXPECT_TRUE(goals_[0]->is_executing());
  EXPECT_EQ(cancel_requests_, 0);
}

TEST_F(AutoMapperNode, DisableDuringSendCancelsLateAcceptedOwnedGoal)
{
  make_navigation(); make_node();
  executor_.remove_node(nav_node_);  // Delay processing of the goal request.
  maps_->publish(scene()); publish_pose();
  const bool sent = wait_for([&] {return marker_updates_ > 0;});
  const bool disabled = set_enabled(false);
  executor_.add_node(nav_node_);
  ASSERT_TRUE(sent);
  ASSERT_TRUE(disabled);
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1 && goals_[0]->is_canceling();}));
  EXPECT_EQ(cancel_requests_, 1);
  goals_[0]->canceled(std::make_shared<Navigate::Result>());
  advance(1.0);
  spin_for(0.2);
  EXPECT_EQ(goals_.size(), 1u);
}

TEST_F(AutoMapperNode, LateResultFromRetiredGoalCannotReplaceNewGoal)
{
  make_navigation(); make_node({rclcpp::Parameter("goal_timeout_sec", 0.2)});
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  auto first = goals_[0];
  advance(0.3);
  ASSERT_TRUE(wait_for([&] {return first->is_canceling();}));
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("goal_timeout_sec", 300.0)).successful);
  // Announce a terminal status but delay the result service response. The
  // client can then release that pending request after its grace period.
  auto statuses = nav_node_->create_publisher<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status", rclcpp::QoS(1).transient_local());
  action_msgs::msg::GoalStatusArray message;
  action_msgs::msg::GoalStatus status;
  status.goal_info.goal_id.uuid = first->get_goal_id();
  status.status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
  message.status_list.push_back(status);
  statuses->publish(message);
  spin_for(0.25);
  advance(0.3);
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 2;}));
  first->canceled(std::make_shared<Navigate::Result>());
  spin_for(0.2);
  EXPECT_EQ(goals_.size(), 2u);
  EXPECT_TRUE(goals_[1]->is_executing());
}

TEST_F(AutoMapperNode, RejectedCancellationDoesNotDispatchReplacement)
{
  make_navigation(); make_node({rclcpp::Parameter("goal_timeout_sec", 0.2)});
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  accept_cancels_ = false;
  advance(0.3);
  ASSERT_TRUE(wait_for([&] {return cancel_requests_ >= 2;}));
  advance(0.4);
  EXPECT_EQ(goals_.size(), 1u);
  EXPECT_TRUE(goals_[0]->is_executing());
  accept_cancels_ = true;
  EXPECT_TRUE(wait_for([&] {return goals_[0]->is_canceling();}));
}

TEST_F(AutoMapperNode, AcceptedCancellationWaitsForTerminalStateBeforeReplacement)
{
  make_navigation(); make_node({rclcpp::Parameter("goal_timeout_sec", 0.2)});
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  advance(0.3);
  ASSERT_TRUE(wait_for([&] {return goals_[0]->is_canceling();}));
  spin_for(0.3);
  advance(0.3);
  EXPECT_EQ(goals_.size(), 1u);
  goals_[0]->canceled(std::make_shared<Navigate::Result>());
  spin_for(0.05);
  advance(0.3);
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 2;}));
}

TEST_F(AutoMapperNode, PausedSimulationDoesNotConsumeMissionTimeout)
{
  make_navigation(); make_node({rclcpp::Parameter("goal_timeout_sec", 0.2)});
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  spin_for(0.4);
  EXPECT_EQ(cancel_requests_, 0);
  advance(0.3);
  EXPECT_TRUE(wait_for([&] {return goals_[0]->is_canceling();}));
}

TEST_F(AutoMapperNode, ReapplyingSameClockSourceDoesNotResetMission)
{
  make_navigation(); make_node();
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  spin_for(0.2);
  EXPECT_EQ(cancel_requests_, 0);
  EXPECT_TRUE(goals_[0]->is_executing());
}

TEST_F(AutoMapperNode, BackwardClockJumpCancelsOldGoalAndRequiresFreshInputs)
{
  make_navigation(); make_node();
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  advance(-10.0, false);
  ASSERT_TRUE(wait_for([&] {return goals_[0]->is_canceling();}));
  goals_[0]->canceled(std::make_shared<Navigate::Result>());
  advance(0.3, false);
  spin_for(0.2);
  EXPECT_EQ(goals_.size(), 1u);
  maps_->publish(scene()); publish_pose();
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 2;}));
}

TEST_F(AutoMapperNode, StalePoseAndMissingTransformPreventDispatch)
{
  make_navigation(); make_node();
  maps_->publish(scene());
  publish_pose(2.5, 10.5, "odom");
  spin_for(0.2);
  EXPECT_TRUE(goals_.empty());
  advance(2.0, false);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.rotation.w = 1;
  tf2_msgs::msg::TFMessage message;
  message.transforms.push_back(transform);
  transforms_->publish(message);
  spin_for(0.2);
  EXPECT_TRUE(goals_.empty());
  publish_pose(2.5, 10.5, "odom");
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 1;}));
}

TEST_F(AutoMapperNode, TransformsPrimaryOdometryBeforeUsingAlternativePose)
{
  make_navigation();
  make_node({rclcpp::Parameter("odom_topic", "odom")});
  maps_->publish(scene());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 100;
  transform.transform.rotation.w = 1;
  tf2_msgs::msg::TFMessage message;
  message.transforms.push_back(transform);
  transforms_->publish(message);
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "odom";
  odometry.header.stamp = stamp();
  odometry.pose.pose.position.x = -97.5;
  odometry.pose.pose.position.y = 10.5;
  odometry.pose.pose.orientation.w = 1;
  odometry_->publish(odometry);
  publish_pose(-1000, -1000);  // Fresh alternate must not overwrite the primary.
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  EXPECT_EQ(goals_[0]->get_goal()->pose.header.frame_id, "map");
  EXPECT_LT(goals_[0]->get_goal()->pose.pose.position.x, 10.0);
}

TEST_F(AutoMapperNode, RuntimeTuningAppliesAtomicallyAndStaticParametersAreReadOnly)
{
  make_navigation(); make_node({rclcpp::Parameter("min_free_threshold", 8)});
  maps_->publish(scene()); publish_pose();
  spin_for(0.2);
  EXPECT_TRUE(goals_.empty());
  auto invalid = node_->set_parameters_atomically({
      rclcpp::Parameter("min_free_threshold", 2),
      rclcpp::Parameter("min_distance_to_frontier_m", 100.0)});
  EXPECT_FALSE(invalid.successful);
  EXPECT_EQ(node_->get_parameter("min_free_threshold").as_int(), 8);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("map_topic", "other")).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("goal_timeout_sec",
    std::numeric_limits<double>::quiet_NaN())).successful);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("min_free_threshold", 2)).successful);
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 1;}));
}

TEST_F(AutoMapperNode, MalformedMapCannotStartNavigation)
{
  make_navigation(); make_node();
  auto map = scene();
  map.data.resize(1);
  maps_->publish(map); publish_pose();
  spin_for(0.2);
  EXPECT_TRUE(goals_.empty());
  map.info.width = map.info.height = 65536;
  map.data.clear();
  maps_->publish(map);
  spin_for(0.15);
  EXPECT_TRUE(goals_.empty());
  maps_->publish(scene());
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 1;}));
}

TEST_F(AutoMapperNode, InvalidPoseFieldsAreRejectedWithoutThrowing)
{
  make_navigation(); make_node();
  maps_->publish(scene());
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1;
  pose.header.stamp.sec = -1;
  poses_->publish(pose);
  EXPECT_NO_THROW(spin_for(0.1));
  pose.header.stamp = stamp();
  pose.header.stamp.nanosec = 1000000000u;
  poses_->publish(pose);
  EXPECT_NO_THROW(spin_for(0.1));
  pose.header.stamp = stamp();
  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  poses_->publish(pose);
  spin_for(0.1);
  EXPECT_TRUE(goals_.empty());
  publish_pose();
  EXPECT_TRUE(wait_for([&] {return goals_.size() == 1;}));
}

TEST_F(AutoMapperNode, OutOfRangeStartupIntegerCannotWrapIntoValidTuning)
{
  // Exercise early construction failure and cleanup repeatedly with sim time.
  for (int attempt = 0; attempt < 20; ++attempt) {
    EXPECT_THROW(make_node({rclcpp::Parameter("min_free_threshold",
      static_cast<int64_t>(4294967298LL))}), std::invalid_argument);
  }
}

TEST_F(AutoMapperNode, SaveRetriesUnavailableServiceAndUsesResolvedMapTopic)
{
  maps_ = helper_->create_publisher<nav_msgs::msg::OccupancyGrid>("remapped_map",
      rclcpp::QoS(1).transient_local());
  make_navigation(); make_node({}, {"map:=remapped_map"});
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  goals_[0]->succeed(std::make_shared<Navigate::Result>());
  maps_->publish(scene(false));
  advance(0.3);
  spin_for(0.2);
  make_saver();
  ASSERT_TRUE(wait_for([&] {return saved_topics_.size() == 1;}));
  EXPECT_EQ(saved_topics_[0], ns_ + "/remapped_map");
  spin_for(0.3);
  EXPECT_EQ(saved_topics_.size(), 1u);
}

TEST_F(AutoMapperNode, SaveFailureAndTimeoutRetryWhileLateResponseCannotClearNewRequest)
{
  make_navigation(); make_node();
  make_saver(); fail_saves_ = true;
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  goals_[0]->succeed(std::make_shared<Navigate::Result>());
  ASSERT_TRUE(wait_for([&] {return saved_topics_.size() >= 2;}));
  fail_saves_ = false; hold_saves_ = true;
  ASSERT_TRUE(wait_for([&] {return held_.size() == 2;}));
  SaveMap::Response response;
  response.result = true;
  held_[0].service->send_response(*held_[0].header, response);  // Already timed out.
  spin_for(0.04);
  const auto count = saved_topics_.size();
  hold_saves_ = false;
  ASSERT_TRUE(wait_for([&] {return saved_topics_.size() > count;}));
  const auto completed = saved_topics_.size();
  spin_for(0.3);
  EXPECT_EQ(saved_topics_.size(), completed);
}

TEST_F(AutoMapperNode, EarlierExhaustionSaveCannotAcknowledgeLaterExhaustion)
{
  make_navigation(); make_node({rclcpp::Parameter("save_timeout_sec", 5.0)});
  make_saver(); hold_saves_ = true;
  maps_->publish(scene()); publish_pose();
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 1;}));
  goals_[0]->abort(std::make_shared<Navigate::Result>());
  maps_->publish(scene(false));
  spin_for(0.06);
  advance(0.3);
  ASSERT_TRUE(wait_for([&] {return held_.size() == 1;}));

  maps_->publish(scene());
  ASSERT_TRUE(wait_for([&] {return goals_.size() == 2;}));
  goals_[1]->abort(std::make_shared<Navigate::Result>());
  maps_->publish(scene(false));
  spin_for(0.06);
  advance(0.3);
  spin_for(0.2);  // Allow the second exhaustion search to complete.
  EXPECT_EQ(saved_topics_.size(), 1u);

  SaveMap::Response response;
  response.result = true;
  hold_saves_ = false;
  held_[0].service->send_response(*held_[0].header, response);
  ASSERT_TRUE(wait_for([&] {return saved_topics_.size() == 2;}));
  spin_for(0.3);
  EXPECT_EQ(saved_topics_.size(), 2u);
}

TEST_F(AutoMapperNode, DisableServiceRemainsResponsiveDuringLargeMapSearch)
{
  make_navigation(); make_node();
  auto map = scene(false);
  map.info.width = map.info.height = 2048;
  map.info.resolution = 0.05;
  map.data.resize(2048 * 2048, 0);
  maps_->publish(map); publish_pose();
  spin_for(0.06);
  const auto begin = std::chrono::steady_clock::now();
  EXPECT_TRUE(set_enabled(false));
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 500ms);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int status = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return status;
}
