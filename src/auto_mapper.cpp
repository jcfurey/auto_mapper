//
// Edited by gglapell on 7/10/25.
//

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <array>
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::OccupancyGrid;
using nav2_msgs::action::NavigateToPose;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using std::chrono::steady_clock;
using namespace std::chrono_literals;

using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class AutoMapper : public rclcpp::Node {
public:
    AutoMapper()
            : rclcpp::Node("auto_mapper") {
        RCLCPP_INFO(get_logger(), "AutoMapper started...");

        // Declare and read parameters in one pass — declare_parameter<T>(name, default)
        // returns the resolved value, so we don't need a separate get_parameter call
        // (and we don't need redundant field initializers either).
        mapTopic_                    = declare_parameter<std::string>("map_topic",   "/map");
        odomTopic_                   = declare_parameter<std::string>("odom_topic",  "/localization/odometry/odom");
        poseTopic_                   = declare_parameter<std::string>("pose_topic",  "");   // optional PoseStamped alternative
        mapPath_                     = declare_parameter<std::string>("map_path",    "/tmp/maps");
        min_frontier_length_m_       = declare_parameter<double>("min_frontier_length_m", 0.25);
        min_distance_to_frontier_m_  = declare_parameter<double>("min_distance_to_frontier_m", 0.75);
        max_distance_to_frontier_m_  = declare_parameter<double>("max_distance_to_frontier_m", 40.0);
        frontier_size_weight_        = declare_parameter<double>("frontier_size_weight", 1.0);
        frontier_distance_weight_    = declare_parameter<double>("frontier_distance_weight", 0.35);
        frontier_distance_cap_m_     = declare_parameter<double>("frontier_distance_cap_m", 20.0);
        min_free_threshold_          = declare_parameter<int>("min_free_threshold", 4);
        goal_clearance_radius_m_     = declare_parameter<double>("goal_clearance_radius_m", 1.5);
        forward_weight_              = declare_parameter<double>("forward_weight", 2.0);
        blacklist_radius_m_          = declare_parameter<double>("blacklist_radius_m", 1.0);
        blacklist_duration_sec_      = declare_parameter<double>("blacklist_duration_sec", 60.0);

        const double startup_delay_sec = declare_parameter<double>("startup_delay_sec", 0.0);
        if (startup_delay_sec > 0.0) {
            next_explore_time_ = steady_clock::now() +
                std::chrono::duration_cast<steady_clock::duration>(
                    std::chrono::duration<double>(startup_delay_sec));
            RCLCPP_INFO(get_logger(), "Startup delay: %.1f seconds before first exploration.", startup_delay_sec);
        }

        // Subscribe to Odometry if odom_topic is non-empty (primary source)
        if (!odomTopic_.empty()) {
            odomSubscription_ = create_subscription<Odometry>(
                    odomTopic_, 10, std::bind(&AutoMapper::odomCallback, this, _1));
            RCLCPP_INFO(get_logger(), "Subscribing to Odometry on '%s'.", odomTopic_.c_str());
        }
        // Subscribe to PoseStamped if pose_topic is non-empty (alternative source)
        if (!poseTopic_.empty()) {
            poseSubscription_ = create_subscription<PoseStamped>(
                    poseTopic_, 10, std::bind(&AutoMapper::poseCallback, this, _1));
            RCLCPP_INFO(get_logger(), "Subscribing to PoseStamped on '%s'.", poseTopic_.c_str());
        }
        if (odomTopic_.empty() && poseTopic_.empty()) {
            RCLCPP_ERROR(get_logger(), "Neither odom_topic nor pose_topic is set — no pose source!");
        }

        mapSubscription_ = create_subscription<OccupancyGrid>(
                mapTopic_, 10, std::bind(&AutoMapper::updateFullMap, this, _1));

        markerArrayPublisher_ = create_publisher<MarkerArray>("/frontiers", 10);
        poseNavigator_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");
        // Create the map_saver client once and reuse. The previous code
        // allocated a fresh client (and blocked the executor for up to 1 s in
        // wait_for_service) on every result_callback — i.e. every Nav2 goal
        // completion. Clients destroyed mid-flight also silently drop their
        // responses.
        mapSaverClient_ = create_client<nav2_msgs::srv::SaveMap>("/map_server/save_map");

        enabled_ = declare_parameter<bool>("start_enabled", true);
        RCLCPP_INFO(get_logger(), "Exploration %s at startup.",
            enabled_ ? "enabled" : "disabled");

        setEnabledService_ = create_service<std_srvs::srv::SetBool>(
            "~/set_enabled",
            std::bind(&AutoMapper::setEnabledCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        // Wait for the navigate_to_pose action server, but yield to the executor
        // so the node can be interrupted (e.g. Ctrl-C) while waiting.
        while (!poseNavigator_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for navigate_to_pose action server.");
                return;
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for navigate_to_pose action server...");
        }
        RCLCPP_INFO(get_logger(), "AutoMapper connected to navigate_to_pose action server.");
    }

private:
    // Tunables — actual default values live at the declare_parameter<>() call
    // site in the constructor; these fields are written there before any use.
    double min_frontier_length_m_;
    double min_distance_to_frontier_m_;
    double max_distance_to_frontier_m_;
    double frontier_size_weight_;
    double frontier_distance_weight_;
    double frontier_distance_cap_m_;
    int    min_free_threshold_;
    double goal_clearance_radius_m_;
    double forward_weight_;
    Costmap2D costmap_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_;
    rclcpp::Publisher<MarkerArray>::SharedPtr markerArrayPublisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setEnabledService_;
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr mapSaverClient_;
    bool mapSaverAvailable_ = false;  // latched true after first successful service discovery
    MarkerArray markersMsg_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    bool isExploring_ = false;
    bool enabled_ = true;     // runtime soft-toggle via ~/set_enabled
    bool exhaustionMapSaved_ = false;  // throttle saveMap() to once per "no frontiers left" episode
    steady_clock::time_point next_explore_time_ = steady_clock::now();  // backoff after rejection
    int markerId_ = 0;

    // Blacklist: rejected goal locations are remembered so the same frontier
    // centroid is not re-selected on the next map update.  Each entry stores
    // the world-frame position and the time it was blacklisted.  Entries expire
    // after blacklist_duration_sec_ so that previously inaccessible areas can
    // be retried once the map has changed significantly.
    struct BlacklistEntry { double x; double y; steady_clock::time_point when; };
    std::vector<BlacklistEntry> blacklist_;
    double blacklist_radius_m_;       // ROS param — reject frontiers within this radius of a blacklisted goal
    double blacklist_duration_sec_;   // ROS param — entries expire after this many seconds
    std::string mapPath_;
    std::string mapTopic_;
    std::string odomTopic_;
    std::string poseTopic_;

    rclcpp::Subscription<Odometry>::SharedPtr   odomSubscription_;   // nav_msgs/Odometry (odom_topic)
    rclcpp::Subscription<PoseStamped>::SharedPtr poseSubscription_;  // geometry_msgs/PoseStamped (pose_topic)
    PoseWithCovarianceStamped::UniquePtr pose_;
    bool hasNavigated_ = false;  // true once at least one goal has been accepted
    std::string mapFrameId_ = "map";  // frame_id of the most recent OccupancyGrid; used for marker headers

    std::array<unsigned char, 256> costTranslationTable_ = initTranslationTable();

    static std::array<unsigned char, 256> initTranslationTable() {
        std::array<unsigned char, 256> cost_translation_table{};

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                    static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = FREE_SPACE;
        cost_translation_table[99] = 253;
        cost_translation_table[100] = LETHAL_OBSTACLE;
        cost_translation_table[static_cast<unsigned char>(-1)] = NO_INFORMATION;

        return cost_translation_table;
    }

    struct Frontier {
        Point centroid;
        std::vector<Point> points;
        std::string getKey() const { return std::to_string(centroid.x) + "," + std::to_string(centroid.y); }
    };

    double frontierDistance(const Frontier & frontier, const Point & position) const {
        const double dx = frontier.centroid.x - position.x;
        const double dy = frontier.centroid.y - position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool isBlacklisted(const Point & p) const {
        for (const auto & entry : blacklist_) {
            double dx = p.x - entry.x;
            double dy = p.y - entry.y;
            if (dx * dx + dy * dy < blacklist_radius_m_ * blacklist_radius_m_) {
                return true;
            }
        }
        return false;
    }

    void pruneBlacklist() {
        const auto now = steady_clock::now();
        const auto duration = std::chrono::duration_cast<steady_clock::duration>(
            std::chrono::duration<double>(blacklist_duration_sec_));
        blacklist_.erase(
            std::remove_if(blacklist_.begin(), blacklist_.end(),
                [now, duration](const BlacklistEntry & e) { return (now - e.when) > duration; }),
            blacklist_.end());
    }

    double robotYaw() const {
        if (!pose_) return 0.0;
        const auto & q = pose_->pose.pose.orientation;
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double scoreFrontier(const Frontier & frontier, const Point & position) const {
        const double distance = frontierDistance(frontier, position);
        const double frontier_length_m = frontier.points.size() * costmap_.getResolution();
        const double clamped_distance = std::min(distance, frontier_distance_cap_m_);

        // Forward bias: dot product of heading vs. robot→frontier direction.
        // Produces depth-first behavior in tunnels — continue forward before
        // turning around to explore side branches.
        double forward_bonus = 0.0;
        if (forward_weight_ > 0.0 && distance > 0.5) {
            double dx = frontier.centroid.x - position.x;
            double dy = frontier.centroid.y - position.y;
            double norm = std::sqrt(dx * dx + dy * dy);
            double yaw = robotYaw();
            // dot ∈ [-1, 1]: +1 = directly ahead, -1 = directly behind
            double dot = (dx * std::cos(yaw) + dy * std::sin(yaw)) / norm;
            // Map to [0, 1] so behind = 0, ahead = 1, side = 0.5
            double forward_factor = (dot + 1.0) / 2.0;
            forward_bonus = forward_weight_ * forward_factor * clamped_distance;
        }

        return frontier_size_weight_ * frontier_length_m +
               frontier_distance_weight_ * clamped_distance +
               forward_bonus;
    }

    // Called for nav_msgs/Odometry messages (odom_topic).
    void odomCallback(Odometry::UniquePtr msg) {
        if (pose_ == nullptr) {
            RCLCPP_INFO(get_logger(), "Initial robot pose received on odom_topic '%s'.", odomTopic_.c_str());
        }
        pose_ = std::make_unique<PoseWithCovarianceStamped>();
        pose_->header = msg->header;
        pose_->pose   = msg->pose;  // PoseWithCovariance — includes covariance
    }

    // Called for geometry_msgs/PoseStamped messages (pose_topic).
    void poseCallback(PoseStamped::UniquePtr msg) {
        if (pose_ == nullptr) {
            RCLCPP_INFO(get_logger(), "Initial robot pose received on pose_topic '%s'.", poseTopic_.c_str());
        }
        pose_ = std::make_unique<PoseWithCovarianceStamped>();
        pose_->header      = msg->header;
        pose_->pose.pose   = msg->pose;
        // Covariance is unavailable from PoseStamped; remains zero-initialized.
    }

    void updateFullMap(OccupancyGrid::UniquePtr occupancyGrid) {
        if (pose_ == nullptr) {
            // Whichever pose source is configured (may be both); print the
            // non-empty one so operators can see what we're actually waiting
            // on. Falling back to "<unset>" makes the misconfiguration loud.
            const std::string pose_source = !odomTopic_.empty() ? odomTopic_
                                          : !poseTopic_.empty() ? poseTopic_
                                          : std::string("<unset>");
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000, // Throttle to every 5 seconds
                "Map received on topic '%s', but waiting for initial pose on topic '%s' to begin exploring.",
                mapTopic_.c_str(), pose_source.c_str());
            return;
        }
        RCLCPP_DEBUG(get_logger(), "updateFullMap...");
        mapFrameId_ = occupancyGrid->header.frame_id;
        const auto occupancyGridInfo = occupancyGrid->info;
        unsigned int size_in_cells_x = occupancyGridInfo.width;
        unsigned int size_in_cells_y = occupancyGridInfo.height;
        double resolution = occupancyGridInfo.resolution;
        double origin_x = occupancyGridInfo.origin.position.x;
        double origin_y = occupancyGridInfo.origin.position.y;

        RCLCPP_DEBUG(get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x,
                    size_in_cells_y);
        costmap_.resizeMap(size_in_cells_x,
                           size_in_cells_y,
                           resolution,
                           origin_x,
                           origin_y);

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        std::lock_guard<Costmap2D::mutex_t> lock(*mutex);

        // fill map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_DEBUG(get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) {
            auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
            costmap_data[i] = costTranslationTable_[cell_cost];
        }

        explore();
    }

    /// Maximum costmap cost we'll accept for a goal cell. Both refinement
    /// (refineGoalClearance) and validation (in explore()) check against
    /// this threshold so a borderline cell can't pass one and fail the other.
    /// Set strictly below INSCRIBED_INFLATED_OBSTACLE (253), so cells in
    /// [253, 254] are always rejected.
    static constexpr unsigned char MAX_ACCEPTABLE_COST_ = 252;

    /// Shift a goal point toward the lowest-cost cell within a search radius.
    /// In tunnels this pulls centroids away from walls toward the corridor center.
    /// Strongly prefers FREE_SPACE cells; within a cost tier, picks the cell
    /// closest to the original centroid.
    Point refineGoalClearance(const Point & centroid) const {
        if (goal_clearance_radius_m_ <= 0.0) return centroid;

        unsigned int cx, cy;
        if (!costmap_.worldToMap(centroid.x, centroid.y, cx, cy)) {
            return centroid;
        }

        int radius_cells = static_cast<int>(
            goal_clearance_radius_m_ / costmap_.getResolution());
        int sx = static_cast<int>(costmap_.getSizeInCellsX());
        int sy = static_cast<int>(costmap_.getSizeInCellsY());

        unsigned char best_cost = 255;  // start worse than anything
        unsigned int best_mx = cx, best_my = cy;
        double best_dist2 = std::numeric_limits<double>::max();

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                int nx = static_cast<int>(cx) + dx;
                int ny = static_cast<int>(cy) + dy;
                if (nx < 0 || nx >= sx || ny < 0 || ny >= sy) continue;

                unsigned char cost = costmap_.getCost(nx, ny);
                // Skip cells that are too dangerous
                if (cost > MAX_ACCEPTABLE_COST_) continue;
                // Skip unknown cells — we want goals in observed free space
                if (cost == NO_INFORMATION) continue;

                double dist2 = dx * dx + dy * dy;
                if (cost < best_cost ||
                    (cost == best_cost && dist2 < best_dist2)) {
                    best_cost = cost;
                    best_mx = static_cast<unsigned int>(nx);
                    best_my = static_cast<unsigned int>(ny);
                    best_dist2 = dist2;
                }
            }
        }

        // If no acceptable cell was found, return original centroid
        // (explore() will catch it in the validation step)
        if (best_cost > MAX_ACCEPTABLE_COST_) return centroid;

        Point refined;
        double wx, wy;
        costmap_.mapToWorld(best_mx, best_my, wx, wy);
        refined.x = wx;
        refined.y = wy;
        refined.z = 0.0;
        return refined;
    }

    void drawMarkers(const std::vector<Frontier> &frontiers) {
        // Send a DELETEALL first so RViz/Trillium drops markers from the previous
        // frame before we publish the current ones. Without this the local
        // markersMsg_ would either grow unbounded across calls (visual stale
        // cruft) or, if cleared, leave RViz holding orphans from earlier ADDs.
        markersMsg_.markers.clear();
        Marker delete_all;
        delete_all.action = Marker::DELETEALL;
        delete_all.ns = "frontiers";
        markersMsg_.markers.push_back(delete_all);

        ColorRGBA green;
        green.r = 0.0;
        green.g = 1.0;
        green.b = 0.0;
        green.a = 1.0;

        const auto stamp = now();
        for (const auto &frontier: frontiers) {
            RCLCPP_DEBUG(get_logger(), "visualising %f,%f ", frontier.centroid.x, frontier.centroid.y);
            Marker m;
            m.header.frame_id = mapFrameId_;
            m.header.stamp = stamp;
            m.frame_locked = true;
            m.action = Marker::ADD;
            m.ns = "frontiers";
            m.id = ++markerId_;
            m.type = Marker::SPHERE;
            m.pose.position = frontier.centroid;
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            m.color = green;
            markersMsg_.markers.push_back(m);
        }
        markerArrayPublisher_->publish(markersMsg_);
    }

    void clearMarkers() {
        markersMsg_.markers.clear();
        Marker delete_all;
        delete_all.action = Marker::DELETEALL;
        delete_all.ns = "frontiers";
        markersMsg_.markers.push_back(delete_all);
        markerArrayPublisher_->publish(markersMsg_);
    }

    void setEnabledCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        const bool prev = enabled_;
        enabled_ = request->data;
        if (enabled_ == prev) {
            response->success = true;
            response->message = enabled_ ? "already enabled" : "already disabled";
            return;
        }
        if (!enabled_) {
            RCLCPP_INFO(get_logger(), "Exploration disabled via service; cancelling any active goal.");
            poseNavigator_->async_cancel_all_goals();
            // isExploring_ will be cleared by the CANCELED result_callback.
            clearMarkers();
            response->message = "exploration disabled";
        } else {
            RCLCPP_INFO(get_logger(), "Exploration enabled via service.");
            next_explore_time_ = steady_clock::now();
            response->message = "exploration enabled";
            // explore() will be triggered by the next OccupancyGrid callback.
        }
        response->success = true;
    }

    void explore() {
        if (isExploring_ || !enabled_) { return; }
        if (steady_clock::now() < next_explore_time_) {
            if (!hasNavigated_) {
                auto remaining = std::chrono::duration_cast<std::chrono::seconds>(
                    next_explore_time_ - steady_clock::now()).count();
                if (remaining > 0) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                        "Exploration starts in %ld seconds...", remaining);
                }
            }
            return;
        }

        pruneBlacklist();

        auto frontiers = findFrontiers();

        // Remove blacklisted frontiers before scoring.
        frontiers.erase(
            std::remove_if(frontiers.begin(), frontiers.end(),
                [this](const Frontier & f) { return isBlacklisted(f.centroid); }),
            frontiers.end());

        if (frontiers.empty()) {
            if (!hasNavigated_) {
                // Map too sparse to find frontiers yet — wait for more scans.
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "No frontiers found yet — waiting for map to populate...");
                return;
            }
            if (!blacklist_.empty()) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                    "All frontiers blacklisted (%zu entries) — waiting for blacklist expiry or new map data...",
                    blacklist_.size());
                return;
            }
            // No frontiers remain. Don't tear the node down — the map can grow
            // when the robot rounds a corner an hour later, or when an operator
            // pushes the rover into a new area; we want to resume automatically.
            // Save the map once per exhaustion episode so we don't spam the
            // map_saver with every map update while idle.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                "No frontiers remaining — exploration appears complete. "
                "Will resume if new frontiers appear in future map updates.");
            if (!exhaustionMapSaved_) {
                saveMap();
                exhaustionMapSaved_ = true;
            }
            return;
        }
        // We have frontiers again — re-arm the one-shot save.
        exhaustionMapSaved_ = false;
        const auto robot_position = pose_->pose.pose.position;
        const auto frontier_it = std::max_element(
            frontiers.begin(), frontiers.end(),
            [this, &robot_position](const Frontier & a, const Frontier & b) {
                return scoreFrontier(a, robot_position) < scoreFrontier(b, robot_position);
            });
        const auto frontier = *frontier_it;

        RCLCPP_INFO(get_logger(),
            "Selected frontier %.2f, %.2f (size=%.2fm, dist=%.2fm, score=%.2f)",
            frontier.centroid.x,
            frontier.centroid.y,
            frontier.points.size() * costmap_.getResolution(),
            frontierDistance(frontier, robot_position),
            scoreFrontier(frontier, robot_position));

        // Refine the goal position: search near the centroid for the cell with
        // the lowest costmap cost.  In tunnels this pulls goals away from walls
        // toward the corridor center.
        Point goal_point = refineGoalClearance(frontier.centroid);

        // Validate that the refined goal is in a traversable cell. Use the same
        // MAX_ACCEPTABLE_COST_ threshold as refineGoalClearance — if the two
        // disagree a borderline cell can pass refinement and fail validation
        // (or vice versa) on the same goal.
        {
            unsigned int goal_mx, goal_my;
            if (costmap_.worldToMap(goal_point.x, goal_point.y, goal_mx, goal_my)) {
                auto cost = costmap_.getCost(goal_mx, goal_my);
                if (cost > MAX_ACCEPTABLE_COST_) {
                    RCLCPP_WARN(get_logger(),
                        "Goal (%.2f, %.2f) is inside obstacle (cost=%d) — blacklisting",
                        goal_point.x, goal_point.y, cost);
                    blacklist_.push_back({goal_point.x, goal_point.y, steady_clock::now()});
                    next_explore_time_ = steady_clock::now() + 1s;
                    return;
                }
            }
        }

        drawMarkers(frontiers);
        auto goal = NavigateToPose::Goal();
        goal.pose.pose.position = goal_point;
        goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = "map";

        RCLCPP_INFO(get_logger(), "Sending goal %.2f,%.2f (centroid was %.2f,%.2f)",
            goal_point.x, goal_point.y, frontier.centroid.x, frontier.centroid.y);

        // Set exploring flag synchronously BEFORE async_send_goal to prevent
        // updateFullMap() from calling explore() again before the response arrives.
        isExploring_ = true;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
            if (goal_handle) {
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
                hasNavigated_ = true;
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Goal rejected by server (not yet active?) — retrying in 5s");
                isExploring_ = false;
                next_explore_time_ = steady_clock::now() + 5s;
            }
        };

        send_goal_options.feedback_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &,
                const std::shared_ptr<const NavigateToPose::Feedback> &feedback) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Distance remaining: %.2f m", feedback->distance_remaining);
        };

        // Capture goal position for blacklisting on abort
        auto goal_x = goal_point.x;
        auto goal_y = goal_point.y;
        send_goal_options.result_callback = [this, goal_x, goal_y](const GoalHandleNavigateToPose::WrappedResult &result) {
            isExploring_ = false;
            saveMap();
            clearMarkers();
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Goal reached");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(get_logger(), "Goal (%.2f, %.2f) aborted — blacklisting", goal_x, goal_y);
                    blacklist_.push_back({goal_x, goal_y, steady_clock::now()});
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code");
                    break;
            }
            explore();
        };
        poseNavigator_->async_send_goal(goal, send_goal_options);
    }

    void saveMap() {
        // Non-blocking availability check. service_is_ready() is a polled view
        // of the discovered server; once it goes true we latch it so we never
        // pay the discovery cost again. We never call wait_for_service here:
        // this runs from the result_callback on the single-threaded executor,
        // and a 1 s block would queue up subsequent goal-completion callbacks.
        if (!mapSaverAvailable_) {
            if (!mapSaverClient_->service_is_ready()) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                    "map_saver service not yet available — skipping map save.");
                return;
            }
            mapSaverAvailable_ = true;
        }
        RCLCPP_INFO(get_logger(), "Saving map to %s ...", mapPath_.c_str());

        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->map_topic = mapTopic_;
        request->map_url = mapPath_;
        request->image_format = "pgm";
        request->map_mode = "trinary";

        mapSaverClient_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Save map request sent for map at %s", mapPath_.c_str());
    }

    std::vector<unsigned int> nhood8(unsigned int idx) {
        unsigned int mx, my;
        std::vector<unsigned int> out;
        costmap_.indexToCells(idx, mx, my);
        const int x = static_cast<int>(mx);
        const int y = static_cast<int>(my);
        const int sx = static_cast<int>(costmap_.getSizeInCellsX());
        const int sy = static_cast<int>(costmap_.getSizeInCellsY());
        const std::pair<int, int> directions[] = {
                {-1, -1}, {-1, 1}, {1, -1}, {1, 1},
                {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };
        for (const auto &d: directions) {
            int newX = x + d.first;
            int newY = y + d.second;
            if (newX >= 0 && newX < sx && newY >= 0 && newY < sy) {
                out.push_back(costmap_.getIndex(newX, newY));
            }
        }
        return out;
    }

    /// A cell is traversable if it is not lethal and not unknown — this includes
    /// FREE_SPACE (0) and inflated cells (1-252).  Using FREE_SPACE alone misses
    /// narrow passages where inflation zones from both walls overlap, leaving no
    /// cost-0 cells even though the robot can physically fit.
    static bool isTraversable(unsigned char cost) {
        return cost != NO_INFORMATION && cost != LETHAL_OBSTACLE;
    }

    bool isAchievableFrontierCell(unsigned int idx,
                                  const std::vector<bool> &frontier_flag) {
        auto map = costmap_.getCharMap();
        // check that cell is unknown and not already marked as frontier
        if (map[idx] != NO_INFORMATION || frontier_flag[idx]) {
            return false;
        }

        // check there's enough traversable space for robot to approach frontier
        int freeCount = 0;
        for (unsigned int nbr: nhood8(idx)) {
            if (isTraversable(map[nbr])) {
                if (++freeCount >= min_free_threshold_) {
                    return true;
                }
            }
        }

        return false;
    }

    Frontier buildNewFrontier(unsigned int neighborCell, std::vector<bool> &frontier_flag) {
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;

        std::queue<unsigned int> bfs;
        bfs.push(neighborCell);

        // Include the seed cell itself. The caller has already set
        // frontier_flag[neighborCell] = true, but never pushes the cell into
        // output.points — the BFS loop below only inspects *neighbors* of cells
        // in the queue. A single-cell frontier (no other achievable-frontier
        // neighbors) would otherwise leave output.points empty, and the
        // size-divide at the bottom of this function would NaN out the centroid.
        {
            unsigned int seed_mx, seed_my;
            double seed_wx, seed_wy;
            costmap_.indexToCells(neighborCell, seed_mx, seed_my);
            costmap_.mapToWorld(seed_mx, seed_my, seed_wx, seed_wy);
            Point seed_point;
            seed_point.x = seed_wx;
            seed_point.y = seed_wy;
            output.points.push_back(seed_point);
            output.centroid.x += seed_wx;
            output.centroid.y += seed_wy;
        }

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr: nhood8(idx)) {
                // check if neighbour is a potential frontier cell
                if (isAchievableFrontierCell(nbr, frontier_flag)) {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_.indexToCells(nbr, mx, my);
                    costmap_.mapToWorld(mx, my, wx, wy);

                    Point point;
                    point.x = wx;
                    point.y = wy;
                    output.points.push_back(point);

                    // update centroid of frontier
                    output.centroid.x += wx;
                    output.centroid.y += wy;

                    bfs.push(nbr);
                }
            }
        }

        // average out frontier centroid
        output.centroid.x /= output.points.size();
        output.centroid.y /= output.points.size();
        return output;
    }

    std::vector<Frontier> findFrontiers() {
        std::vector<Frontier> frontier_list;
        const auto position = pose_->pose.pose.position;
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
            RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        std::lock_guard<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        auto map_ = costmap_.getCharMap();
        auto size_x_ = costmap_.getSizeInCellsX();
        auto size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        std::vector<bool> frontier_flag(size_x_ * size_y_,
                                   false);
        std::vector<bool> visited_flag(size_x_ * size_y_,
                                  false);

        // initialize breadth first search
        std::queue<unsigned int> bfs;

        unsigned int pos = costmap_.getIndex(mx, my);

        // If the robot's cell is not traversable (common with VDB+patchworkpp since the
        // robot's immediate vicinity has no lidar rays), search outward for the nearest
        // traversable cell and seed the BFS from there instead.
        if (!isTraversable(map_[pos])) {
            std::queue<unsigned int> seed_bfs;
            std::vector<bool> seed_visited(size_x_ * size_y_, false);
            seed_bfs.push(pos);
            seed_visited[pos] = true;
            bool found_free = false;
            // Search up to 200×200 cells for the nearest traversable cell.
            const unsigned int MAX_SEED_SEARCH = 200 * 200;
            unsigned int seed_iters = 0;
            while (!seed_bfs.empty() && seed_iters < MAX_SEED_SEARCH) {
                unsigned int idx = seed_bfs.front();
                seed_bfs.pop();
                ++seed_iters;
                for (unsigned int nbr : nhood8(idx)) {
                    if (seed_visited[nbr]) continue;
                    seed_visited[nbr] = true;
                    if (isTraversable(map_[nbr])) {
                        pos = nbr;
                        found_free = true;
                        break;
                    }
                    if (map_[nbr] == NO_INFORMATION) {
                        seed_bfs.push(nbr);
                    }
                }
                if (found_free) break;
            }
            if (!found_free) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Robot vicinity is all unknown/lethal — no traversable cells reachable yet.");
                return frontier_list;
            }
        }

        bfs.push(pos);
        visited_flag[bfs.front()] = true;

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            for (unsigned nbr: nhood8(idx)) {
                // Expand through all traversable cells (free + inflated).
                // Using FREE_SPACE alone blocks the BFS at inflation boundaries,
                // making corridors narrower than 2×inflation_radius unreachable.
                if (isTraversable(map_[nbr]) && !visited_flag[nbr]) {
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                } else if (isAchievableFrontierCell(nbr, frontier_flag)) {
                    frontier_flag[nbr] = true;
                    const Frontier frontier = buildNewFrontier(nbr, frontier_flag);

                    const double distance = frontierDistance(frontier, position);
                    const double frontier_length_m = frontier.points.size() * costmap_.getResolution();
                    if (distance < min_distance_to_frontier_m_) { continue; }
                    if (distance > max_distance_to_frontier_m_) { continue; }
                    if (frontier_length_m < min_frontier_length_m_) { continue; }
                    frontier_list.push_back(frontier);
                }
            }
        }

        return frontier_list;
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoMapper>());
    rclcpp::shutdown();
    return 0;
}
