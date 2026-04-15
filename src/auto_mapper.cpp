//
// Edited by gglapell on 7/10/25.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <fstream>
#include <algorithm>

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
#include "std_msgs/std_msgs/msg/color_rgba.hpp"


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
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;
using namespace rclcpp;
using namespace rclcpp_action;
using std::chrono::steady_clock;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class AutoMapper : public Node {
public:
    AutoMapper()
            : Node("auto_mapper") {
        RCLCPP_INFO(get_logger(), "AutoMapper started...");

        // Declare and get parameters
        declare_parameter<string>("map_topic",   "/map");
        declare_parameter<string>("odom_topic",  "/localization/odometry/odom");
        declare_parameter<string>("pose_topic",  "");   // optional PoseStamped alternative
        declare_parameter<string>("map_path",    "/tmp/maps");
        declare_parameter<double>("min_frontier_length_m", 0.25);
        declare_parameter<double>("min_distance_to_frontier_m", 0.75);
        declare_parameter<double>("max_distance_to_frontier_m", 40.0);
        declare_parameter<double>("frontier_size_weight", 1.0);
        declare_parameter<double>("frontier_distance_weight", 0.35);
        declare_parameter<double>("frontier_distance_cap_m", 20.0);
        declare_parameter<int>("min_free_threshold", 4);
        get_parameter("map_topic",   mapTopic_);
        get_parameter("odom_topic",  odomTopic_);
        get_parameter("pose_topic",  poseTopic_);
        get_parameter("map_path",    mapPath_);
        get_parameter("min_frontier_length_m", min_frontier_length_m_);
        get_parameter("min_distance_to_frontier_m", min_distance_to_frontier_m_);
        get_parameter("max_distance_to_frontier_m", max_distance_to_frontier_m_);
        get_parameter("frontier_size_weight", frontier_size_weight_);
        get_parameter("frontier_distance_weight", frontier_distance_weight_);
        get_parameter("frontier_distance_cap_m", frontier_distance_cap_m_);
        get_parameter("min_free_threshold", min_free_threshold_);

        // Subscribe to Odometry if odom_topic is non-empty (primary source)
        if (!odomTopic_.empty()) {
            odomSubscription_ = create_subscription<Odometry>(
                    odomTopic_, 10, bind(&AutoMapper::odomCallback, this, _1));
            RCLCPP_INFO(get_logger(), "Subscribing to Odometry on '%s'.", odomTopic_.c_str());
        }
        // Subscribe to PoseStamped if pose_topic is non-empty (alternative source)
        if (!poseTopic_.empty()) {
            poseSubscription_ = create_subscription<PoseStamped>(
                    poseTopic_, 10, bind(&AutoMapper::poseCallback, this, _1));
            RCLCPP_INFO(get_logger(), "Subscribing to PoseStamped on '%s'.", poseTopic_.c_str());
        }
        if (odomTopic_.empty() && poseTopic_.empty()) {
            RCLCPP_ERROR(get_logger(), "Neither odom_topic nor pose_topic is set — no pose source!");
        }

        mapSubscription_ = create_subscription<OccupancyGrid>(
                mapTopic_, 10, bind(&AutoMapper::updateFullMap, this, _1));

        markerArrayPublisher_ = create_publisher<MarkerArray>("/frontiers", 10);
        poseNavigator_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");

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
    double min_frontier_length_m_ = 0.25;
    double min_distance_to_frontier_m_ = 0.75;
    double max_distance_to_frontier_m_ = 40.0;
    double frontier_size_weight_ = 1.0;
    double frontier_distance_weight_ = 0.35;
    double frontier_distance_cap_m_ = 20.0;
    int min_free_threshold_ = 4;
    Costmap2D costmap_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_;
    Publisher<MarkerArray>::SharedPtr markerArrayPublisher_;
    MarkerArray markersMsg_;
    Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    bool isExploring_ = false;
    bool isStopped_ = false;  // true once stop() has been called — prevents result_callback cascade
    steady_clock::time_point next_explore_time_ = steady_clock::now();  // backoff after rejection
    int markerId_;
    string mapPath_;
    string mapTopic_;
    string odomTopic_;
    string poseTopic_;

    Subscription<Odometry>::SharedPtr   odomSubscription_;   // nav_msgs/Odometry (odom_topic)
    Subscription<PoseStamped>::SharedPtr poseSubscription_;  // geometry_msgs/PoseStamped (pose_topic)
    PoseWithCovarianceStamped::UniquePtr pose_;
    bool hasNavigated_ = false;  // true once at least one goal has been accepted

    array<unsigned char, 256> costTranslationTable_ = initTranslationTable();

    static array<unsigned char, 256> initTranslationTable() {
        array<unsigned char, 256> cost_translation_table{};

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
        vector<Point> points;
        string getKey() const { return to_string(centroid.x) + "," + to_string(centroid.y); }
    };

    double frontierDistance(const Frontier & frontier, const Point & position) const {
        return sqrt(pow((double(frontier.centroid.x) - double(position.x)), 2.0) +
                    pow((double(frontier.centroid.y) - double(position.y)), 2.0));
    }

    double scoreFrontier(const Frontier & frontier, const Point & position) const {
        const double distance = frontierDistance(frontier, position);
        const double frontier_length_m = frontier.points.size() * costmap_.getResolution();
        const double clamped_distance = std::min(distance, frontier_distance_cap_m_);
        return frontier_size_weight_ * frontier_length_m +
               frontier_distance_weight_ * clamped_distance;
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
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000, // Throttle to every 5 seconds
                "Map received on topic '%s', but waiting for initial pose on topic '%s' to begin exploring.",
                mapTopic_.c_str(), poseTopic_.c_str());
            return;
        }
        RCLCPP_DEBUG(get_logger(), "updateFullMap...");
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
        lock_guard<Costmap2D::mutex_t> lock(*mutex);

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

    void drawMarkers(const vector<Frontier> &frontiers) {
        for (const auto &frontier: frontiers) {
            RCLCPP_DEBUG(get_logger(), "visualising %f,%f ", frontier.centroid.x, frontier.centroid.y);
            ColorRGBA green;
            green.r = 0;
            green.g = 1.0;
            green.b = 0;
            green.a = 1.0;

            vector<Marker> &markers = markersMsg_.markers;
            Marker m;

            m.header.frame_id = "map";
            m.header.stamp = now();
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
            markers.push_back(m);
            markerArrayPublisher_->publish(markersMsg_);
        }
    }

    void clearMarkers() {
        for (auto &m: markersMsg_.markers) {
            m.action = Marker::DELETE;
        }
        markerArrayPublisher_->publish(markersMsg_);
    }

    void stop() {
        if (isStopped_) return;  // prevent cascading stop from result_callbacks
        isStopped_ = true;
        RCLCPP_INFO(get_logger(), "Stopped...");
        odomSubscription_.reset();
        poseSubscription_.reset();
        mapSubscription_.reset();
        poseNavigator_->async_cancel_all_goals();
        saveMap();
        clearMarkers();
    }

    void explore() {
        if (isExploring_ || isStopped_) { return; }
        if (steady_clock::now() < next_explore_time_) { return; }  // backoff after rejection
        auto frontiers = findFrontiers();
        if (frontiers.empty()) {
            if (!hasNavigated_) {
                // Map too sparse to find frontiers yet — wait for more scans.
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "No frontiers found yet — waiting for map to populate...");
                return;
            }
            RCLCPP_WARN(get_logger(), "No frontiers remaining — exploration complete.");
            stop();
            return;
        }
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

        // Validate that the frontier centroid is in a traversable cell.
        // The centroid (average of frontier cells) can land inside obstacles,
        // especially at tunnel entrances where frontiers wrap around corners.
        {
            unsigned int goal_mx, goal_my;
            if (costmap_.worldToMap(frontier.centroid.x, frontier.centroid.y, goal_mx, goal_my)) {
                auto cost = costmap_.getCost(goal_mx, goal_my);
                if (cost == LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    RCLCPP_WARN(get_logger(),
                        "Frontier centroid (%.2f, %.2f) is inside obstacle (cost=%d) — skipping",
                        frontier.centroid.x, frontier.centroid.y, cost);
                    next_explore_time_ = steady_clock::now() + 2s;
                    return;
                }
            }
        }

        drawMarkers(frontiers);
        auto goal = NavigateToPose::Goal();
        goal.pose.pose.position = frontier.centroid;
        goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = "map";

        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontier.centroid.x, frontier.centroid.y);

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

        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) {
            isExploring_ = false;
            saveMap();
            clearMarkers();
            explore();
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Goal reached");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code");
                    break;
            }
        };
        poseNavigator_->async_send_goal(goal, send_goal_options);
    }

    void saveMap() {
        auto map_saver_cli = create_client<nav2_msgs::srv::SaveMap>("/map_server/save_map");

        if (!map_saver_cli->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "map_saver service not available — skipping map save.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Saving map to %s ...", mapPath_.c_str());

        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->map_topic = mapTopic_;
        request->map_url = mapPath_;
        request->image_format = "pgm";
        request->map_mode = "trinary";

        map_saver_cli->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Save map request sent for map at %s", mapPath_.c_str());
    }

    vector<unsigned int> nhood8(unsigned int idx) {
        unsigned int mx, my;
        vector<unsigned int> out;
        costmap_.indexToCells(idx, mx, my);
        const int x = static_cast<int>(mx);
        const int y = static_cast<int>(my);
        const int sx = static_cast<int>(costmap_.getSizeInCellsX());
        const int sy = static_cast<int>(costmap_.getSizeInCellsY());
        const pair<int, int> directions[] = {
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
                                  const vector<bool> &frontier_flag) {
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

    Frontier buildNewFrontier(unsigned int neighborCell, vector<bool> &frontier_flag) {
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;

        queue<unsigned int> bfs;
        bfs.push(neighborCell);

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

    vector<Frontier> findFrontiers() {
        vector<Frontier> frontier_list;
        const auto position = pose_->pose.pose.position;
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
            RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        lock_guard<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        auto map_ = costmap_.getCharMap();
        auto size_x_ = costmap_.getSizeInCellsX();
        auto size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        vector<bool> frontier_flag(size_x_ * size_y_,
                                   false);
        vector<bool> visited_flag(size_x_ * size_y_,
                                  false);

        // initialize breadth first search
        queue<unsigned int> bfs;

        unsigned int pos = costmap_.getIndex(mx, my);

        // If the robot's cell is not traversable (common with VDB+patchworkpp since the
        // robot's immediate vicinity has no lidar rays), search outward for the nearest
        // traversable cell and seed the BFS from there instead.
        if (!isTraversable(map_[pos])) {
            queue<unsigned int> seed_bfs;
            vector<bool> seed_visited(size_x_ * size_y_, false);
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
    init(argc, argv);
    spin(make_shared<AutoMapper>());
    shutdown();
    return 0;
}
