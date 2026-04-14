cmake_minimum_required(VERSION 3.8)
project(auto_mapper)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

set(deps
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        nav2_msgs
        tf2
        tf2_ros
        rclcpp_action
        nav2_costmap_2d
        nav2_map_server
        visualization_msgs
        nav2_util
        std_msgs
        angles
        tf2_geometry_msgs
        map_msgs)
set(targets
        auto_mapper)

# Optional map backends — auto_mapper works with any OccupancyGrid source.
# These are only needed for the bundled launch files, not the node itself.
set(optional_deps
        slam_toolbox)

foreach (d ${deps})
  find_package(${d} REQUIRED)
endforeach ()

foreach (d ${optional_deps})
  find_package(${d} QUIET)
endforeach ()

foreach (t ${targets})
  add_executable(${t} src/${t}.cpp)
  ament_target_dependencies(${t} ${deps})
endforeach ()


install(TARGETS
        ${targets}
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY config launch rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
