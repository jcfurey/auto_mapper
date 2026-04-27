cmake_minimum_required(VERSION 3.8)
project(auto_mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wshadow -Wunused-variable -Wunused-function
    -Wno-ignored-qualifiers
  )
endif ()

find_package(ament_cmake REQUIRED)

# Only the deps actually included by src/auto_mapper.cpp.
set(deps
        rclcpp
        rclcpp_action
        geometry_msgs
        nav_msgs
        nav2_msgs
        nav2_costmap_2d
        std_msgs
        std_srvs
        visualization_msgs)

foreach (d ${deps})
  find_package(${d} REQUIRED)
endforeach ()

add_executable(auto_mapper src/auto_mapper.cpp)
ament_target_dependencies(auto_mapper ${deps})

install(TARGETS
        auto_mapper
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
