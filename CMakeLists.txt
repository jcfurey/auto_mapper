cmake_minimum_required(VERSION 3.16)
project(auto_mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wshadow
    -Wno-ignored-qualifiers
  )
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(auto_mapper_logic SHARED src/frontier_search.cpp src/search_worker.cpp)
target_link_libraries(auto_mapper_logic Threads::Threads)
ament_auto_add_library(auto_mapper_node SHARED src/auto_mapper.cpp)
target_link_libraries(auto_mapper_node auto_mapper_logic)
ament_auto_add_executable(auto_mapper src/main.cpp)
target_link_libraries(auto_mapper auto_mapper_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_exploration_logic test/test_exploration_logic.cpp)
  target_include_directories(test_exploration_logic PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  ament_add_gtest(test_frontier_search test/test_frontier_search.cpp)
  target_link_libraries(test_frontier_search auto_mapper_logic)

  ament_add_gtest(test_auto_mapper test/test_auto_mapper.cpp TIMEOUT 120)
  target_link_libraries(test_auto_mapper auto_mapper_node)
  set_tests_properties(test_auto_mapper PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=178;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
endif()

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Scoped header install (include/auto_mapper instead of include) — the
# default from ROS 2 Kilted onward; silences the ament_auto deprecation note.
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
