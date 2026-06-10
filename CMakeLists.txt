cmake_minimum_required(VERSION 3.8)
project(auto_mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wshadow
    -Wno-ignored-qualifiers
  )
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(auto_mapper src/auto_mapper.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncrustify is excluded deliberately: the codebase predates the ament
  # style and keeps its original formatting. Run `ament_uncrustify
  # --reformat` and drop this exclusion to adopt the ament style wholesale.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_exploration_logic test/test_exploration_logic.cpp)
  target_include_directories(test_exploration_logic PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
endif()

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
