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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(auto_mapper src/auto_mapper.cpp)

install(
        DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
