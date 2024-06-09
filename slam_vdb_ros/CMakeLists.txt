# ---------------------------------------------
# CMake configuration for slam_vdb_ros
# ---------------------------------------------
# author - Sriharsha Ghanta (ghanta1996@gmail.com)
# --------------------------------------------

# set minimum cmake version
cmake_minimum_required(VERSION 3.8)
project(slam_vdb_ros)

option(SLAM_VDB_LOCATION "Location of SLAM_VDB" ../slam_vdb/install)

# Default build type is Release
set(CMAKE_CXX_FLAGS_RELEASE)
set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD_TYPE})
  message(STATUS "Using default build type: ${CMAKE_BUILD_TYPE}")
else()
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

set(CMAKE_CXX_STANDARD 17)
message(STATUS "Require C++${CMAKE_CXX_STANDARD}")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(DEFINED SLAM_VDB_LOCATION)
  find_package(SLAM_VDB 1.0.0 PATHS ${SLAM_VDB_LOCATION} REQUIRED)
else()
  find_package(SLAM_VDB 1.0.0 REQUIRED)
endif()

set(dependencies
  ament_cmake
  rclcpp
  sensor_msgs)

add_executable(slam_vdb_node src/main.cpp src/slam_vdb_ros.cpp)
target_link_libraries(slam_vdb_node SLAM_VDB::slam_vdb)
target_include_directories(slam_vdb_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(slam_vdb_node rclcpp service_msgs std_msgs sensor_msgs rcl_interfaces)

install(
  TARGETS slam_vdb_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(
# DIRECTORY launch
# DESTINATION share/${PROJECT_NAME}
# )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()