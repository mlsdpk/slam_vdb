cmake_minimum_required(VERSION 3.8)
project(slam_vdb)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
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

add_subdirectory(third_party/nanovdb/nanovdb)

set(dependencies
  ament_cmake
  rclcpp
  sensor_msgs)

include_directories(include third_party/nanovdb)

add_library(slam_vdb src/slam_vdb.cpp)
ament_target_dependencies(slam_vdb ${dependencies})

add_executable(slam_vdb_node src/slam_vdb_node.cpp)
target_link_libraries(slam_vdb_node slam_vdb)
target_include_directories(slam_vdb_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(slam_vdb_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS slam_vdb_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS slam_vdb
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

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
ament_export_libraries(slam_vdb)
ament_export_dependencies(${dependencies})
ament_package()