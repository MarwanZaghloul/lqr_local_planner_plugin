cmake_minimum_required(VERSION 3.8)
project(lqr_local_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

include_directories(
  include
)

# Set up dependencies
set(dependencies
  rclcpp
  nav2_core
  pluginlib
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  tf2_ros
)

add_library(lqr_local_planner SHARED
  src/lqr_local_planner.cpp
)
target_include_directories(lqr_local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Prevent pluginlib from using boost
target_compile_definitions(lqr_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Link dependencies to the target
ament_target_dependencies(lqr_local_planner
  ${dependencies}
)

install(TARGETS lqr_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(lqr_local_planner)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core lqr_local_planner.xml)

ament_package()
