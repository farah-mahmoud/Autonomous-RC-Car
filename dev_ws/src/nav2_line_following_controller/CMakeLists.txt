cmake_minimum_required(VERSION 3.5)
project(nav2_line_following_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

add_library(nav2_line_following_controller SHARED
  src/line_following_controller.cpp
  src/geometry.cpp)


# require c++17
target_compile_features(nav2_line_following_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# prevent pluginlib from using boost
target_compile_definitions(nav2_line_following_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2_line_following_controller
  ${dependencies}
)

install(TARGETS nav2_line_following_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(geometry_test src/geometry_test.cpp src/geometry.cpp)
  ament_add_gtest(route_test src/route_test.cpp src/geometry.cpp)

  ament_target_dependencies(route_test
  ${dependencies}
)
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_line_following_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_line_following_controller.xml)

ament_package()
