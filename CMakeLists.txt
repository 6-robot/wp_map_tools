cmake_minimum_required(VERSION 3.8)
project(wp_map_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
include_directories(${QT_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_dependencies(
  "interactive_markers"
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "visualization_msgs"
  "geometry_msgs"
  "std_msgs"
  "yaml-cpp")

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg" 
  "srv/SaveWaypoints.srv"
  "srv/GetWaypointByName.srv"
  DEPENDENCIES 
  "geometry_msgs"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 库
set(wp_map_plugins_headers_to_moc
  include/wp_map_tools/add_waypoint_tool.hpp
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

set(library_name wp_map_plugins)

add_library(${library_name} SHARED
  include/wp_map_tools/add_waypoint_tool.hpp
  src/add_waypoint_tool.cpp
  ${wp_map_plugins_headers_to_moc}
)

set(dependencies
  geometry_msgs
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  tf2_geometry_msgs
  visualization_msgs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
  ${QT_LIBRARIES}
  "${cpp_typesupport_target}"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  FILES 
  plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# 编译节点
add_executable(wp_edit_node src/wp_edit_node.cpp)
ament_target_dependencies(wp_edit_node 
  "rclcpp" 
  "visualization_msgs"
  "interactive_markers"
  "yaml-cpp"
)
target_link_libraries(wp_edit_node "yaml-cpp" "${cpp_typesupport_target}")

add_executable(wp_saver src/wp_saver.cpp)
ament_target_dependencies(wp_saver "rclcpp" )
target_link_libraries(wp_saver "${cpp_typesupport_target}")

add_executable(wp_navi_server src/wp_navi_server.cpp)
ament_target_dependencies(wp_navi_server "rclcpp" "std_msgs" "rclcpp_action" "nav2_msgs" "tf2_ros")
target_link_libraries(wp_navi_server "${cpp_typesupport_target}")

add_executable(demo_navi_waypoint src/demo_navi_waypoint.cpp)
ament_target_dependencies(demo_navi_waypoint "rclcpp" "std_msgs" )
target_link_libraries(demo_navi_waypoint "${cpp_typesupport_target}")

# 安装节点
install(DIRECTORY launch meshes rviz msg srv icons
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS
    wp_edit_node 
    wp_saver
    wp_navi_server
    demo_navi_waypoint
  DESTINATION
    lib/${PROJECT_NAME}
)

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

ament_package()
