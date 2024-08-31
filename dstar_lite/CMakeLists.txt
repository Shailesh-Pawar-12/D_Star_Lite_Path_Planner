cmake_minimum_required(VERSION 3.8)
project(dstar_lite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dstar_lite_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  


# ////////////////////////////////////////////////////////////////////

add_library(dstar_lite src/dstar_lite.cpp )

ament_target_dependencies(
  dstar_lite)
target_include_directories(dstar_lite PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${nav_msgs_INCLUDE_DIRS})
  # ${nlohmann_json_schema_validator_INCLUDE_DIRS})
target_compile_features(dstar_lite PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++

ament_export_targets(dstar_lite HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/dstar_lite
  DESTINATION include
)
 install(TARGETS dstar_lite
 EXPORT dstar_lite
 LIBRARY DESTINATION lib
 ARCHIVE DESTINATION lib
 RUNTIME DESTINATION bin
 INCLUDES DESTINATION include/dstar_lite
 )


# ////////////////////////////////////////////////////////////////////////////////


add_executable(dstar_node src/main.cpp src/dstar_node.cpp)
ament_target_dependencies(dstar_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  dstar_lite_interfaces  
  rclcpp_action
  visualization_msgs
  rclcpp_lifecycle  
)
target_link_libraries(dstar_node
dstar_lite
)


add_executable(map_publisher src/map_publisher.cpp)
ament_target_dependencies(map_publisher rclcpp nav_msgs)

target_include_directories(dstar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



add_executable(dstar_lite_action_client src/dstar_lite_action_client.cpp)

target_include_directories(dstar_lite_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dstar_lite_action_client
  rclcpp
  geometry_msgs
  dstar_lite_interfaces  
  rclcpp_action
)

install(TARGETS
  map_publisher
  dstar_node
  dstar_lite_action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
