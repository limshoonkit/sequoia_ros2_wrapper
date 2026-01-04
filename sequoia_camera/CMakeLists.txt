cmake_minimum_required(VERSION 3.8)
project(sequoia_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find Python
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SunshineSensor.msg"
  "srv/CameraControl.srv"
  DEPENDENCIES std_msgs
)

# Sequoia camera component
add_library(sequoia_camera_component SHARED
  src/sequoia_camera_component.cpp
  src/ptpy_wrapper.cpp
)
target_compile_features(sequoia_camera_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(sequoia_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Python3_INCLUDE_DIRS}
)
ament_target_dependencies(sequoia_camera_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sequoia_camera_component
  "${cpp_typesupport_target}"
  ${Python3_LIBRARIES}
)

# Export Python symbols for embedded modules
if(UNIX AND NOT APPLE)
  target_link_options(sequoia_camera_component PRIVATE "-Wl,--export-dynamic")
endif()

rclcpp_components_register_nodes(sequoia_camera_component "sequoia_camera::SequoiaCameraComponent")

# Install
install(TARGETS
  sequoia_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(
  PROGRAMS scripts/unmount_sequoia.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(sequoia_camera_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_package()
