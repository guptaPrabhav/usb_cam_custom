cmake_minimum_required(VERSION 3.5)
project(usb_cam_custom)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_action)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ChangeImageMode.action"
)

# add_executable(image_proc src/image_proc.cpp)
add_executable(image_mode_action_server src/image_mode_action_server.cpp)

ament_target_dependencies(image_mode_action_server rclcpp rclcpp_action sensor_msgs cv_bridge OpenCV)

# Link the generated interfaces
rosidl_target_interfaces(image_mode_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  # image_proc
  image_mode_action_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()