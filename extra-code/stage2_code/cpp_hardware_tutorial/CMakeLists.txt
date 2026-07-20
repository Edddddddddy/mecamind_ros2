cmake_minimum_required(VERSION 3.8)
project(cpp_hardware_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interfaces REQUIRED)

add_executable(hardware_status_publisher src/hardware_status_publisher.cpp)
ament_target_dependencies(hardware_status_publisher rclcpp hardware_interfaces)

add_executable(hardware_status_subscriber src/hardware_status_subscriber.cpp)
ament_target_dependencies(hardware_status_subscriber rclcpp hardware_interfaces)

add_executable(rectangle_area_server src/rectangle_area_server.cpp)
ament_target_dependencies(rectangle_area_server rclcpp hardware_interfaces)

install(TARGETS
  hardware_status_publisher
  hardware_status_subscriber
  rectangle_area_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

