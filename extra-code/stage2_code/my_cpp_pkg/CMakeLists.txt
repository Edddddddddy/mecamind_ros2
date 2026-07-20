cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(param_publisher src/param_publisher.cpp)
ament_target_dependencies(param_publisher rclcpp rcl_interfaces std_msgs)

install(TARGETS
  param_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
