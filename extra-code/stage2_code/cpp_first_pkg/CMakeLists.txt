cmake_minimum_required(VERSION 3.8)
project(cpp_first_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_minimal src/minimal_node.cpp)
ament_target_dependencies(cpp_minimal rclcpp)
install(TARGETS
  cpp_minimal
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(cpp_oop src/oop_node.cpp)
ament_target_dependencies(cpp_oop rclcpp)
install(TARGETS
  cpp_oop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
