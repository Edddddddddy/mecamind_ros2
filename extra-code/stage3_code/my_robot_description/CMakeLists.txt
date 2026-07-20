cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf scripts meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/verify_urdf.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
