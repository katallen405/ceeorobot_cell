cmake_minimum_required(VERSION 3.8)
project(ceeorobot_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
