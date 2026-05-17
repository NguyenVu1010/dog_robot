cmake_minimum_required(VERSION 3.8)
project(dog_robot_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
