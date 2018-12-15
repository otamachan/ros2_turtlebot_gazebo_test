cmake_minimum_required(VERSION 3.5)
project(ros2_turtlebot_gazebo_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
