cmake_minimum_required(VERSION 3.5)
project(create_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}/)
