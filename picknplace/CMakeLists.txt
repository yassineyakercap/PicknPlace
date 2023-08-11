cmake_minimum_required(VERSION 3.5)
project(picknplace)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


include_directories(include)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR scripts)

file(GLOB SCRIPT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
install(PROGRAMS ${SCRIPT_FILES} DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch urdf worlds maps rviz DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/picknplace.dsv.in")

ament_package()
