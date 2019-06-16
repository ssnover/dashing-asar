cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

project(image_capture)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(image_publisher src/run_publish_images.cpp)
ament_target_dependencies(image_publisher rclcpp std_msgs)


install(
  TARGETS
    image_publisher
  DESTINATION
    lib/${PROJECT_NAME}
    )

ament_package()
