cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

project(image_capture)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_publisher src/run_publish_images.cpp)
target_include_directories(image_publisher
    SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_publisher ${OpenCV_LIBRARIES})
ament_target_dependencies(image_publisher cv_bridge image_transport rclcpp std_msgs)


install(
  TARGETS
    image_publisher
  DESTINATION
    lib/${PROJECT_NAME}
    )

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
