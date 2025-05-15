cmake_minimum_required(VERSION 3.5)
project(kinect)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(kinect_receiver src/kinect_receiver.cpp)
ament_target_dependencies(kinect_receiver rclcpp sensor_msgs std_msgs builtin_interfaces pcl_conversions)
target_link_libraries(kinect_receiver ${PCL_LIBRARIES})
install(TARGETS kinect_receiver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
