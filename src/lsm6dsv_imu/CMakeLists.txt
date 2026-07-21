cmake_minimum_required(VERSION 2.8.3)
project(lsm6dsv_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lsm6dsv_imu_node src/lsm6dsv_imu_node.cpp)
target_link_libraries(lsm6dsv_imu_node ${catkin_LIBRARIES})
target_link_libraries (lsm6dsv_imu_node wiringPi)

