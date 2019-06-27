cmake_minimum_required(VERSION 2.8.3)
project(motion_ctrl)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
)

add_message_files(
  FILES
  DriveInfo.msg
  SequiturData.msg
  IMUStats.msg
  DeadReckoningData.msg
  SensorProcessingData.msg
  )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    motion_ctrl
    src/motion_ctrl.cpp
)

target_link_libraries(
    motion_ctrl
    ${catkin_LIBRARIES}
)

add_dependencies(
    motion_ctrl
    motion_ctrl_generate_messages_cpp
)

add_executable(
    map_ctrl
    src/map_ctrl.cpp
)

target_link_libraries(
    map_ctrl
    ${catkin_LIBRARIES}
)

add_executable(
    sensor_processing
    src/sensor_processing.cpp
)

target_link_libraries(
    sensor_processing
    ${catkin_LIBRARIES}
)

add_dependencies(
    sensor_processing
    sensor_processing_generate_messages_cpp
)
