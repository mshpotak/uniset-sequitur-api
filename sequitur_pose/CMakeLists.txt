cmake_minimum_required(VERSION 2.8.3)
project(sequitur_pose)

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
  SequiturData.msg
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
    sequitur_pose
    src/sequitur_pose.cpp
    src/sequitur-api.cpp
    src/unix-network.cpp
)

target_link_libraries(
    sequitur_pose
    ${catkin_LIBRARIES}
)

add_dependencies(
    sequitur_pose
    sequitur_pose_generate_messages_cpp
)
