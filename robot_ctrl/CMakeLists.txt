cmake_minimum_required(VERSION 2.8.3)
project(robot_ctrl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  sequitur_pose
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_ctrl
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    robot_ctrl
    src/robot_ctrl.cpp
    src/lib-vector.cpp
    src/position-proc.cpp
)

target_link_libraries(
    robot_ctrl
    ${catkin_LIBRARIES}
)

add_dependencies(
    robot_ctrl
    ${catkin_EXPORTED_TARGETS}
)

add_executable(
    positioning
    src/positioning.cpp
    src/lib-vector.cpp
    src/position-proc.cpp
)

target_link_libraries(
    positioning
    ${catkin_LIBRARIES}
)

add_dependencies(
    positioning
    ${catkin_EXPORTED_TARGETS}
)
