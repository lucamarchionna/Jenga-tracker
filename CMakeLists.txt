cmake_minimum_required(VERSION 3.0.2)
project(tracker_visp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED) 
find_package(VISP REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  visp_ros REQUIRED
  VISP REQUIRED
  geometry_msgs
  std_msgs
  sensor_msgs

  message_generation
  roscpp
  rospy
  visp_ros
  visp_bridge

  pcl_ros
  pcl_conversions
  pcl_msgs
  
  moveit_ros_planning_interface 
  moveit_ros_move_group

  tf2
  tf2_ros
  #tf2_eigen
)

add_message_files(
  FILES
  ReferenceBlock.msg
)

add_service_files(
  FILES
  YolactInitializeCaoPose.srv
)

add_message_files(
  FILES
  angle_velocity.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

add_definitions(${VISP_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${visp_ros_INCLUDE_DIRS}

  ${PCL_INCLUDE_DIRS}
)
catkin_package(
  CATKIN_DEPENDS message_generation rospy roscpp geometry_msgs std_msgs pcl_ros pcl_msgs pcl_conversions
  DEPENDS PCL VISP visp_bridge Boost
)
# tracker
# add_executable(tracker
#   src/tracker.cpp
# )
# target_link_libraries(tracker
#   ${catkin_LIBRARIES}
#   ${VISP_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${realsense2_LIBRARY}
# )
# add_dependencies(tracker
#   ${catkin_EXPORTED_TARGETS}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )

# singleBlock_servo
# add_executable(single_block_servo
#   src/singleBlock_servo.cpp
# )

# target_link_libraries(single_block_servo
#   ${catkin_LIBRARIES}
#   ${VISP_LIBRARIES}
#   ${realsense2_LIBRARY}
#   ${PCL_LIBRARIES}
# )

# add_dependencies(single_block_servo
#   ${catkin_EXPORTED_TARGETS}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )

# old tracker
add_executable(tracker_0
  src/tracker.cpp
)

target_link_libraries(tracker_0
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(tracker_0
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(tracker_sim
  src/tracker_sim.cpp
)

target_link_libraries(tracker_sim
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(tracker_sim
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(detection
  src/tracker_params_class.cpp
)

target_link_libraries(detection
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(detection
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(tracker_params
  src/tracker_params.cpp
)

target_link_libraries(tracker_params
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(tracker_params
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Client
add_executable(client
  src/ros_client_pic.cpp
)

target_link_libraries(client
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(client
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# new tracker
add_executable(tracker_client
  src/tracker_v3.cpp
)

target_link_libraries(tracker_client
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(tracker_client
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#Rotating_base
add_executable(rotating_base
  src/clientPic_toServo.cpp
)

target_link_libraries(rotating_base
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(rotating_base
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#tracker+pbvs
add_executable(visualServo
  src/visual_servo.cpp
)

target_link_libraries(visualServo
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(visualServo
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
