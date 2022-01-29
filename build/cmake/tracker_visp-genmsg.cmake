# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tracker_visp: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tracker_visp_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv" NAME_WE)
add_custom_target(_tracker_visp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tracker_visp" "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv" "sensor_msgs/Image:std_msgs/String:geometry_msgs/Vector3:sensor_msgs/CameraInfo:geometry_msgs/Transform:sensor_msgs/RegionOfInterest:std_msgs/Header:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tracker_visp
  "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracker_visp
)

### Generating Module File
_generate_module_cpp(tracker_visp
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracker_visp
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tracker_visp_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tracker_visp_generate_messages tracker_visp_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv" NAME_WE)
add_dependencies(tracker_visp_generate_messages_cpp _tracker_visp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracker_visp_gencpp)
add_dependencies(tracker_visp_gencpp tracker_visp_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracker_visp_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(tracker_visp
  "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracker_visp
)

### Generating Module File
_generate_module_eus(tracker_visp
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracker_visp
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tracker_visp_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tracker_visp_generate_messages tracker_visp_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv" NAME_WE)
add_dependencies(tracker_visp_generate_messages_eus _tracker_visp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracker_visp_geneus)
add_dependencies(tracker_visp_geneus tracker_visp_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracker_visp_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tracker_visp
  "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracker_visp
)

### Generating Module File
_generate_module_lisp(tracker_visp
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracker_visp
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tracker_visp_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tracker_visp_generate_messages tracker_visp_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv" NAME_WE)
add_dependencies(tracker_visp_generate_messages_lisp _tracker_visp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracker_visp_genlisp)
add_dependencies(tracker_visp_genlisp tracker_visp_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracker_visp_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(tracker_visp
  "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracker_visp
)

### Generating Module File
_generate_module_nodejs(tracker_visp
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracker_visp
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tracker_visp_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tracker_visp_generate_messages tracker_visp_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv" NAME_WE)
add_dependencies(tracker_visp_generate_messages_nodejs _tracker_visp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracker_visp_gennodejs)
add_dependencies(tracker_visp_gennodejs tracker_visp_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracker_visp_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tracker_visp
  "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracker_visp
)

### Generating Module File
_generate_module_py(tracker_visp
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracker_visp
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tracker_visp_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tracker_visp_generate_messages tracker_visp_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/giulio/catkin_ws/src/Jenga-tracker/srv/YolactInitializeCaoPose.srv" NAME_WE)
add_dependencies(tracker_visp_generate_messages_py _tracker_visp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracker_visp_genpy)
add_dependencies(tracker_visp_genpy tracker_visp_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracker_visp_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracker_visp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracker_visp
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tracker_visp_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tracker_visp_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(tracker_visp_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracker_visp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracker_visp
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tracker_visp_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tracker_visp_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(tracker_visp_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracker_visp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracker_visp
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tracker_visp_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tracker_visp_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(tracker_visp_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracker_visp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracker_visp
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tracker_visp_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tracker_visp_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(tracker_visp_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracker_visp)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracker_visp\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracker_visp
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tracker_visp_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tracker_visp_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(tracker_visp_generate_messages_py sensor_msgs_generate_messages_py)
endif()
