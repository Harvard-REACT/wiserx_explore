# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "explore_lite: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iexplore_lite:/home/react-ws-1/catkin_ws/src/m-explore/explore/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(explore_lite_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg" NAME_WE)
add_custom_target(_explore_lite_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "explore_lite" "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(explore_lite
  "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/explore_lite
)

### Generating Services

### Generating Module File
_generate_module_cpp(explore_lite
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/explore_lite
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(explore_lite_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(explore_lite_generate_messages explore_lite_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg" NAME_WE)
add_dependencies(explore_lite_generate_messages_cpp _explore_lite_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(explore_lite_gencpp)
add_dependencies(explore_lite_gencpp explore_lite_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS explore_lite_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(explore_lite
  "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/explore_lite
)

### Generating Services

### Generating Module File
_generate_module_eus(explore_lite
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/explore_lite
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(explore_lite_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(explore_lite_generate_messages explore_lite_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg" NAME_WE)
add_dependencies(explore_lite_generate_messages_eus _explore_lite_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(explore_lite_geneus)
add_dependencies(explore_lite_geneus explore_lite_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS explore_lite_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(explore_lite
  "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/explore_lite
)

### Generating Services

### Generating Module File
_generate_module_lisp(explore_lite
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/explore_lite
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(explore_lite_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(explore_lite_generate_messages explore_lite_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg" NAME_WE)
add_dependencies(explore_lite_generate_messages_lisp _explore_lite_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(explore_lite_genlisp)
add_dependencies(explore_lite_genlisp explore_lite_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS explore_lite_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(explore_lite
  "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/explore_lite
)

### Generating Services

### Generating Module File
_generate_module_nodejs(explore_lite
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/explore_lite
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(explore_lite_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(explore_lite_generate_messages explore_lite_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg" NAME_WE)
add_dependencies(explore_lite_generate_messages_nodejs _explore_lite_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(explore_lite_gennodejs)
add_dependencies(explore_lite_gennodejs explore_lite_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS explore_lite_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(explore_lite
  "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/explore_lite
)

### Generating Services

### Generating Module File
_generate_module_py(explore_lite
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/explore_lite
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(explore_lite_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(explore_lite_generate_messages explore_lite_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/react-ws-1/catkin_ws/src/m-explore/explore/msg/RangeBearing.msg" NAME_WE)
add_dependencies(explore_lite_generate_messages_py _explore_lite_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(explore_lite_genpy)
add_dependencies(explore_lite_genpy explore_lite_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS explore_lite_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/explore_lite)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/explore_lite
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(explore_lite_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/explore_lite)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/explore_lite
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(explore_lite_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/explore_lite)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/explore_lite
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(explore_lite_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/explore_lite)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/explore_lite
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(explore_lite_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/explore_lite)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/explore_lite\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/explore_lite
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(explore_lite_generate_messages_py std_msgs_generate_messages_py)
endif()
