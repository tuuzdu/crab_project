# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "crab_msgs: 7 messages, 1 services")

set(MSG_I_FLAGS "-Icrab_msgs:/home/ubuntu/catkin_ws/src/crab_msgs/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(crab_msgs_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/BodyCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)
_generate_msg_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)
_generate_msg_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/BodyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)
_generate_msg_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegIKRequest.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg;/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)
_generate_msg_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)
_generate_msg_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegsJointsState.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)
_generate_msg_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/GaitCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)

### Generating Services
_generate_srv_cpp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/srv/GetLegIKSolver.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg;/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
)

### Generating Module File
_generate_module_cpp(crab_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(crab_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(crab_msgs_generate_messages crab_msgs_generate_messages_cpp)

# target for backward compatibility
add_custom_target(crab_msgs_gencpp)
add_dependencies(crab_msgs_gencpp crab_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crab_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/BodyCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)
_generate_msg_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)
_generate_msg_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/BodyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)
_generate_msg_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegIKRequest.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg;/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)
_generate_msg_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)
_generate_msg_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegsJointsState.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)
_generate_msg_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/GaitCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)

### Generating Services
_generate_srv_lisp(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/srv/GetLegIKSolver.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg;/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
)

### Generating Module File
_generate_module_lisp(crab_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(crab_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(crab_msgs_generate_messages crab_msgs_generate_messages_lisp)

# target for backward compatibility
add_custom_target(crab_msgs_genlisp)
add_dependencies(crab_msgs_genlisp crab_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crab_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/BodyCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)
_generate_msg_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)
_generate_msg_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/BodyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)
_generate_msg_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegIKRequest.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg;/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)
_generate_msg_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)
_generate_msg_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegsJointsState.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)
_generate_msg_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/GaitCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)

### Generating Services
_generate_srv_py(crab_msgs
  "/home/ubuntu/catkin_ws/src/crab_msgs/srv/GetLegIKSolver.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegJointsState.msg;/home/ubuntu/catkin_ws/src/crab_msgs/msg/LegPositionState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
)

### Generating Module File
_generate_module_py(crab_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(crab_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(crab_msgs_generate_messages crab_msgs_generate_messages_py)

# target for backward compatibility
add_custom_target(crab_msgs_genpy)
add_dependencies(crab_msgs_genpy crab_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crab_msgs_generate_messages_py)


debug_message(2 "crab_msgs: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crab_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(crab_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crab_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(crab_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crab_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(crab_msgs_generate_messages_py std_msgs_generate_messages_py)
