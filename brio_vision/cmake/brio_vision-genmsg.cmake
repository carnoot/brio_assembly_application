# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "brio_vision: 2 messages, 1 services")

set(MSG_I_FLAGS "-Ibrio_vision:/home/paul/ros_ws/src/brio_vision/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(brio_vision_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(brio_vision
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/brio_vision
)
_generate_msg_cpp(brio_vision
  "/home/paul/ros_ws/src/brio_vision/msg/Container.msg"
  "${MSG_I_FLAGS}"
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/brio_vision
)

### Generating Services
_generate_srv_cpp(brio_vision
  "/home/paul/ros_ws/src/brio_vision/srv/Is_This_Easier.srv"
  "${MSG_I_FLAGS}"
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg;/home/paul/ros_ws/src/brio_vision/msg/Container.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/brio_vision
)

### Generating Module File
_generate_module_cpp(brio_vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/brio_vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(brio_vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(brio_vision_generate_messages brio_vision_generate_messages_cpp)

# target for backward compatibility
add_custom_target(brio_vision_gencpp)
add_dependencies(brio_vision_gencpp brio_vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS brio_vision_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(brio_vision
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/brio_vision
)
_generate_msg_lisp(brio_vision
  "/home/paul/ros_ws/src/brio_vision/msg/Container.msg"
  "${MSG_I_FLAGS}"
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/brio_vision
)

### Generating Services
_generate_srv_lisp(brio_vision
  "/home/paul/ros_ws/src/brio_vision/srv/Is_This_Easier.srv"
  "${MSG_I_FLAGS}"
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg;/home/paul/ros_ws/src/brio_vision/msg/Container.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/brio_vision
)

### Generating Module File
_generate_module_lisp(brio_vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/brio_vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(brio_vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(brio_vision_generate_messages brio_vision_generate_messages_lisp)

# target for backward compatibility
add_custom_target(brio_vision_genlisp)
add_dependencies(brio_vision_genlisp brio_vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS brio_vision_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(brio_vision
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/brio_vision
)
_generate_msg_py(brio_vision
  "/home/paul/ros_ws/src/brio_vision/msg/Container.msg"
  "${MSG_I_FLAGS}"
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/brio_vision
)

### Generating Services
_generate_srv_py(brio_vision
  "/home/paul/ros_ws/src/brio_vision/srv/Is_This_Easier.srv"
  "${MSG_I_FLAGS}"
  "/home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg;/home/paul/ros_ws/src/brio_vision/msg/Container.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/brio_vision
)

### Generating Module File
_generate_module_py(brio_vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/brio_vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(brio_vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(brio_vision_generate_messages brio_vision_generate_messages_py)

# target for backward compatibility
add_custom_target(brio_vision_genpy)
add_dependencies(brio_vision_genpy brio_vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS brio_vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/brio_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/brio_vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(brio_vision_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/brio_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/brio_vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(brio_vision_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/brio_vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/brio_vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/brio_vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(brio_vision_generate_messages_py std_msgs_generate_messages_py)
