# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${crab_msgs_DIR}/.." "msg" crab_msgs_MSG_INCLUDE_DIRS UNIQUE)
set(crab_msgs_MSG_DEPENDENCIES std_msgs)
