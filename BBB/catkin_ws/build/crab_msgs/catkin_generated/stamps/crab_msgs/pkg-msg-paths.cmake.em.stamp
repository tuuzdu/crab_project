# generated from genmsg/cmake/pkg-msg-paths.cmake.em

@[if DEVELSPACE]@
# message include dirs in develspace
set(@(PROJECT_NAME)_MSG_INCLUDE_DIRS "@(PKG_MSG_INCLUDE_DIRS)")
@[else]@
# message include dirs in installspace
_prepend_path("${@(PROJECT_NAME)_DIR}/.." "@(PKG_MSG_INCLUDE_DIRS)" @(PROJECT_NAME)_MSG_INCLUDE_DIRS UNIQUE)
@[end if]@
set(@(PROJECT_NAME)_MSG_DEPENDENCIES @(ARG_DEPENDENCIES))
