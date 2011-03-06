remake_define(LASER_USE_PTHREAD ON)

remake_add_library(
  laser_dev
  LINK serial laser_interface ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_headers()
