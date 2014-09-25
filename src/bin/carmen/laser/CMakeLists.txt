remake_define(LASER_USE_PTHREAD ON)
remake_add_executables(LINK param_interface laser_dev
  ${CMAKE_THREAD_LIBS_INIT})
