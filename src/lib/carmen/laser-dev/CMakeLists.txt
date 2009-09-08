remake_find_package(Threads)
if(NOT ${CMAKE_USE_PTHREADS_INIT})
  message(FATAL_ERROR "Missing POSIX thread support!")
endif(NOT ${CMAKE_USE_PTHREADS_INIT})

remake_define(LASER_USE_PTHREAD ON)

remake_include(
  ../../interface/laser
)
remake_add_library(
  laser_dev
  LINK serial ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_headers()
