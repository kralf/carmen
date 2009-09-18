remake_find_package(Threads)
if(NOT ${CMAKE_USE_PTHREADS_INIT})
  message(FATAL_ERROR "Missing POSIX thread support!")
endif(NOT ${CMAKE_USE_PTHREADS_INIT})

remake_add_library(
  multicentral
  LINK global param_interface ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_headers()
