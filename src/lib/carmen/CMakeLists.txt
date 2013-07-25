remake_find_package(Threads)
if(NOT ${CMAKE_USE_PTHREADS_INIT})
  message(FATAL_ERROR "Missing POSIX thread support!")
endif(NOT ${CMAKE_USE_PTHREADS_INIT})

remake_include(
  ../core/*
  ../interface/*
  *
)
remake_add_headers()
remake_add_directories()
remake_pkg_config_generate(REQUIRES carmen-interface)
