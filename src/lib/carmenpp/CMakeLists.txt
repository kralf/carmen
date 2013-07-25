remake_include(
  ../core/*
  ../interface/*
  ../carmen/*
)
remake_add_library(
  cpp
  LINK global readlog writelog map_interface
)
remake_add_headers()
remake_pkg_config_generate(REQUIRES carmen)
