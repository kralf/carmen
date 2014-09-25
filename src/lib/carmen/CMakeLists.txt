remake_include(
  ../core/*
  ../interface/*
  *
)
remake_add_headers()
remake_add_directories()
remake_pkg_config_generate(REQUIRES carmen-interface)
