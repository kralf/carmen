remake_find_package(zlib CONFIG)

remake_include(*)
remake_add_directories()
remake_pkg_config_generate(REQUIRES zlib)
