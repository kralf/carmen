remake_find_package(ZLIB)
remake_find_library(m math.h PACKAGE libm)

remake_include(*)
remake_add_directories()
remake_pkg_config_generate()
