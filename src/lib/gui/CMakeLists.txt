remake_include(
  ../core/*
  ../interface/*
  ../carmen/*
  *
  ${GTK+2_INCLUDE_DIRS}
  ${ImageMagick_INCLUDE_DIRS}
)
remake_qt3_include(MT)

remake_add_headers()
remake_add_directories()
remake_pkg_config_generate(REQUIRES carmen gtk+-2.0 libart-2.0 qt-mt)
