remake_find_package(GTK2 CONFIG gtk+-2.0)

remake_include(${GTK2_INCLUDE_DIRS})
remake_add_library(
  global_graphics
  LINK ${GTK2_LIBRARIES}
)
remake_add_headers()
