remake_include(${GTK+2_INCLUDE_DIRS})
remake_add_library(
  global_graphics
  LINK ${GTK+2_LIBRARIES} ${JPEG_LIBRARIES}
)
remake_add_headers()
