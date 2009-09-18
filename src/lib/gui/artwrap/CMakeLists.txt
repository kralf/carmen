remake_include(${LIBART2_INCLUDE_DIRS})
remake_add_library(
  artwrap
  LINK ${LIBART2_LIBRARIES}
)
remake_add_headers()
