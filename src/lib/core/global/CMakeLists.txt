remake_find_library(
  m math.h
  PACKAGE libm
)
remake_find_package(ZLIB)

remake_add_library(
  global
  LINK ipc ${M_LIBRARY} ${ZLIB_LIBRARIES}
)
remake_add_headers()
