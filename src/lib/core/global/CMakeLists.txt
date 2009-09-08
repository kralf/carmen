remake_find_library(
  m math.h
  PACKAGE libm
)
remake_find_package(ZLIB)

remake_add_library(
  global
  LINK ${M_LIBRARY} ${ZLIB_LIBRARIES} ${IPC_SERVER_LIBRARY}
)
remake_add_headers()
