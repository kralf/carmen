remake_find_library(
  ipc-server ipc.h
  PACKAGE ipc
)

remake_define(REDHAT_52 ON)
remake_define(REDHAT_6 ON)
remake_define(REDHAT_71 ON)

remake_add_directories()
