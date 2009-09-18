remake_project_option(WITH_TESTING "testing binaries" ON)
remake_project_option(WITH_GUI "graphical user interface (GUI)" ON)

remake_find_library(
  ipc-server ipc.h
  PACKAGE libipc
)

if(WITH_GUI)
  remake_find_package(GTK+2 CONFIG gtk+-2.0)
  remake_find_package(libart2 CONFIG libart-2.0)
  remake_find_package(JPEG)
endif(WITH_GUI)

remake_define(REDHAT_52 ON)
remake_define(REDHAT_6 ON)
remake_define(REDHAT_71 ON)

remake_include(${IPC_SERVER_HEADERS})

remake_add_directories()
