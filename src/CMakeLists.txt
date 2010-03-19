remake_project_option(WITH_TESTING "testing binaries" ON)
remake_project_option(WITH_GUI "graphical user interface (GUI)" ON)

if(WITH_GUI)
  remake_find_package(GTK+2 CONFIG gtk+-2.0)
  remake_find_package(gthread2 CONFIG gthread-2.0)
  remake_find_package(libart2 CONFIG libart-2.0)
  remake_find_package(JPEG)
  remake_find_package(ImageMagick COMPONENTS MagickWand)
endif(WITH_GUI)

remake_define(REDHAT_52 ON)
remake_define(REDHAT_6 ON)
remake_define(REDHAT_71 ON)

remake_add_directories(lib)
remake_add_directories(bin)
