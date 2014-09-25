remake_set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)

remake_component(gui)
if(CARMEN_GUI_COMPONENT_BUILD)
  remake_find_package(X11)
  remake_find_package(GTK+2 CONFIG gtk+-2.0)
  remake_find_package(gthread2 CONFIG gthread-2.0)
  remake_find_package(libart2 CONFIG libart-2.0)
  remake_find_package(JPEG)
  remake_find_package(ImageMagick COMPONENTS MagickWand)
endif(CARMEN_GUI_COMPONENT_BUILD)

remake_define(REDHAT_52 ON)
remake_define(REDHAT_6 ON)
remake_define(REDHAT_71 ON)

remake_add_directories(lib)
remake_add_directories(bin)
