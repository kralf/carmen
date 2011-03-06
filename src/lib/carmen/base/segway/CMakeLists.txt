remake_find_library(canlib canlib.h PACKAGE canlib OPTIONAL)
if(CANLIB_FOUND)
  remake_define(REDHAT_52 WITH_CANLIB)
endif(CANLIB_FOUND)

remake_add_library(segway_base LINK global)
remake_add_headers()
