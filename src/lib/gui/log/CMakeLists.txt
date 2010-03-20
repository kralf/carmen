remake_qt3_moc(log_graphics laserdisplay.h MT)
remake_add_library(
  log_graphics
  LINK log_tools map_io ${ImageMagick_LIBRARIES} ${QT_LIBRARIES}
)
