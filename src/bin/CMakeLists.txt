remake_add_directories(carmen)
remake_add_directories(
  gui
  COMPONENT gui
  IF WITH_GUI
)
remake_add_directories(
  test
  COMPONENT test
  IF WITH_TESTING
)
if(WITH_GUI AND WITH_TESTING)
  remake_add_directories(
    gui-test
    COMPONENT test
  )
endif(WITH_GUI AND WITH_TESTING)
