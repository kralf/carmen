remake_add_directories(carmen)
remake_add_directories(
  test
  COMPONENT test
  IF WITH_TESTING
)
remake_add_directories(
  gui
  COMPONENT gui
  IF WITH_GUI
)
