remake_add_directories(
  core
  COMPONENT core
)
remake_add_directories(
  interface
  COMPONENT interface
)
remake_add_directories(carmen)
remake_add_directories(
  gui
  COMPONENT gui
  IF WITH_GUI
)
