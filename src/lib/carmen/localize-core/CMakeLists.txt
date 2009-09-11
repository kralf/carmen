remake_add_library(
  localize_core
  LINK localize_interface localize_motion param_interface robot_interface
    map_interface
)
remake_add_headers()
