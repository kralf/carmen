remake_add_library(
  simulator_core
  LINK simulator_interface param_interface localize_interface map_interface
    localize_motion geometry
)
remake_add_headers()
