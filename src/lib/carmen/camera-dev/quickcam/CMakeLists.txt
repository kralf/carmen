enable_language(ASM-ATT)
remake_add_library(
  quickcam_dev *.c *.s
  LINK camera_interface param_interface
)
remake_add_headers()
