remake_add_library(
  readlog
  LINK param_interface base_interface arm_interface pantilt_interface
    robot_interface laser_interface localize_interface simulator_interface
    imu_interface gps_nmea_interface
)
remake_add_headers()
