 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include "global.h"

#include "playback_interface.h"
#include "readlog.h"

#define        MAX_LINE_LENGTH           100000

carmen_FILE *logfile = NULL;
carmen_logfile_index_p logfile_index = NULL;

double playback_starttime = 0.0;
double last_logfile_time = 0.0;
double playback_speed = 1.0;

int current_position = 0;
int offset = 0; 
int paused = 1;
int fast = 0;
int advance_frame = 0;
int rewind_frame = 0;
int basic_messages = 0;

double playback_timestamp;

carmen_base_odometry_message odometry;
carmen_simulator_truepos_message truepos;
carmen_robot_laser_message laser1, laser2, laser3, laser4, laser5;
carmen_laser_laser_message rawlaser1, rawlaser2, rawlaser3, rawlaser4, rawlaser5;
carmen_localize_globalpos_message globalpos;
carmen_arm_state_message arm;
carmen_base_sonar_message sonar;
carmen_base_bumper_message bumper;

carmen_imu_message imu;
carmen_gps_gpgga_message gpsgga;
carmen_gps_gprmc_message gpsrmc;

void print_playback_status(void)
{
  char str[100];
  
  if(paused)
    sprintf(str, "PAUSED ");
  else
    sprintf(str, "PLAYING");
  fprintf(stderr, "\rSTATUS:    %s   TIME:    %.2f    ", 
	  str, playback_timestamp);
}

void playback_command_handler(carmen_playback_command_message *command)
{
  switch(command->cmd) {
  case CARMEN_PLAYBACK_COMMAND_PLAY:
    if(paused) {
      playback_starttime = 0.0;
      paused = 0;
      //      fprintf(stderr, " PLAY ");
      print_playback_status();
    }
    break;
  case CARMEN_PLAYBACK_COMMAND_STOP:
    if(!paused) {
      paused = 1;
      //      fprintf(stderr, " STOP ");
      print_playback_status();
    }
    break;
  case CARMEN_PLAYBACK_COMMAND_RESET:
    if(!paused)
      paused = 1;
    current_position = 0;
    playback_starttime = 0.0;
    //    fprintf(stderr, "\nRESET ");
    playback_timestamp = 0;
    print_playback_status();
    break;
  case CARMEN_PLAYBACK_COMMAND_FORWARD:
    offset = command->arg;
    if(offset > 0 && paused)
      advance_frame = 1;
    break;
  case CARMEN_PLAYBACK_COMMAND_REWIND:
    offset = -1 * command->arg;
    if(offset < 0 && paused)
      advance_frame = 1;
    break;
  case CARMEN_PLAYBACK_COMMAND_FWD_SINGLE:
    if(!paused)
      paused = 1;
    advance_frame = 1;
    break;
  case CARMEN_PLAYBACK_COMMAND_RWD_SINGLE:
    if(!paused)
      paused = 1;
    rewind_frame = 1;
    break;
  case CARMEN_PLAYBACK_COMMAND_SET_SPEED:
  	break;
  }
  if(fabs(command->speed - playback_speed) > 0.001) {
    playback_starttime = 0.0;
    playback_speed = command->speed;
    print_playback_status();
  }
}

void register_ipc_messages(void)
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_BASE_ODOMETRY_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_ODOMETRY_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ODOMETRY_NAME);

  err = IPC_defineMsg(CARMEN_ARM_STATE_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_ARM_STATE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ARM_STATE_NAME);

  err = IPC_defineMsg(CARMEN_SIMULATOR_TRUEPOS_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_SIMULATOR_TRUEPOS_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_SIMULATOR_TRUEPOS_NAME);

  err = IPC_defineMsg(CARMEN_ROBOT_FRONTLASER_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_ROBOT_FRONTLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_FRONTLASER_NAME);

  err = IPC_defineMsg(CARMEN_ROBOT_REARLASER_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_ROBOT_REARLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_REARLASER_NAME);

  err = IPC_defineMsg(CARMEN_LASER_FRONTLASER_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_FRONTLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_FRONTLASER_NAME);

  err = IPC_defineMsg(CARMEN_LASER_REARLASER_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_REARLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_REARLASER_NAME);

  err = IPC_defineMsg(CARMEN_LASER_LASER3_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_LASER3_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER3_NAME);

  err = IPC_defineMsg(CARMEN_LASER_LASER4_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_LASER4_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER4_NAME);

  err = IPC_defineMsg(CARMEN_LASER_LASER5_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_LASER5_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER5_NAME);

  err = IPC_defineMsg(CARMEN_LOCALIZE_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LOCALIZE_GLOBALPOS_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_LOCALIZE_GLOBALPOS_NAME);
  
  err = IPC_defineMsg(CARMEN_GPS_GPGGA_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_GPS_GPGGA_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_GPS_GPGGA_MESSAGE_NAME);
  
  err = IPC_defineMsg(CARMEN_GPS_GPRMC_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_GPS_GPRMC_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_GPS_GPRMC_MESSAGE_NAME);
  
  
  carmen_subscribe_message(CARMEN_PLAYBACK_COMMAND_NAME, 
                           CARMEN_PLAYBACK_COMMAND_FMT,
                           NULL, sizeof(carmen_playback_command_message),
                           (carmen_handler_t)playback_command_handler, 
                           CARMEN_SUBSCRIBE_LATEST);
}

// ts is in logfile time
void wait_for_timestamp(double ts) 
{
  double current_time; // in logfile time
  struct timeval tv;

  // playback_starttime is offset between file-start and playback-start
  if(playback_starttime == 0.0)
    playback_starttime = (carmen_get_time() - ts / playback_speed); 
  current_time = (carmen_get_time() - playback_starttime) * playback_speed;
  if(!fast && !paused && ts > current_time) {
    double towait = (ts - current_time) / playback_speed;
    tv.tv_sec = (int)floor(towait);
    tv.tv_usec = (towait - tv.tv_sec) * 1e6;
    select(0, NULL, NULL, NULL, &tv);
  }
}

typedef char *(*converter_func)(char *, void *);

typedef struct {
  char *logger_message_name;
  char *ipc_message_name;
  converter_func conv_func;
  void *message_data;
  int interpreted;
} logger_callback_t;

logger_callback_t logger_callbacks[] = {
  {"RAWLASER1", CARMEN_LASER_FRONTLASER_NAME, 
   (converter_func)carmen_string_to_laser_laser_message, &rawlaser1, 0},
  {"RAWLASER2", CARMEN_LASER_REARLASER_NAME, 
   (converter_func)carmen_string_to_laser_laser_message, &rawlaser2, 0},
  {"RAWLASER3", CARMEN_LASER_LASER3_NAME, 
   (converter_func)carmen_string_to_laser_laser_message, &rawlaser3, 0},
  {"RAWLASER4", CARMEN_LASER_LASER4_NAME, 
   (converter_func)carmen_string_to_laser_laser_message, &rawlaser4, 0},
  {"RAWLASER5", CARMEN_LASER_LASER5_NAME, 
   (converter_func)carmen_string_to_laser_laser_message, &rawlaser5, 0},
  {"ROBOTLASER1", CARMEN_ROBOT_FRONTLASER_NAME, 
   (converter_func)carmen_string_to_robot_laser_message, &laser1, 0},
  {"ROBOTLASER2", CARMEN_ROBOT_REARLASER_NAME, 
   (converter_func)carmen_string_to_robot_laser_message, &laser2, 0},
  {"ROBOTLASER3", CARMEN_ROBOT_FRONTLASER_NAME, 
   (converter_func)carmen_string_to_robot_laser_message, &laser3, 0},
  {"ROBOTLASER4", CARMEN_ROBOT_FRONTLASER_NAME, 
   (converter_func)carmen_string_to_robot_laser_message, &laser4, 0},
  {"ROBOTLASER5", CARMEN_ROBOT_FRONTLASER_NAME, 
   (converter_func)carmen_string_to_robot_laser_message, &laser5, 0},
  {"ODOM", CARMEN_BASE_ODOMETRY_NAME, 
   (converter_func)carmen_string_to_base_odometry_message, &odometry, 0},
  {"SONAR", CARMEN_BASE_SONAR_NAME,
    (converter_func) carmen_string_to_base_sonar_message, &sonar, 0},
  {"BUMPER", CARMEN_BASE_BUMPER_NAME,
    (converter_func) carmen_string_to_base_bumper_message, &bumper, 0},
  {"ARM", CARMEN_ARM_STATE_NAME, 
   (converter_func)carmen_string_to_arm_state_message, &arm, 0},
  {"TRUEPOS", CARMEN_SIMULATOR_TRUEPOS_NAME,
   (converter_func)carmen_string_to_simulator_truepos_message, &odometry, 0},
  {"FLASER", CARMEN_ROBOT_FRONTLASER_NAME, 
   (converter_func)carmen_string_to_robot_laser_message_orig, &laser1, 0},
  {"RLASER", CARMEN_ROBOT_REARLASER_NAME,
   (converter_func)carmen_string_to_robot_laser_message_orig, &laser2, 0},
  {"LASER3", CARMEN_LASER_LASER3_NAME, 
   (converter_func)carmen_string_to_laser_laser_message_orig, &rawlaser3, 0},
  {"LASER4", CARMEN_LASER_LASER4_NAME, 
   (converter_func)carmen_string_to_laser_laser_message_orig, &rawlaser4, 0},
  {"LASER5", CARMEN_LASER_LASER5_NAME, 
   (converter_func)carmen_string_to_laser_laser_message_orig, &rawlaser5, 0},
  {"IMU", CARMEN_IMU_MESSAGE_NAME,
    (converter_func) carmen_string_to_imu_message, &imu, 0},
  {"NMEAGGA", CARMEN_GPS_GPGGA_MESSAGE_NAME, 
   (converter_func)carmen_string_to_gps_gpgga_message, &gpsgga, 0},
  {"NMEARMC", CARMEN_GPS_GPRMC_MESSAGE_NAME, 
   (converter_func)carmen_string_to_gps_gprmc_message, &gpsrmc, 0},
};

int read_message(int message_num, int publish)
{
  char line[MAX_LINE_LENGTH], *current_pos;
  IPC_RETURN_TYPE err;
  int i, j;
  char command[100];
  static double last_update = 0;
  double current_time;

  carmen_logfile_read_line(logfile_index, logfile, message_num, 
			   MAX_LINE_LENGTH, line);
  current_pos = carmen_next_word(line);

  for(i = 0; i < (int)(sizeof(logger_callbacks) / 
		       sizeof(logger_callback_t)); i++) {
    /* copy the command over */
    j = 0;
    while(line[j] != ' ') {
      command[j] = line[j];
      j++;
    }
    command[j] = '\0';
    if(strncmp(command, logger_callbacks[i].logger_message_name, j) == 0) {
      if(!basic_messages || !logger_callbacks[i].interpreted) {
	current_pos = 
	  logger_callbacks[i].conv_func(current_pos, 
					logger_callbacks[i].message_data);
	playback_timestamp = atof(current_pos);
	if(publish) {
	  current_time = carmen_get_time();
	  if(current_time - last_update > 1.0) {
	    print_playback_status();
	    last_update = current_time;
	  }
	  wait_for_timestamp(playback_timestamp);
	  err = IPC_publishData(logger_callbacks[i].ipc_message_name, 
				logger_callbacks[i].message_data);
	}
	/* return 1 if it is a front laser message */
	return (strcmp(command, "FLASER") == 0);
      }
    }
  }
  return 0;
}

void main_playback_loop(void)
{
  int laser;

  print_playback_status();
  while(1) {
    if(offset != 0) {
      playback_starttime = 0.0;
      current_position += offset;
      if(current_position < 0)
	current_position = 0;
      if(current_position >= logfile_index->num_messages - 1)
	current_position = logfile_index->num_messages - 2;
      offset = 0;
    }
    
    if(!paused && current_position >= logfile_index->num_messages - 1) {
      paused = 1;
      current_position = 0;
      playback_starttime = 0.0;
      playback_timestamp = 0;
      print_playback_status();
    }
    else if(!paused && current_position < logfile_index->num_messages - 1) {
      read_message(current_position, 1);
      current_position++;
    }
    else if(paused && advance_frame) {
      laser = 0;
      while(current_position < logfile_index->num_messages - 1 && !laser) {
	laser = read_message(current_position, 1);
	current_position++;
      } 
      advance_frame = 0;
    }
    else if(paused && rewind_frame) {
      laser = 0;
      while(current_position > 0 && !laser) {
	current_position--;
	laser = read_message(current_position, 0);
      } 
      laser = 0;
      while(current_position > 0 && !laser) {
	current_position--;
	laser = read_message(current_position, 0);
      }
      read_message(current_position, 1);
      current_position++;
      rewind_frame = 0;
    }
    if(paused)
      carmen_ipc_sleep(0.1);
    if(fast)
      carmen_ipc_sleep(0.000001);
    else
      carmen_ipc_sleep(0.0001);
  }
}


void usage(char *fmt, ...) 
{
  va_list args;
  
  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);
  
  fprintf(stderr, "Usage: playback filename <args>\n");
  fprintf(stderr, "\t-fast         - ignore timestamps.\n");
  exit(-1);
}

void read_parameters(int argc, char **argv)
{
  int index;

  if(argc < 2)
    usage("Needs at least one argument.\n");

  for(index = 0; index < argc; index++) {
    if(strncmp(argv[index], "-h", 2) == 0 || 
       strncmp(argv[index], "--help", 6) == 0)
      usage(NULL);
    if(strncmp(argv[index], "-fast", 5) == 0)
      fast = 1;
    if(strncmp(argv[index], "-autostart", 5) == 0)
      paused = 0;
    if(strncmp(argv[index], "-basic", 6) == 0)
      basic_messages = 1;
  }
}

void shutdown_playback_module(int sig)
{
  if(sig == SIGINT) {
    fprintf(stderr, "\n");
    exit(1);
  }
}

int main(int argc, char **argv)
{
  memset(&odometry, 0, sizeof(odometry));
  memset(&arm, 0, sizeof(arm));
  memset(&imu, 0, sizeof(imu));
  memset(&sonar, 0, sizeof(sonar));
  memset(&bumper, 0, sizeof(bumper));
  memset(&truepos, 0, sizeof(truepos));
  memset(&laser1, 0, sizeof(laser1));
  memset(&laser2, 0, sizeof(laser2));
  memset(&laser3, 0, sizeof(laser3));
  memset(&laser4, 0, sizeof(laser4));
  memset(&laser5, 0, sizeof(laser5));
  memset(&rawlaser1, 0, sizeof(rawlaser1));
  memset(&rawlaser2, 0, sizeof(rawlaser2));
  memset(&rawlaser3, 0, sizeof(rawlaser3));
  memset(&rawlaser4, 0, sizeof(rawlaser4));
  memset(&rawlaser5, 0, sizeof(rawlaser5));
  memset(&gpsgga, 0, sizeof(gpsgga));
  memset(&gpsrmc, 0, sizeof(gpsrmc));


  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);
  
  register_ipc_messages();
  read_parameters(argc, argv);
  signal(SIGINT, shutdown_playback_module);
  
  logfile = carmen_fopen(argv[1], "r");
  if(logfile == NULL)
    carmen_die("Error: could not open file %s for reading.\n", argv[1]);
  logfile_index = carmen_logfile_index_messages(logfile);
  main_playback_loop();
  return 0;
}

