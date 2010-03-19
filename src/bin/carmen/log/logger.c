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

#include "param_interface.h"
#include "writelog.h"

carmen_FILE *outfile = NULL;
double logger_starttime;

static int log_odometry = 1;
static int log_arm = 1;
static int log_laser = 1;
static int log_robot_laser = 1;
static int log_localize = 1;
static int log_simulator = 1;
static int log_params = 1;
static int log_gps = 1;
static int log_imu = 1;
static int log_sonars = 1;
static int log_bumpers = 1;
static int log_pantilt = 1;
static int log_motioncmds = 0; 

void get_logger_params(int argc, char** argv) {

  int num_items;

  carmen_param_t param_list[] = {
    {"logger", "odometry",    CARMEN_PARAM_ONOFF, &log_odometry, 0, NULL},
    {"logger", "arm",         CARMEN_PARAM_ONOFF, &log_arm, 0, NULL},
    {"logger", "laser",       CARMEN_PARAM_ONOFF, &log_laser, 0, NULL},
    {"logger", "robot_laser", CARMEN_PARAM_ONOFF, &log_robot_laser, 0, NULL},
    {"logger", "localize",    CARMEN_PARAM_ONOFF, &log_localize, 0, NULL},
    {"logger", "params",      CARMEN_PARAM_ONOFF, &log_params, 0, NULL},
    {"logger", "simulator",   CARMEN_PARAM_ONOFF, &log_simulator, 0, NULL},
    {"logger", "sonar",       CARMEN_PARAM_ONOFF, &log_sonars, 0, NULL},
    {"logger", "bumper",      CARMEN_PARAM_ONOFF, &log_bumpers, 0, NULL},
    {"logger", "gps",         CARMEN_PARAM_ONOFF, &log_gps, 0, NULL},
    {"logger", "imu",         CARMEN_PARAM_ONOFF, &log_imu, 0, NULL},
    {"logger", "motioncmds",  CARMEN_PARAM_ONOFF, &log_motioncmds, 0, NULL}
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);
}

void get_all_params(void)
{
  char **variables, **values, **modules;
  int list_length, index, num_modules, module_index;
  char *robot_name, *hostname;

  robot_name = carmen_param_get_robot();
  carmen_param_get_modules(&modules, &num_modules);
  carmen_logwrite_write_robot_name(robot_name, outfile);
  free(robot_name);
  carmen_param_get_paramserver_host(&hostname);
  for(module_index = 0; module_index < num_modules; module_index++) {
    if(carmen_param_get_all(modules[module_index], &variables, &values, NULL,
			    &list_length) < 0) {
      IPC_perror("Error retrieving all variables of module");
      exit(-1);
    }
    for(index = 0; index < list_length; index++) {
      carmen_logwrite_write_param(modules[module_index], variables[index], 
				  values[index], carmen_get_time(), 
				  hostname, outfile, carmen_get_time());
      free(variables[index]);
      free(values[index]);
    }
    free(variables);
    free(values);
    free(modules[module_index]);
  }
  free(hostname);
  free(modules);
}

void param_change_handler(carmen_param_variable_change_message *msg)
{
  carmen_logwrite_write_param(msg->module_name, msg->variable_name, 
			      msg->value, msg->timestamp, 
			      msg->host, outfile, carmen_get_time());
}

void carmen_simulator_truepos_handler(carmen_simulator_truepos_message
				      *truepos)
{
  fprintf(stderr, "T");
  carmen_logwrite_write_truepos(truepos, outfile, 
				carmen_get_time() - logger_starttime);
}

void base_odometry_handler(carmen_base_odometry_message *odometry)
{
  fprintf(stderr, "O");
  carmen_logwrite_write_odometry(odometry, outfile, 
				 carmen_get_time() - logger_starttime);
}

void base_sonar_handler(carmen_base_sonar_message *sonar)
{
  fprintf(stderr, "S");
  carmen_logwrite_write_base_sonar(sonar, outfile, 
				 carmen_get_time() - logger_starttime);
}


void base_bumper_handler(carmen_base_bumper_message *bumper)
{
  fprintf(stderr, "B");
  carmen_logwrite_write_base_bumper(bumper, outfile, 
				 carmen_get_time() - logger_starttime);
}

void arm_state_handler(carmen_arm_state_message *arm)
{
  fprintf(stderr, "A");
  carmen_logwrite_write_arm(arm, outfile, 
			    carmen_get_time() - logger_starttime);
}

void pantilt_scanmark_handler(carmen_pantilt_scanmark_message *scanmark)
{
  fprintf(stderr, "M");
  carmen_logwrite_write_pantilt_scanmark(scanmark, outfile, 
			    carmen_get_time() - logger_starttime);
}

void pantilt_laserpos_handler(carmen_pantilt_laserpos_message *laserpos)
{
  fprintf(stderr, "P");
  carmen_logwrite_write_pantilt_laserpos(laserpos, outfile, 
			    carmen_get_time() - logger_starttime);
}


void robot_frontlaser_handler(carmen_robot_laser_message *laser)
{
  fprintf(stderr, "F");
  carmen_logwrite_write_robot_laser(laser, 1, outfile, 
				    carmen_get_time() - logger_starttime);
}

void robot_rearlaser_handler(carmen_robot_laser_message *laser)
{
  fprintf(stderr, "R");
  carmen_logwrite_write_robot_laser(laser, 2, outfile, 
				    carmen_get_time() - logger_starttime);
}

void laser_laser1_handler(carmen_laser_laser_message *laser)
{
  fprintf(stderr, "1");
  carmen_logwrite_write_laser_laser(laser, 1, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser2_handler(carmen_laser_laser_message *laser)
{
  fprintf(stderr, "2");
  carmen_logwrite_write_laser_laser(laser, 2, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser3_handler(carmen_laser_laser_message *laser)
{
  fprintf(stderr, "3");
  carmen_logwrite_write_laser_laser(laser, 3, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser4_handler(carmen_laser_laser_message *laser)
{
  fprintf(stderr, "4");
  carmen_logwrite_write_laser_laser(laser, 4, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser5_handler(carmen_laser_laser_message *laser)
{
  fprintf(stderr, "5");
  carmen_logwrite_write_laser_laser(laser, 5, outfile,
				    carmen_get_time() - logger_starttime);
}

void localize_handler(carmen_localize_globalpos_message *msg)
{
  fprintf(stderr, "L");
  carmen_logwrite_write_localize(msg, outfile, carmen_get_time() - 
				 logger_starttime);
}

static void sync_handler(carmen_logger_sync_message *sync)
{
  carmen_logwrite_write_sync(sync, outfile);
}

void ipc_gps_gpgga_handler( carmen_gps_gpgga_message *gps_data)
{
  if(gps_data->num_satellites ==0)
    fprintf(stderr, "(G)");
  else
    fprintf(stderr, "G");
  
  carmen_logger_write_gps_gpgga(gps_data, outfile, carmen_get_time() - logger_starttime);
}


void ipc_gps_gprmc_handler( carmen_gps_gprmc_message *gps_data)
{
  fprintf(stderr, "g");
  carmen_logger_write_gps_gprmc(gps_data, outfile, carmen_get_time() - logger_starttime);
}


void imu_handler(carmen_imu_message *msg)
{
  fprintf(stderr, "i");
  carmen_logwrite_write_imu(msg, outfile, carmen_get_time() - logger_starttime);
}

void robot_follow_trajectory_handler( carmen_robot_follow_trajectory_message *msg)
{
	fprintf(stderr, "t");
	carmen_logwrite_write_robot_follow_trajectory(msg, outfile, carmen_get_time() - logger_starttime);
}

void robot_vector_move_handler( carmen_robot_vector_move_message *msg )
{
	fprintf(stderr, "m");
	carmen_logwrite_write_robot_vector_move(msg, outfile, carmen_get_time() - logger_starttime);
}

void robot_velocity_handler( carmen_robot_velocity_message *msg )
{
	fprintf(stderr, "v");
	carmen_logwrite_write_robot_velocity(msg, outfile, carmen_get_time() - logger_starttime);
}

void base_velocity_handler( carmen_base_velocity_message *msg )
{
	fprintf(stderr, "b");
	carmen_logwrite_write_base_velocity(msg, outfile, carmen_get_time() - logger_starttime);
}

void logger_comment_handler( carmen_logger_comment_message *msg )
{
	fprintf(stderr, "C");
	carmen_logwrite_write_logger_comment(msg, outfile, carmen_get_time() - logger_starttime);
}



void register_ipc_messages(void)
{
  carmen_subscribe_message(CARMEN_LOGGER_SYNC_NAME, CARMEN_LOGGER_SYNC_FMT,
			   NULL, sizeof(carmen_logger_sync_message),
			   (carmen_handler_t)sync_handler, 
			   CARMEN_SUBSCRIBE_LATEST);

  carmen_subscribe_message(CARMEN_PARAM_VARIABLE_CHANGE_NAME, 
			   CARMEN_PARAM_VARIABLE_CHANGE_FMT,
			   NULL, sizeof(carmen_param_variable_change_message),
			   (carmen_handler_t)param_change_handler, 
			   CARMEN_SUBSCRIBE_LATEST);
}

void shutdown_module(int sig)
{
  if(sig == SIGINT) {
    carmen_fclose(outfile);
    carmen_ipc_disconnect();
    fprintf(stderr, "\nDisconnecting.\n");
    exit(0);
  }
}

int main(int argc, char **argv)
{
  char filename[1024];
  char key;

  /* initialize connection to IPC network */
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);	

  /* open logfile, check if file overwrites something */
  if(argc < 2) 
    carmen_die("usage: %s <logfile>\n", argv[0]);
  sprintf(filename, argv[1]);

  outfile = carmen_fopen(filename, "r");
  if (outfile != NULL) {
    fprintf(stderr, "Overwrite %s? ", filename);
    scanf("%c", &key);
    if (toupper(key) != 'Y')
      exit(-1);
    carmen_fclose(outfile);
  }
  outfile = carmen_fopen(filename, "w");
  if(outfile == NULL)
    carmen_die("Error: Could not open file %s for writing.\n", filename);
  carmen_logwrite_write_header(outfile);


  get_logger_params(argc, argv);

  if  ( !(log_odometry && log_laser && log_robot_laser ) )
    carmen_warn("\nWARNING: You are neither logging laser nor odometry messages!\n");
  


  if (log_params)
    get_all_params();

  register_ipc_messages();


  if (log_odometry) 
    carmen_base_subscribe_odometry_message(NULL, (carmen_handler_t)
					   base_odometry_handler, 
					   CARMEN_SUBSCRIBE_ALL);

  if (log_sonars) 
    carmen_base_subscribe_sonar_message(NULL, (carmen_handler_t)
					   base_sonar_handler, 
					   CARMEN_SUBSCRIBE_ALL);
  if (log_bumpers) 
    carmen_base_subscribe_bumper_message(NULL, (carmen_handler_t)
					   base_bumper_handler, 
					   CARMEN_SUBSCRIBE_ALL);

  if (log_arm) 
    carmen_arm_subscribe_state_message(NULL, (carmen_handler_t)
				       arm_state_handler, 
				       CARMEN_SUBSCRIBE_ALL);

  if (log_pantilt) {
    carmen_pantilt_subscribe_scanmark_message (NULL, (carmen_handler_t)
					       pantilt_scanmark_handler, 
					       CARMEN_SUBSCRIBE_ALL);
    
    carmen_pantilt_subscribe_laserpos_message (NULL, (carmen_handler_t)
					       pantilt_laserpos_handler, 
					       CARMEN_SUBSCRIBE_ALL);
  }

  if (log_robot_laser) {
    carmen_robot_subscribe_frontlaser_message(NULL, (carmen_handler_t)
					      robot_frontlaser_handler, 
					      CARMEN_SUBSCRIBE_ALL);
    carmen_robot_subscribe_rearlaser_message(NULL, (carmen_handler_t)
					     robot_rearlaser_handler, 
					     CARMEN_SUBSCRIBE_ALL);
  }

  if (log_laser) {
    carmen_laser_subscribe_laser1_message(NULL, (carmen_handler_t)
					  laser_laser1_handler, 
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser2_message(NULL, (carmen_handler_t)
					  laser_laser2_handler, 
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser3_message(NULL, (carmen_handler_t)
					  laser_laser3_handler, 
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser4_message(NULL, (carmen_handler_t)
					  laser_laser4_handler, 
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser5_message(NULL, (carmen_handler_t)
					  laser_laser5_handler, 
					  CARMEN_SUBSCRIBE_ALL);
  }

  if (log_localize) {
    carmen_localize_subscribe_globalpos_message(NULL, (carmen_handler_t)
						localize_handler,
						CARMEN_SUBSCRIBE_ALL);
  }

  if (log_simulator) {

    carmen_simulator_subscribe_truepos_message(NULL, (carmen_handler_t)  
					       carmen_simulator_truepos_handler,
					       CARMEN_SUBSCRIBE_ALL);
  }

  if (log_imu) {

    carmen_imu_subscribe_imu_message(NULL, (carmen_handler_t)  
				     imu_handler,
				     CARMEN_SUBSCRIBE_ALL);
  }
  
  
  if (log_gps) {
    carmen_gps_subscribe_nmea_message( NULL,
				       (carmen_handler_t) ipc_gps_gpgga_handler,
				       CARMEN_SUBSCRIBE_ALL );
    
    carmen_gps_subscribe_nmea_rmc_message( NULL,
					   (carmen_handler_t) ipc_gps_gprmc_handler,
					   CARMEN_SUBSCRIBE_ALL );
  }
  
  if (log_motioncmds) {
  	 carmen_robot_subscribe_vector_move_message( NULL,
  	 					 (carmen_handler_t) robot_vector_move_handler,
  	 					 CARMEN_SUBSCRIBE_ALL );
  	 carmen_robot_subscribe_follow_trajectory_message( NULL,
  	 					 (carmen_handler_t) robot_follow_trajectory_handler,
  	 					 CARMEN_SUBSCRIBE_ALL );
	 carmen_robot_subscribe_velocity_message( NULL,
	 					 (carmen_handler_t) robot_velocity_handler,
	 					 CARMEN_SUBSCRIBE_ALL );		
	 carmen_base_subscribe_velocity_message( NULL,
	 					 (carmen_handler_t) base_velocity_handler,
	 					 CARMEN_SUBSCRIBE_ALL );							 			 
  }

  carmen_logger_subscribe_comment_message( NULL,
  	 					 (carmen_handler_t) logger_comment_handler,
  	 					 CARMEN_SUBSCRIBE_ALL );

  signal(SIGINT, shutdown_module);
  logger_starttime = carmen_get_time();
  carmen_ipc_dispatch();
  return 0;
}
