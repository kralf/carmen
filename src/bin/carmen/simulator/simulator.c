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

#include <signal.h>
#include "global.h"

#include "simulator.h"
#include "simulator_messages.h"
#include "localize_messages.h"

#include "param_interface.h"
#include "map_interface.h"

static carmen_simulator_config_t *simulator_config;
static int use_robot = 1;

static int publish_readings(void);

/* handlers */

static void set_object_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
			       void *clientData __attribute__ ((unused)))
{
  carmen_simulator_set_object_message msg;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_unmarshallData(formatter, callData, &msg,
                     sizeof(carmen_simulator_set_object_message));
  IPC_freeByteArray(callData);

  carmen_simulator_create_object(msg.pose.x, msg.pose.y, msg.pose.theta, 
				 CARMEN_SIMULATOR_RANDOM_OBJECT, msg.speed);
}

static void set_truepose_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
				 void *clientData __attribute__ ((unused)))
{
  carmen_simulator_set_truepose_message msg;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_unmarshallData(formatter, callData, &msg,
                     sizeof(carmen_simulator_set_truepose_message));
  IPC_freeByteArray(callData);

  simulator_config->true_pose.x = msg.pose.x;
  simulator_config->true_pose.y = msg.pose.y;
  simulator_config->true_pose.theta = msg.pose.theta;
}

static void
connect_robots_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
		       void *clientData __attribute__ ((unused)))
{
  carmen_simulator_connect_robots_message msg;
  FORMATTER_PTR formatter;
  char program_name[1024];

  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_unmarshallData(formatter, callData, &msg,
                     sizeof(carmen_simulator_connect_robots_message));
  IPC_freeByteArray(callData);

  sprintf(program_name, "%d", getpid());
  carmen_simulator_add_robot(program_name, msg.other_central); 
}



static void clear_objects_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
				  void *clientData __attribute__ ((unused)))
{
  carmen_simulator_clear_objects_message msg;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_unmarshallData(formatter, callData, &msg,
                     sizeof(carmen_default_message));
  IPC_freeByteArray(callData);

  carmen_simulator_clear_objects();	
}

static void init_handler(carmen_localize_initialize_message *init_msg)
{
  if(init_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN) {
    simulator_config->true_pose.x = init_msg->mean[0].x;
    simulator_config->true_pose.y = init_msg->mean[0].y;
    simulator_config->true_pose.theta = init_msg->mean[0].theta;
    simulator_config->tv = 0;
    simulator_config->rv = 0;
    simulator_config->target_tv = 0;
    simulator_config->target_rv = 0;
  }
}

/* handles base velocity messages */ 
static void velocity_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			     void *clientData __attribute__ ((unused)))
{
  carmen_base_velocity_message vel;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_unmarshallData(formatter, callData, &vel,
                     sizeof(carmen_base_velocity_message));
  IPC_freeByteArray(callData);
  simulator_config->target_tv = vel.tv;
  simulator_config->target_rv = vel.rv;
  simulator_config->time_of_last_command = carmen_get_time();
}

void map_update_handler(carmen_map_t *new_map) 
{
  carmen_map_p map_ptr;
  int map_x, map_y;
  carmen_point_t zero = {0, 0, 0};

  map_ptr = carmen_map_copy(new_map);
  simulator_config->map = *map_ptr;
  free(map_ptr);

  /* Reset odometry and true pose only of the robot's pose     */
  /* is not valid given the new map. Otherwise keep old poses. */
  /* This enables to activate a new map without messing up     */
  /* the odometry. */

  map_x = simulator_config->true_pose.x / 
    simulator_config->map.config.resolution;
  map_y = simulator_config->true_pose.y / 
    simulator_config->map.config.resolution;

  if(map_x < 0 || map_x >= simulator_config->map.config.x_size || 
     map_y < 0 || map_y >= simulator_config->map.config.y_size ||
     simulator_config->map.map[map_x][map_y] > .15 ||
     carmen_simulator_object_too_close(simulator_config->true_pose.x, 
				       simulator_config->true_pose.y, -1)) {
    simulator_config->odom_pose = zero;
    simulator_config->true_pose = zero;
  }
  
}

static void next_tick_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
			      void *clientData __attribute__ ((unused)))
{
  carmen_simulator_next_tick_message msg;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_unmarshallData(formatter, callData, &msg,
                     sizeof(carmen_default_message));
  IPC_freeByteArray(callData);

  if (simulator_config->sync_mode) {
    if (use_robot)
      carmen_robot_run();
    publish_readings();
  }
}

static void truepos_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
				  void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  carmen_simulator_truepos_message response;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_freeByteArray(callData);

  response.timestamp = carmen_get_time();
  response.host = carmen_get_host();
  response.truepose = simulator_config->true_pose;
  response.odometrypose = simulator_config->odom_pose;

  err = IPC_respondData(msgRef, CARMEN_SIMULATOR_TRUEPOS_NAME, &response);
  carmen_test_ipc(err, "Could not respond", CARMEN_SIMULATOR_TRUEPOS_NAME);

}

/* handles C^c */
static void shutdown_module(int x)
{
  if(x == SIGINT) {
    if (use_robot) 
      carmen_robot_shutdown(x);
    carmen_ipc_disconnect();
    carmen_warn("\nDisconnected.\n");
    exit(1);
  }
}

static int initialize_ipc(void)
{
  IPC_RETURN_TYPE err;

  /* define messages created by this module */
  err = IPC_defineMsg(CARMEN_LASER_FRONTLASER_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_FRONTLASER_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_LASER_REARLASER_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_FRONTLASER_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_BASE_SONAR_NAME,
		      IPC_VARIABLE_LENGTH,
		      CARMEN_BASE_SONAR_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_BASE_ODOMETRY_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_ODOMETRY_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_BASE_VELOCITY_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_VELOCITY_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_SIMULATOR_TRUEPOS_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_SIMULATOR_TRUEPOS_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_SIMULATOR_SET_OBJECT_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_SIMULATOR_SET_OBJECT_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_SIMULATOR_CONNECT_ROBOTS_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_SIMULATOR_CONNECT_ROBOTS_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_SIMULATOR_OBJECTS_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_SIMULATOR_OBJECTS_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_SIMULATOR_CLEAR_OBJECTS_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_DEFAULT_MESSAGE_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_SIMULATOR_NEXT_TICK_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_DEFAULT_MESSAGE_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_defineMsg(CARMEN_SIMULATOR_TRUEPOS_QUERY_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_DEFAULT_MESSAGE_FMT);
  if(err != IPC_OK)
    return -1;

  err = IPC_subscribe(CARMEN_SIMULATOR_SET_TRUEPOSE_NAME, 
		      set_truepose_handler, NULL);
  if (err != IPC_OK)
    return 1;
  IPC_setMsgQueueLength(CARMEN_SIMULATOR_SET_TRUEPOSE_NAME, 100);

  err = IPC_subscribe(CARMEN_SIMULATOR_SET_OBJECT_NAME, 
		      set_object_handler, NULL);
  if (err != IPC_OK)
    return 1;
  IPC_setMsgQueueLength(CARMEN_SIMULATOR_SET_OBJECT_NAME, 100);

  err = IPC_subscribe(CARMEN_SIMULATOR_CONNECT_ROBOTS_NAME, 
		      connect_robots_handler, NULL);
  if (err != IPC_OK)
    return 1;
  IPC_setMsgQueueLength(CARMEN_SIMULATOR_CONNECT_ROBOTS_NAME, 100);

  err = IPC_subscribe(CARMEN_SIMULATOR_CLEAR_OBJECTS_NAME, 
		      clear_objects_handler, NULL);
  if (err != IPC_OK)
    return 1;
  IPC_setMsgQueueLength(CARMEN_SIMULATOR_CLEAR_OBJECTS_NAME, 1);

  err = IPC_subscribe(CARMEN_SIMULATOR_NEXT_TICK_NAME, 
		      next_tick_handler, NULL);
  if (err != IPC_OK)
    return 1;
  IPC_setMsgQueueLength(CARMEN_SIMULATOR_NEXT_TICK_NAME, 1);

  err = IPC_subscribe(CARMEN_SIMULATOR_TRUEPOS_QUERY_NAME, 
		      truepos_query_handler, NULL);
  if (err != IPC_OK)
    return 1;

  return 0;
}

/* updates the robot's position,
   the people's position, and the laser reading 
and then publishes the laser reading and the odometry */
static int
publish_readings(void)
{
  IPC_RETURN_TYPE err = IPC_OK;
  static int first = 1;

  static carmen_laser_laser_message flaser;
  static carmen_laser_laser_message rlaser;
  static carmen_base_sonar_message sonar;
  static carmen_base_odometry_message odometry;

  static carmen_simulator_truepos_message position;
  static carmen_simulator_objects_message objects;

  double delta_time;
  double timestamp;

  if (first) {
    odometry.host = carmen_get_host();
    position.host = carmen_get_host();
    
    objects.host = carmen_get_host();
    
    odometry.x = position.truepose.x;
    odometry.y = position.truepose.y;
    odometry.theta = position.truepose.theta;
    
    odometry.tv = odometry.rv = 0;
    
    if (simulator_config->use_front_laser) {
      flaser.host = carmen_get_host();
      flaser.num_readings = 
	simulator_config->front_laser_config.num_lasers;
      flaser.range = (float *)calloc
	(simulator_config->front_laser_config.num_lasers, sizeof(float));
      carmen_test_alloc(flaser.range);

      flaser.num_remissions = 0;
      flaser.remission = 0;      

    }
    
    if (simulator_config->use_rear_laser) {
      rlaser.host = carmen_get_host();
      
      rlaser.num_readings = simulator_config->rear_laser_config.num_lasers;
      rlaser.range = (float *)calloc
	(simulator_config->rear_laser_config.num_lasers, sizeof(float));
      carmen_test_alloc(rlaser.range);

      rlaser.num_remissions = 0;
      rlaser.remission = 0;      

    }
    
    if (simulator_config->use_sonar) {
      int i;

      sonar.host = carmen_get_host();
      sonar.num_sonars = simulator_config->sonar_config.num_sonars;
      sonar.cone_angle = simulator_config->sonar_config.sensor_angle;
      sonar.sonar_offsets = (carmen_point_t*)calloc
	(sonar.num_sonars, sizeof(carmen_point_t));
      for (i = 0; i < sonar.num_sonars; ++i) {
        sonar.sonar_offsets[i].x = simulator_config->sonar_config.offsets[i].x;
        sonar.sonar_offsets[i].y = simulator_config->sonar_config.offsets[i].y;
        sonar.sonar_offsets[i].theta = simulator_config->sonar_config.offsets[i].theta;
      }
      
      sonar.range = (double *)calloc
	(simulator_config->sonar_config.num_sonars, sizeof(double));
      carmen_test_alloc(sonar.range);	  
    }
    first = 0;
  }

  timestamp = carmen_get_time();

  if (simulator_config->real_time && !simulator_config->sync_mode) {
    delta_time = timestamp - simulator_config->time_of_last_command;
    if ((simulator_config->tv > 0 || simulator_config->rv > 0) && 
	delta_time > simulator_config->motion_timeout) {
      simulator_config->target_tv = 0;
      simulator_config->target_rv = 0;
    }
  }
  carmen_simulator_recalc_pos(simulator_config);
  
  odometry.x = simulator_config->odom_pose.x;
  odometry.y = simulator_config->odom_pose.y;
  odometry.theta = simulator_config->odom_pose.theta;
  odometry.tv = simulator_config->tv;
  odometry.rv = simulator_config->rv;
  odometry.acceleration = simulator_config->acceleration;
  odometry.timestamp = timestamp;
  err = IPC_publishData(CARMEN_BASE_ODOMETRY_NAME, &odometry);
  carmen_test_ipc(err, "Could not publish base_odometry_message", 
		  CARMEN_BASE_ODOMETRY_NAME);

  position.truepose = simulator_config->true_pose;
  position.odometrypose = simulator_config->odom_pose;
  position.timestamp = timestamp;
  err = IPC_publishData(CARMEN_SIMULATOR_TRUEPOS_NAME, &position);
  carmen_test_ipc(err, "Could not publish simualator_truepos_message", 
		  CARMEN_SIMULATOR_TRUEPOS_NAME);

  carmen_simulator_update_objects(simulator_config);
  carmen_simulator_get_object_poses
    (&(objects.num_objects), &(objects.objects_list));
  objects.timestamp = timestamp;
  err = IPC_publishData(CARMEN_SIMULATOR_OBJECTS_NAME, &objects);
  carmen_test_ipc(err, "Could not publish simulator_objects_message", 
		  CARMEN_SIMULATOR_OBJECTS_NAME);

  if (simulator_config->use_front_laser) {
    carmen_simulator_calc_laser_msg(&flaser, simulator_config, 0);
    
    flaser.timestamp = timestamp;
    err = IPC_publishData(CARMEN_LASER_FRONTLASER_NAME, &flaser);
    carmen_test_ipc(err, "Could not publish laser_frontlaser_message", 
		    CARMEN_LASER_FRONTLASER_NAME);
  }
  
  if (simulator_config->use_rear_laser) {
    carmen_simulator_calc_laser_msg(&rlaser, simulator_config, 1);
    
    rlaser.timestamp = timestamp;
    err = IPC_publishData(CARMEN_LASER_REARLASER_NAME, &rlaser);
    carmen_test_ipc(err, "Could not publish laser_rearlaser_message", 
		    CARMEN_LASER_REARLASER_NAME);
  }

  if (simulator_config->use_sonar) {
    carmen_simulator_calc_sonar_msg(&sonar, simulator_config);
    
    sonar.timestamp = timestamp;
    err=IPC_publishData(CARMEN_BASE_SONAR_NAME, &sonar);
    carmen_test_ipc(err, "Could not publish base_sonar_message", 
		    CARMEN_BASE_SONAR_NAME);
  }
  
  carmen_publish_heartbeat("simulator");

  return 1;
}


void fill_laser_config_data(carmen_simulator_laser_config_t *lasercfg) 
{
  lasercfg->num_lasers = 1 + carmen_round(lasercfg->fov / lasercfg->angular_resolution);
  lasercfg->start_angle = -0.5*lasercfg->fov;

  /* give a warning if it is not a standard configuration */

  if ( fabs(lasercfg->fov - M_PI) > 1e-6 && 
       fabs(lasercfg->fov - 100.0/180.0 * M_PI) > 1e-6 && 
       fabs(lasercfg->fov -  90.0/180.0 * M_PI) > 1e-6)
    carmen_warn("Warnung: You are not using a standard SICK configuration (fov=%.4f deg)\n",
		carmen_radians_to_degrees(lasercfg->fov) );

  if ( fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(1.0)) > 1e-6 && 
       fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.5)) > 1e-6 && 
       fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.25)) > 1e-6)
    carmen_warn("Warnung: You are not using a standard SICK configuration (res=%.4f deg)\n",
		carmen_radians_to_degrees(lasercfg->angular_resolution) );

}

static void read_parameters(int argc, char *argv[], 
			    carmen_simulator_config_t *config)
{
  char *offset_string;
  int num_items;

  carmen_param_t param_list[]= {
    {"simulator", "dt", CARMEN_PARAM_DOUBLE,
     &(config->delta_t), 1, NULL},
    {"simulator", "time", CARMEN_PARAM_DOUBLE,
     &(config->real_time), 1, NULL},
    {"simulator", "sync_mode", CARMEN_PARAM_ONOFF,
     &(config->sync_mode), 1, NULL},
    {"simulator", "use_robot", CARMEN_PARAM_ONOFF, &use_robot, 1, NULL},
#ifdef OLD_MOTION_MODEL
    {"localize", "odom_a1", CARMEN_PARAM_DOUBLE, &(config->odom_a1), 1, NULL},
    {"localize", "odom_a2", CARMEN_PARAM_DOUBLE, &(config->odom_a2), 1, NULL},
    {"localize", "odom_a3", CARMEN_PARAM_DOUBLE, &(config->odom_a3), 1, NULL},
    {"localize", "odom_a4", CARMEN_PARAM_DOUBLE, &(config->odom_a4), 1, NULL},
#endif
    {"base", "motion_timeout", CARMEN_PARAM_DOUBLE, &(config->motion_timeout),
     1, NULL},
    {"robot", "frontlaser_use", CARMEN_PARAM_ONOFF, &(config->use_front_laser), 
     1, NULL},
    {"robot", "frontlaser_id", CARMEN_PARAM_INT,
     &(config->front_laser_config.id), 0, NULL}, 
    {"robot", "rearlaser_use", CARMEN_PARAM_ONOFF, &(config->use_rear_laser), 
     1, NULL},
    {"robot", "rearlaser_id", CARMEN_PARAM_INT,
     &(config->rear_laser_config.id), 0, NULL}, 
    {"robot", "use_sonar", CARMEN_PARAM_ONOFF, &(config->use_sonar), 1, NULL},
    {"robot", "width", CARMEN_PARAM_DOUBLE, &(config->width), 1,NULL},
    {"robot", "acceleration", CARMEN_PARAM_DOUBLE, &(config->acceleration),
     1,NULL} };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);


  static char frontlaser_fov_string[256];
  static char frontlaser_res_string[256];
  static char rearlaser_fov_string[256];
  static char rearlaser_res_string[256];

  sprintf(frontlaser_fov_string, "laser%d_fov", config->front_laser_config.id);
  sprintf(frontlaser_res_string, "laser%d_resolution", config->front_laser_config.id);

  sprintf(rearlaser_fov_string, "laser%d_fov", config->rear_laser_config.id);
  sprintf(rearlaser_res_string, "laser%d_resolution", config->rear_laser_config.id);
  
  carmen_param_t param_list_front_laser[] = {
    {"simulator", "frontlaser_maxrange", CARMEN_PARAM_DOUBLE, 
     &(config->front_laser_config.max_range), 1, NULL},
    {"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE, 
     &(config->front_laser_config.offset), 1, NULL},
    {"robot", "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, 
     &(config->front_laser_config.side_offset), 1, NULL},
    {"robot", "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, 
     &(config->front_laser_config.angular_offset), 1, NULL},
    {"laser", frontlaser_fov_string, CARMEN_PARAM_DOUBLE,
     &(config->front_laser_config.fov), 0, NULL}, 
    {"laser", frontlaser_res_string, CARMEN_PARAM_DOUBLE,
     &(config->front_laser_config.angular_resolution), 0, NULL}, 
    {"simulator", "laser_probability_of_random_max",
     CARMEN_PARAM_DOUBLE, 
     &(config->front_laser_config.prob_of_random_max), 1, NULL},
    {"simulator", "laser_probability_of_random_reading",
     CARMEN_PARAM_DOUBLE, 
     &(config->front_laser_config.prob_of_random_reading), 1, NULL},
    {"simulator", "laser_sensor_variance", CARMEN_PARAM_DOUBLE, 
     &(config->front_laser_config.variance), 1, NULL}};

  carmen_param_t param_list_rear_laser[] = {
    {"simulator", "rearlaser_maxrange", CARMEN_PARAM_DOUBLE, 
     &(config->rear_laser_config.max_range), 1, NULL},
    {"robot", "rearlaser_offset", CARMEN_PARAM_DOUBLE, 
     &(config->rear_laser_config.offset), 1, NULL},
    {"robot", "rearlaser_side_offset", CARMEN_PARAM_DOUBLE, 
     &(config->rear_laser_config.side_offset), 1, NULL},
    {"robot", "rearlaser_angular_offset", CARMEN_PARAM_DOUBLE, 
     &(config->rear_laser_config.angular_offset), 1, NULL},
    {"laser", rearlaser_fov_string, CARMEN_PARAM_DOUBLE,
     &(config->rear_laser_config.fov), 0, NULL}, 
    {"laser", rearlaser_res_string, CARMEN_PARAM_DOUBLE,
     &(config->rear_laser_config.angular_resolution), 0, NULL}, 
    {"simulator", "laser_probability_of_random_max",
     CARMEN_PARAM_DOUBLE, 
     &(config->rear_laser_config.prob_of_random_max), 1, NULL},
    {"simulator", "laser_probability_of_random_reading",
     CARMEN_PARAM_DOUBLE,
     &(config->rear_laser_config.prob_of_random_reading), 1, NULL},
    {"simulator", "laser_sensor_variance", CARMEN_PARAM_DOUBLE, 
     &(config->rear_laser_config.variance), 1, NULL}};

  carmen_param_t param_list_sonar[] = {
    {"robot", "num_sonars", CARMEN_PARAM_INT, 
     &(config->sonar_config.num_sonars), 1, NULL},
    {"robot", "sensor_angle", CARMEN_PARAM_DOUBLE, 
     &(config->sonar_config.sensor_angle), 1, NULL},
    {"robot", "max_sonar", CARMEN_PARAM_DOUBLE, 
     &(config->sonar_config.max_range), 1, NULL},
    {"robot", "sonar_offsets", CARMEN_PARAM_STRING, 
     &offset_string, 1, NULL},
    {"simulator", "sonar_probability_of_random_max",
     CARMEN_PARAM_DOUBLE,
     &(config->sonar_config.prob_of_random_max), 1, NULL},
    {"simulator", "sonar_probability_of_random_reading",
     CARMEN_PARAM_DOUBLE, 
     &(config->sonar_config.prob_of_random_reading), 1, NULL},
    {"simulator", "sonar_sensor_variance", CARMEN_PARAM_DOUBLE, 
     &(config->sonar_config.variance), 1, NULL} };
  

  if(config->use_front_laser) {
    num_items = sizeof(param_list_front_laser)/
      sizeof(param_list_front_laser[0]);
    carmen_param_install_params(argc, argv, param_list_front_laser, 
				num_items);
    config->front_laser_config.angular_resolution =
      carmen_degrees_to_radians(config->front_laser_config.angular_resolution);

    config->front_laser_config.fov =
      carmen_degrees_to_radians(config->front_laser_config.fov);
  }

  if(config->use_rear_laser) {
    num_items = sizeof(param_list_rear_laser)/
      sizeof(param_list_rear_laser[0]);
    carmen_param_install_params(argc, argv, param_list_rear_laser, 
				num_items);
    config->rear_laser_config.angular_resolution =
      carmen_degrees_to_radians(config->rear_laser_config.angular_resolution);

    config->rear_laser_config.fov =
      carmen_degrees_to_radians(config->rear_laser_config.fov);
  }

  /* from now on all angles in the config struct are in radians */
  
  if(config->use_sonar) {
    num_items = sizeof(param_list_sonar)/sizeof(param_list_sonar[0]);
    carmen_param_install_params(argc, argv, param_list_sonar, num_items);
    
    config->sonar_config.offsets =
      (carmen_point_p)calloc(config->sonar_config.num_sonars, 
			     sizeof(carmen_point_t));
    carmen_test_alloc(config->sonar_config.offsets);
    
    carmen_parse_sonar_offsets
      (offset_string, config->sonar_config.offsets,
       config->sonar_config.num_sonars);
  }
  
#ifndef OLD_MOTION_MODEL
  config->motion_model = carmen_localize_motion_initialize(argc, argv);
#endif

  fill_laser_config_data( &(config->front_laser_config) );

  if(config->use_rear_laser)
    fill_laser_config_data( &(config->rear_laser_config));
  
}


int main(int argc, char** argv)
{
  carmen_simulator_config_t simulator_conf;
  carmen_localize_initialize_message init_msg;

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  memset(&simulator_conf, 0, sizeof(carmen_simulator_config_t));
  memset(&init_msg, 0, sizeof(carmen_localize_initialize_message));
  simulator_config = &simulator_conf;
  read_parameters(argc, argv, &simulator_conf);
  carmen_simulator_initialize_object_model(argc, argv);

  signal(SIGINT, shutdown_module);

  if (initialize_ipc() < 0)
    carmen_die("Error in initializing ipc\n");

  carmen_randomize(&argc, &argv);

  IPC_subscribe(CARMEN_BASE_VELOCITY_NAME, velocity_handler, NULL);
  IPC_setMsgQueueLength(CARMEN_BASE_VELOCITY_NAME, 1);
  carmen_localize_subscribe_initialize_message
    (&init_msg, (carmen_handler_t)init_handler, CARMEN_SUBSCRIBE_LATEST);
  if (carmen_map_get_gridmap(&(simulator_conf.map)))
    carmen_die("%s\tCould not get a map -- did you remember to "
	       "start the mapserver,\nor give a map to the paramServer? %s\n",
	       carmen_red_code, carmen_normal_code);

  carmen_map_subscribe_gridmap_update_message
    (NULL, (carmen_handler_t)map_update_handler, CARMEN_SUBSCRIBE_LATEST);

  if (use_robot) 
    carmen_robot_start(argc, argv);

  while (1) {
    carmen_ipc_sleep(simulator_conf.real_time);
    if (!simulator_conf.sync_mode) {
      if (use_robot)
	carmen_robot_run();
      publish_readings();
    }
  }

  carmen_simulator_clear_objects();

  carmen_ipc_disconnect();

  exit(0);
}

