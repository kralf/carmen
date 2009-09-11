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
#ifndef NO_ZLIB
#include <zlib.h>
#endif

#include "global.h"
#include "localize_core.h"

#include "param_interface.h"

/* global variables */
carmen_localize_map_t map;
carmen_map_placelist_t placelist;
carmen_localize_particle_filter_p filter;
carmen_localize_summary_t summary;

carmen_robot_laser_message front_laser;

/* publish a global position message */

void publish_globalpos(carmen_localize_summary_p summary)
{
  static carmen_localize_globalpos_message globalpos;
  IPC_RETURN_TYPE err;
  
  globalpos.timestamp = carmen_get_time();
  globalpos.host = carmen_get_host();
  globalpos.globalpos = summary->mean;
  globalpos.globalpos_std = summary->std;
  globalpos.globalpos_xy_cov = summary->xy_cov;
  globalpos.odometrypos = summary->odometry_pos;
  globalpos.converged = summary->converged;
  err = IPC_publishData(CARMEN_LOCALIZE_GLOBALPOS_NAME, &globalpos);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_LOCALIZE_GLOBALPOS_NAME);  
}

/* publish a particle message */

void publish_particles(carmen_localize_particle_filter_p filter, 
		       carmen_localize_summary_p summary)
{
  static carmen_localize_particle_message pmsg;
  IPC_RETURN_TYPE err;

  pmsg.timestamp = carmen_get_time();
  pmsg.host = carmen_get_host();
  pmsg.globalpos = summary->mean;
  pmsg.globalpos_std = summary->mean;
  pmsg.num_particles = filter->param->num_particles;
  pmsg.particles = (carmen_localize_particle_ipc_p)filter->particles;
  err = IPC_publishData(CARMEN_LOCALIZE_PARTICLE_NAME, &pmsg);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_LOCALIZE_PARTICLE_NAME);  
  fprintf(stderr, "P");
}

/* publish sensor message */

void publish_sensor(carmen_localize_particle_filter_p filter,
		    carmen_localize_summary_p summary,
		    carmen_robot_laser_message *laser, int front)
{
  static carmen_localize_sensor_message sensor;
  IPC_RETURN_TYPE err;

  sensor.timestamp = carmen_get_time();
  sensor.host = carmen_get_host();
  if(front) {
    sensor.pose.x = summary->mean.x + filter->param->front_laser_offset *
      cos(summary->mean.theta);
    sensor.pose.y = summary->mean.y + filter->param->front_laser_offset *
      sin(summary->mean.theta);
    sensor.pose.theta = summary->mean.theta;
    sensor.num_laser = 1;
  }
  else {
    sensor.pose.x = summary->mean.x + filter->param->rear_laser_offset *
      cos(summary->mean.theta + M_PI);
    sensor.pose.y = summary->mean.y + filter->param->rear_laser_offset *
      sin(summary->mean.theta + M_PI);
    sensor.pose.theta = summary->mean.theta + M_PI;
    sensor.num_laser = 2;
  }
  sensor.num_readings = laser->num_readings;
  sensor.laser_skip = filter->param->laser_skip;
  sensor.config = laser->config;
  sensor.range = laser->range;
  sensor.mask = filter->laser_mask;
  err = IPC_publishData(CARMEN_LOCALIZE_SENSOR_NAME, &sensor);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_LOCALIZE_SENSOR_NAME);  
}

/* process initialization messages */

void carmen_localize_initialize_handler(carmen_localize_initialize_message 
					*initialize_msg)
{
  if(initialize_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN)
    carmen_localize_initialize_particles_gaussians(filter,
						   initialize_msg->num_modes,
						   initialize_msg->mean,
						   initialize_msg->std);
  else if(initialize_msg->distribution == CARMEN_INITIALIZE_UNIFORM) {
    carmen_localize_initialize_particles_uniform(filter, &front_laser, &map);
    publish_particles(filter, &summary);
  }
}

void carmen_localize_initialize_placename_handler(carmen_localize_initialize_placename_message *init_place)
{
  carmen_localize_initialize_particles_placename(filter, &placelist,
						 init_place->placename);
  publish_particles(filter, &summary);
}

void robot_frontlaser_handler(carmen_robot_laser_message *flaser)
{
  fprintf(stderr, "F");
  carmen_localize_run(filter, &map, flaser, 
	       filter->param->front_laser_offset, 0);
  if(filter->initialized) {
    carmen_localize_summarize(filter, &summary, &map, flaser->num_readings, 
			      flaser->range, filter->param->front_laser_offset,
			      flaser->config.angular_resolution,
			      flaser->config.start_angle, 0);
    publish_globalpos(&summary);
    publish_particles(filter, &summary);
    publish_sensor(filter, &summary, flaser, 1);
  }
}

void robot_rearlaser_handler(carmen_robot_laser_message *rlaser)
{
  rlaser = rlaser;
}

void 
map_update_handler(carmen_map_t *new_map) 
{
  int i;
  carmen_localize_param_p param;

  param = filter->param;

  for(i = 0; i < filter->param->num_particles; i++) 
    free(filter->temp_weights[i]);  
  free(filter->temp_weights);
  free(filter->particles);
  free(filter);

  free(map.complete_x_offset);
  free(map.complete_y_offset);
  free(map.complete_distance);
  free(map.complete_prob);
  free(map.complete_gprob);
  free(map.x_offset);
  free(map.y_offset);
  free(map.distance);
  free(map.prob);
  free(map.gprob);

  filter = carmen_localize_particle_filter_new(param);

  if (placelist.num_places > 0)
    free(placelist.places);

  if(carmen_map_get_placelist(&placelist) < 0)
    carmen_die("Could not get placelist from the map server.\n");

  /* create a localize map */
  carmen_warn("Creating likelihood maps... ");
  carmen_to_localize_map(new_map, &map, param);
  carmen_warn("done.\n");
}

static void
globalpos_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
			void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  carmen_localize_globalpos_message globalpos;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_freeByteArray(callData);

  globalpos.timestamp = carmen_get_time();
  globalpos.host = carmen_get_host();
  globalpos.globalpos = summary.mean;
  globalpos.globalpos_std = summary.std;
  globalpos.globalpos_xy_cov = summary.xy_cov;
  globalpos.odometrypos = summary.odometry_pos;
  globalpos.converged = summary.converged;

  err = IPC_respondData(msgRef, CARMEN_LOCALIZE_GLOBALPOS_NAME, &globalpos);
  carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_GLOBALPOS_NAME);
}

static void map_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			      void *clientData __attribute__ ((unused))) 
{
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;
  carmen_localize_map_query_message msg;
  carmen_localize_map_message response;

#ifndef NO_ZLIB
  unsigned long compress_buf_size;
  int compress_return;
  unsigned char *compressed_map;
#endif

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg, 
			   sizeof(carmen_localize_map_query_message));
  IPC_freeByteArray(callData);
  
  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));

  response.config = map.config;

  if (msg.map_is_global_likelihood) {
    response.map_is_global_likelihood = 1;
    response.data = (unsigned char *)map.complete_gprob;
    response.size = map.config.x_size*map.config.y_size*sizeof(float);
  } else {
    response.map_is_global_likelihood = 0;
    response.data = (unsigned char *)map.complete_prob;
    response.size = map.config.x_size*map.config.y_size*sizeof(float);
  }

#ifndef NO_ZLIB
  compress_buf_size = response.size*1.01+12;
  compressed_map = (unsigned char *)
    calloc(compress_buf_size, sizeof(unsigned char));
  carmen_test_alloc(compressed_map);
  compress_return = compress((void *)compressed_map, 
			     (unsigned long *)&compress_buf_size,
			     (void *)response.data, 
			     (unsigned long)response.size);
  if (compress_return != Z_OK) {
    free(compressed_map);
    response.compressed = 0;
  } else {
    response.size = compress_buf_size;
    response.data = compressed_map;
    response.compressed = 1;
  }
#else
  response.compressed = 0;
#endif

  response.timestamp = carmen_get_time();
  response.host = carmen_get_host();
  
  err = IPC_respondData(msgRef, CARMEN_LOCALIZE_MAP_NAME, &response);
  carmen_test_ipc(err, "Could not respond", CARMEN_LOCALIZE_MAP_NAME);
}

/* declare all IPC messages */

int register_ipc_messages(void)
{
  IPC_RETURN_TYPE err;

  /* register globalpos message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_LOCALIZE_GLOBALPOS_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_GLOBALPOS_NAME);

  /* register robot particle message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_PARTICLE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_LOCALIZE_PARTICLE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_PARTICLE_NAME);

  /* register sensor message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_SENSOR_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_SENSOR_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_SENSOR_NAME);

  /* register initialize message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_INITIALIZE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_LOCALIZE_INITIALIZE_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_LOCALIZE_INITIALIZE_NAME);

  /* subscribe to initialization messages */
  carmen_localize_subscribe_initialize_message(NULL, 
					       (carmen_handler_t)
					       carmen_localize_initialize_handler,
					       CARMEN_SUBSCRIBE_LATEST);

  carmen_localize_subscribe_initialize_placename_message(NULL, 
							 (carmen_handler_t)
							 carmen_localize_initialize_placename_handler,
							 CARMEN_SUBSCRIBE_LATEST);

  /* register map request message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_MAP_QUERY_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_MAP_QUERY_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_LOCALIZE_MAP_QUERY_NAME);

  err = IPC_defineMsg(CARMEN_LOCALIZE_MAP_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_MAP_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_MAP_NAME);

  /* subscribe to map request message */
  err = IPC_subscribe(CARMEN_LOCALIZE_MAP_QUERY_NAME, map_query_handler, NULL);
  carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_MAP_QUERY_NAME);
  IPC_setMsgQueueLength(CARMEN_LOCALIZE_MAP_QUERY_NAME, 1);


  /* register globalpos request message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME, 
		      IPC_VARIABLE_LENGTH,
		      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_LOCALIZE_MAP_QUERY_NAME);

  /* subscribe to globalpos request message */
  err = IPC_subscribe(CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME, 
		      globalpos_query_handler, NULL);
  carmen_test_ipc(err, "Could not subscribe", 
		  CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME);
  IPC_setMsgQueueLength(CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME, 1);

  return 0;
}

/* read all parameters from .ini file and command line. */

void read_parameters(int argc, char **argv, carmen_localize_param_p param)
{
  double integrate_angle_deg;
  integrate_angle_deg=1.0;

  carmen_param_t param_list[] = {
    {"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE, 
     &param->front_laser_offset, 0, NULL},
    {"robot", "rearlaser_offset", CARMEN_PARAM_DOUBLE, 
     &param->rear_laser_offset, 0, NULL},
    {"localize", "use_rear_laser", CARMEN_PARAM_ONOFF, 
     &param->use_rear_laser, 0, NULL},
    {"localize", "num_particles", CARMEN_PARAM_INT, 
     &param->num_particles, 0, NULL},
    {"localize", "laser_max_range", CARMEN_PARAM_DOUBLE, &param->max_range, 1, NULL},
    {"localize", "min_wall_prob", CARMEN_PARAM_DOUBLE, 
     &param->min_wall_prob, 0, NULL},
    {"localize", "outlier_fraction", CARMEN_PARAM_DOUBLE, 
     &param->outlier_fraction, 0, NULL},
    {"localize", "update_distance", CARMEN_PARAM_DOUBLE, 
     &param->update_distance, 0, NULL},
    {"localize", "integrate_angle_deg", CARMEN_PARAM_DOUBLE,
     &integrate_angle_deg, 0, NULL},
    {"localize", "do_scanmatching", CARMEN_PARAM_ONOFF,
     &param->do_scanmatching, 1, NULL},
    {"localize", "constrain_to_map", CARMEN_PARAM_ONOFF, 
     &param->constrain_to_map, 1, NULL},
#ifdef OLD_MOTION_MODEL
    {"localize", "odom_a1", CARMEN_PARAM_DOUBLE, &param->odom_a1, 1, NULL},
    {"localize", "odom_a2", CARMEN_PARAM_DOUBLE, &param->odom_a2, 1, NULL},
    {"localize", "odom_a3", CARMEN_PARAM_DOUBLE, &param->odom_a3, 1, NULL},
    {"localize", "odom_a4", CARMEN_PARAM_DOUBLE, &param->odom_a4, 1, NULL},
#endif
    {"localize", "occupied_prob", CARMEN_PARAM_DOUBLE, 
     &param->occupied_prob, 0, NULL},
    {"localize", "lmap_std", CARMEN_PARAM_DOUBLE, 
     &param->lmap_std, 0, NULL},
    {"localize", "global_lmap_std", CARMEN_PARAM_DOUBLE, 
     &param->global_lmap_std, 0, NULL},
    {"localize", "global_evidence_weight", CARMEN_PARAM_DOUBLE, 
     &param->global_evidence_weight, 0, NULL},
    {"localize", "global_distance_threshold", CARMEN_PARAM_DOUBLE, 
     &param->global_distance_threshold, 1, NULL},
    {"localize", "global_test_samples", CARMEN_PARAM_INT,
     &param->global_test_samples, 1, NULL},
    {"localize", "use_sensor", CARMEN_PARAM_ONOFF,
     &param->use_sensor, 0, NULL},
    {"localize", "tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE, 
     &param->tracking_beam_minlikelihood, 0, NULL},
    {"localize", "global_beam_minlikelihood", CARMEN_PARAM_DOUBLE, 
     &param->global_beam_minlikelihood, 0, NULL}
  };

  carmen_param_install_params(argc, argv, param_list, 
			      sizeof(param_list) / sizeof(param_list[0]));

  param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);

}

/* create localize specific maps */

void create_localize_map(carmen_localize_map_t *map, 
			 carmen_localize_param_p param)
{
  carmen_map_t raw_map;
  
  /* Request map, and wait for it to arrive */
  carmen_warn("Requesting a map from the map server... ");
  if(carmen_map_get_gridmap(&raw_map) < 0) 
    carmen_die("Could not get a map from the map server.\n");
  else
    carmen_warn("done.\n");
  if(carmen_map_get_placelist(&placelist) < 0)
    carmen_die("Could not get placelist from the map server.\n");

  /* create a localize map */
  carmen_warn("Creating likelihood maps... ");
  carmen_to_localize_map(&raw_map, map, param);
  carmen_warn("done.\n");
}

void shutdown_localize(int x)
{
  if(x == SIGINT) {
    carmen_verbose("Disconnecting from IPC network.\n");
    exit(1);
  }
}

int main(int argc, char **argv) 
{ 
  carmen_localize_param_t param;

  /* initialize carmen */
  carmen_randomize(&argc, &argv);
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);
  
  /* Setup exit handler */
  signal(SIGINT, shutdown_localize);

  /* register localize related messages */
  register_ipc_messages();

  /* Initialize all the relevant parameters */
  read_parameters(argc, argv, &param);

  /* get a map */
  create_localize_map(&map, &param);

#ifndef OLD_MOTION_MODEL
  param.motion_model = carmen_localize_motion_initialize(argc, argv);
#endif

  /* Allocate memory for the particle filter */
  filter = carmen_localize_particle_filter_new(&param);

  /* Subscribe to front and rear laser messages */
  carmen_robot_subscribe_frontlaser_message(&front_laser, (carmen_handler_t)
					    robot_frontlaser_handler,
					    CARMEN_SUBSCRIBE_LATEST);
  if(filter->param->use_rear_laser)
    carmen_robot_subscribe_rearlaser_message(NULL, (carmen_handler_t)
					     robot_rearlaser_handler,
					     CARMEN_SUBSCRIBE_LATEST);

  carmen_map_subscribe_gridmap_update_message(NULL, (carmen_handler_t)
					      map_update_handler,
					      CARMEN_SUBSCRIBE_LATEST);

  /* Loop forever */
  carmen_ipc_dispatch();
  return 0;
}
