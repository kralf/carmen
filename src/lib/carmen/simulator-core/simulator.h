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

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "simulator_messages.h"
#include "laser_messages.h"
#include "base_messages.h"

#include "localize_motion.h"
#include "map.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double max_range;
  double variance;
  double prob_of_random_reading;
  double prob_of_random_max;
  int num_lasers;
  double offset;
  double side_offset;
  double angular_offset;
  double angular_resolution;
  double fov;
  double start_angle;
  int id;
} carmen_simulator_laser_config_t;

typedef struct {
  int num_sonars;
  double max_range;
  double sensor_angle;
  carmen_point_t *offsets;
  double variance;
  double prob_of_random_reading;
  double prob_of_random_max;
} carmen_simulator_sonar_config_t;

typedef struct {
  carmen_simulator_laser_config_t front_laser_config;
  carmen_simulator_laser_config_t rear_laser_config;
  carmen_simulator_sonar_config_t sonar_config;
  int use_sonar;
  int use_front_laser, use_rear_laser;

#ifndef OLD_MOTION_MODEL
  carmen_localize_motion_model_t *motion_model;
#else
  double odom_a1;
  double odom_a2;
  double odom_a3;
  double odom_a4;
#endif

  carmen_map_t map;
  carmen_point_t odom_pose;
  carmen_point_t true_pose;
  
  double tv;
  double rv;
  double acceleration;
  double target_tv;
  double target_rv;
  double width;

  double delta_t;
  double real_time;
  int sync_mode;
  double motion_timeout;
  double time_of_last_command;
} carmen_simulator_config_t, *carmen_simulator_config_p;

/* recalculates the actual position */

void carmen_simulator_recalc_pos(carmen_simulator_config_t *simulator_config);

/* produce a laser message based upon the current
   position */

void carmen_simulator_calc_laser_msg(carmen_laser_laser_message *flaser,
             carmen_simulator_config_t 
             *simulator_config, int is_rear_laser);

void carmen_simulator_calc_sonar_msg(carmen_base_sonar_message *sonar,
             carmen_simulator_config_t 
             *simulator_config);

#ifdef __cplusplus
}
#endif

#endif
