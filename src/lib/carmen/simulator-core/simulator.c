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

/*******************************************
 * library with the functions for the guts *
 * of the simulator                        *
 *******************************************/
#include "global.h"
#include "simulator.h"
#include "objects.h"

/* updates x without inaccuracies */
static carmen_inline double 
updatex(carmen_simulator_config_t *simulator_config)
{
  double tvx;
  tvx = (double)(simulator_config->tv) * 
    cos(simulator_config->odom_pose.theta);
  return simulator_config->delta_t * (tvx);
}

/* updates y withou inaccuracies */
static carmen_inline double 
updatey(carmen_simulator_config_t *simulator_config)
{
  double tvy;
  tvy = (double)(simulator_config->tv) * 
    sin(simulator_config->odom_pose.theta);
  return simulator_config->delta_t * (tvy);
}

/* updates theta without inaccuracies */
static carmen_inline double 
updatetheta(carmen_simulator_config_t *simulator_config)
{
  return simulator_config->delta_t * (double)(simulator_config->rv);
}

/* recalculates the actual position */
void 
carmen_simulator_recalc_pos(carmen_simulator_config_t *simulator_config)
{
  carmen_point_t old_odom;
  carmen_point_t new_odom;
  carmen_point_t new_true;

  double distance, radius, centre_x, centre_y, delta_angle;
  int backwards;
  double dx, dy, odom_theta;
  double delta_t, delta_theta;
#ifndef OLD_MOTION_MODEL
  double downrange, crossrange, turn;
  carmen_localize_motion_model_t *model;
#else
  double dr1, dr2;
  double dhatr1, dhatt, dhatr2;
  double std_r1, std_r2, std_t;
#endif

  int map_x, map_y;
  double tv, rv;
  carmen_map_p map;
  double acceleration, rot_acceleration;
  double delta_v;

  old_odom = simulator_config->odom_pose;
  new_odom = old_odom;
  new_true = simulator_config->true_pose;

  acceleration = simulator_config->acceleration;
  delta_v = simulator_config->target_tv - simulator_config->tv;
  delta_t = fabs(delta_v / acceleration);
  if (delta_t > simulator_config->delta_t) 
    {
      if (delta_v < 0) 
	simulator_config->tv -= acceleration*simulator_config->delta_t;
      else
	simulator_config->tv += acceleration*simulator_config->delta_t;
    } 
  else
    simulator_config->tv = simulator_config->target_tv;

  rot_acceleration = acceleration/simulator_config->width;
  delta_v = simulator_config->target_rv - simulator_config->rv;
  delta_t = fabs(delta_v / rot_acceleration);
  if (delta_t > simulator_config->delta_t) 
    {
      if (delta_v < 0) 
	simulator_config->rv -= rot_acceleration*simulator_config->delta_t;
      else
	simulator_config->rv += rot_acceleration*simulator_config->delta_t;
    } 
  else
    simulator_config->rv = simulator_config->target_rv;

  tv = simulator_config->tv;
  rv = simulator_config->rv;

  /* Not translating or rotating */
  if (fabs(tv) < 1e-3 && fabs(rv) < 1e-3)
    return;

  /* Rotating but not translating */
  if (fabs(tv) < 1e-3 && fabs(rv) >= 1e-3) 
    new_odom.theta += rv * simulator_config->delta_t;
  /* Translating but not rotating */
  else if (fabs(tv) >= 1e-3 && fabs(rv) < 1e-3) {
    new_odom.x += tv * simulator_config->delta_t * cos(new_odom.theta);
    new_odom.y += tv * simulator_config->delta_t * sin(new_odom.theta);
  /* Translating and also rotating */
  } else {
    distance = simulator_config->delta_t * tv;
    radius = tv / rv;
    centre_x = new_odom.x + radius * cos(new_odom.theta+M_PI/2);
    centre_y = new_odom.y + radius * sin(new_odom.theta+M_PI/2);
    delta_angle = distance / radius;
    new_odom.theta += delta_angle;
    new_odom.x = centre_x + radius * cos(new_odom.theta-M_PI/2);
    new_odom.y = centre_y + radius * sin(new_odom.theta-M_PI/2);
  }

  dx = new_odom.x - old_odom.x;
  dy = new_odom.y - old_odom.y;

  delta_t = sqrt(dx * dx + dy * dy);
  delta_theta = carmen_normalize_theta(new_odom.theta - old_odom.theta);

  odom_theta = atan2(dy, dx);
  backwards = (dx * cos(new_odom.theta) + dy * sin(new_odom.theta) < 0);

#ifndef OLD_MOTION_MODEL  
  model = simulator_config->motion_model;
  downrange = 
    carmen_localize_sample_noisy_downrange(delta_t, delta_theta, model);
  crossrange = 
    carmen_localize_sample_noisy_crossrange(delta_t, delta_theta, model);
  turn = carmen_localize_sample_noisy_turn(delta_t, delta_theta, model);

  if(backwards) {
    new_true.x -= downrange * cos(new_true.theta + turn/2.0) + 
      crossrange * cos(new_true.theta + turn/2.0 + M_PI/2.0);
    new_true.y -= downrange * sin(new_true.theta + turn/2.0) + 
      crossrange * sin(new_true.theta + turn/2.0 + M_PI/2.0);
  }
  else {
    new_true.x += downrange * cos(new_true.theta + turn/2.0) + 
      crossrange * cos(new_true.theta + turn/2.0 + M_PI/2.0);
    new_true.y += downrange * sin(new_true.theta + turn/2.0) + 
      crossrange * sin(new_true.theta + turn/2.0 + M_PI/2.0);
  }
  new_true.theta = carmen_normalize_theta(new_true.theta+turn);

#else
 /* The dr1/dr2 code becomes unstable if dt is too small. */
  if(delta_t < 0.05) {
    dr1 = carmen_normalize_theta(new_odom.theta - old_odom.theta) / 2.0;
    dr2 = dr1;
  }
  else {
    if(backwards)
      dr1 = carmen_normalize_theta
	(atan2(old_odom.y - new_odom.y, old_odom.x - new_odom.x) -
	 old_odom.theta);
    else
      dr1 = carmen_normalize_theta
	(atan2(new_odom.y - old_odom.y, new_odom.x - old_odom.x) -
	 old_odom.theta);
    dr2 = carmen_normalize_theta(new_odom.theta - old_odom.theta - dr1);
  }

  /* compute motion model parameters */
  std_r1 = simulator_config->odom_a1 * fabs(dr1) + 
    simulator_config->odom_a2 * delta_t;
  std_t = simulator_config->odom_a3 * delta_t + 
    simulator_config->odom_a4 * fabs(dr1 + dr2);
  std_r2 = simulator_config->odom_a1 * fabs(dr2) + 
    simulator_config->odom_a2 * delta_t;

  dhatr1 = carmen_gaussian_random(dr1, std_r1);
  dhatt = carmen_gaussian_random(delta_t, std_t);
  dhatr2 = carmen_gaussian_random(dr2, std_r2);
    
  if(backwards) {
    new_true.x -= dhatt * cos(new_true.theta + dhatr1);
    new_true.y -= dhatt * sin(new_true.theta + dhatr1);
  }
  else {
    new_true.x += dhatt * cos(new_true.theta + dhatr1);
    new_true.y += dhatt * sin(new_true.theta + dhatr1);
  }
  new_true.theta = carmen_normalize_theta(new_true.theta+dhatr1+dhatr2);
#endif
    
  map = &(simulator_config->map);
  map_x = new_true.x / map->config.resolution;
  map_y = new_true.y / map->config.resolution;

  if(map_x < 0 || map_x >= map->config.x_size || 
     map_y < 0 || map_y >= map->config.y_size ||
     map->map[map_x][map_y] > .15 ||
     carmen_simulator_object_too_close(new_true.x, new_true.y, -1))
    return;
  
  new_odom.theta = carmen_normalize_theta(new_odom.theta);
  simulator_config->odom_pose = new_odom;
  simulator_config->true_pose = new_true;
}

/* adds error to a sonar scan */
static void
add_sonar_error(carmen_base_sonar_message * base_sonar, 
		carmen_simulator_sonar_config_t *sonar_config)
{
  int i;
  for(i=0;i<base_sonar->num_sonars; i++)
    {
      if(base_sonar->range[i] > sonar_config->max_range)
	base_sonar->range[i] = sonar_config->max_range;
      else if (carmen_uniform_random(0, 1.0) < 
	       sonar_config->prob_of_random_max)
	base_sonar->range[i] = sonar_config->max_range;
      else if (carmen_uniform_random(0, 1.0) < 
	       sonar_config->prob_of_random_reading)
	base_sonar->range[i] = carmen_uniform_random
	  (0, sonar_config->max_range);
      else
	base_sonar->range[i] += carmen_gaussian_random
	  (0.0, sonar_config->variance);
    }
}

/* adds error to a laser scan */
static void 
add_laser_error(carmen_laser_laser_message * laser, 
		carmen_simulator_laser_config_t *laser_config)
{
  int i;
  for(i = 0; i < laser_config->num_lasers; i ++)
    {
      if (laser->range[i] > laser_config->max_range)
	laser->range[i] = laser_config->max_range;
      else if (carmen_uniform_random(0, 1.0) < 
	       laser_config->prob_of_random_max)
	laser->range[i] = laser_config->max_range;
      else if(carmen_uniform_random(0, 1.0) < 
	      laser_config->prob_of_random_reading)
	laser->range[i] = carmen_uniform_random(0, laser_config->num_lasers);
      else 
	laser->range[i] += 
	  carmen_gaussian_random(0.0, laser_config->variance);
    }
}

/* calculates a sonar message based upon the current position*/
void
carmen_simulator_calc_sonar_msg(carmen_base_sonar_message *sonar,
				carmen_simulator_config_t *simulator_config)
{
  carmen_traj_point_t point;

  point.x=simulator_config->true_pose.x;
  point.y=simulator_config->true_pose.y;
  point.theta=simulator_config->true_pose.theta;
  point.t_vel=simulator_config->tv;
  point.r_vel=simulator_config->rv;
 
  carmen_geometry_generate_sonar_data(sonar->range, &point, 
				      simulator_config->sonar_config.offsets, 
				      simulator_config->sonar_config.
				      num_sonars, 
				      &(simulator_config->map));
  carmen_simulator_add_objects_to_sonar(sonar, simulator_config);
  add_sonar_error(sonar, &(simulator_config->sonar_config));
}

/*calculates a laser message based upon the current position*/
void
carmen_simulator_calc_laser_msg(carmen_laser_laser_message *laser, 
				carmen_simulator_config_p simulator_config,
				int is_rear)
{
  carmen_traj_point_t point;
  carmen_simulator_laser_config_t *laser_config = NULL;

 
  if (is_rear) {
    laser_config = &(simulator_config->rear_laser_config);
  } 
  else  {
    laser_config = &(simulator_config->front_laser_config);
  }

  laser->id = laser_config->id; 

  point.x = simulator_config->true_pose.x 
    + laser_config->offset *  cos(simulator_config->true_pose.theta) 
    - laser_config->side_offset *  sin(simulator_config->true_pose.theta) ;

  point.y = simulator_config->true_pose.y 
    + laser_config->offset * sin(simulator_config->true_pose.theta)
    + laser_config->side_offset * cos(simulator_config->true_pose.theta);

  point.theta = carmen_normalize_theta(simulator_config->true_pose.theta +
				       laser_config->angular_offset);
				       

  point.t_vel = simulator_config->tv;
  point.r_vel = simulator_config->rv;

  laser->num_readings = laser_config->num_lasers;


  laser->config.maximum_range       = laser_config->max_range;
  laser->config.fov                 = laser_config->fov;
  laser->config.start_angle         = laser_config->start_angle;
  laser->config.angular_resolution  = laser_config->angular_resolution;
  laser->config.laser_type          = SIMULATED_LASER;
  laser->config.accuracy            = laser_config->variance; 

  //this was placed here because compiling with the old motion model
  //did't work, check this if this breaks something
  laser->config.remission_mode      = REMISSION_NONE;



  carmen_geometry_generate_laser_data(laser->range, &point, laser->config.start_angle, 
				      laser->config.start_angle+laser->config.fov, 
				      laser_config->num_lasers, 
				      &(simulator_config->map));

  carmen_simulator_add_objects_to_laser(laser, simulator_config, is_rear);

  add_laser_error(laser, laser_config);
}
