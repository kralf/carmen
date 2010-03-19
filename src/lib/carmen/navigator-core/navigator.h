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

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "global.h"

#include "localize_messages.h"

typedef struct {
  int num_lasers_to_use;
  int use_fast_laser;
  double map_update_radius;
  int map_update_obstacles;
  int map_update_freespace;
  double replan_frequency;
  int smooth_path;
  double waypoint_tolerance;
  double goal_size;
  double goal_theta_tolerance;
  int dont_integrate_odometry;
  int plan_to_nearest_free_point;
} carmen_navigator_config_t;

extern carmen_map_p nav_map;
extern carmen_map_placelist_t placelist;
extern carmen_robot_config_t robot_config;
extern carmen_navigator_config_t nav_config;

extern int cheat;
extern int autonomous_status;

extern carmen_traj_point_t robot_position;

void carmen_navigator_goal_triplet(carmen_point_p point);
void carmen_navigator_goal(double x, double y);
int carmen_navigator_goal_place(char *name);
void carmen_navigator_set_max_velocity(double vel);
carmen_map_placelist_p carmen_navigator_get_places(void);
int carmen_navigator_autonomous_status(void);

void carmen_navigator_start_autonomous(void);
void carmen_navigator_stop_autonomous(void);

void generate_next_motion_command(void);

typedef struct {
  double tv;
  double rv;
} command_t;

#ifdef __cplusplus
}
#endif

#endif
