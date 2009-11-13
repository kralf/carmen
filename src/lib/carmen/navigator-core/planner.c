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

#include <assert.h>
#include <float.h>

#include "global.h"

#include "planner.h"
#include "navigator.h"
#include "conventional.h"
#include "trajectory.h"
#include "map_io.h"
#include "map_modify.h"

#define NUM_ACTIONS 8

int carmen_planner_x_offset[NUM_ACTIONS] = {0, 1, 1, 1, 0, -1, -1, -1};
int carmen_planner_y_offset[NUM_ACTIONS] = {-1, -1, 0, 1, 1, 1, 0, -1};

carmen_map_t *carmen_planner_map = NULL;

static carmen_map_t *true_map = NULL;

static int have_plan = 0;

static int allow_any_orientation = 0;
static carmen_point_t requested_goal;
static carmen_point_t intermediate_goal;
static int goal_is_accessible;

static carmen_traj_point_t robot;
static carmen_planner_path_t path = {NULL, 0, 0};

static int goal_set = 0;

static int
extract_path_from_value_function(void)
{
  int position;
  carmen_traj_point_t path_point;
  carmen_map_point_t cur_point, prev_point, map_goal;

  if (!have_plan)
    return -1;

  carmen_planner_util_clear_path(&path);
  carmen_planner_util_add_path_point(robot, &path);

  carmen_trajectory_to_map(&robot, &cur_point, carmen_planner_map);

  if (goal_is_accessible) {
    map_goal.x = carmen_round(requested_goal.x /
			      carmen_planner_map->config.resolution);
    map_goal.y = carmen_round(requested_goal.y /
			      carmen_planner_map->config.resolution);
    map_goal.map = carmen_planner_map;
  } else {
    map_goal.x = carmen_round(intermediate_goal.x /
			      carmen_planner_map->config.resolution);
    map_goal.y = carmen_round(intermediate_goal.y /
			      carmen_planner_map->config.resolution);
    map_goal.map = carmen_planner_map;
  }

  do {
    prev_point = cur_point;
    carmen_conventional_find_best_action(&cur_point);
    carmen_map_to_trajectory(&cur_point, &path_point);
    position = carmen_planner_util_add_path_point(path_point, &path);
    if (cur_point.x == map_goal.x && cur_point.y == map_goal.y)
      return 0;
  } while (cur_point.x != prev_point.x || cur_point.y != prev_point.y);

  return -1;
}

static void
compute_cost(int start_index, int end_index, double *cost_along_path,
	     double *min_cost)
{
  carmen_traj_point_p start_point;
  carmen_traj_point_p end_point;

  carmen_map_point_t p1;
  carmen_map_point_t p2;

  carmen_bresenham_param_t params;

  int x, y;
  double total_cost, cur_cost = 0;

  start_point = carmen_planner_util_get_path_point(start_index, &path);
  carmen_trajectory_to_map(start_point, &p1, carmen_planner_map);
  end_point = carmen_planner_util_get_path_point(end_index, &path);
  carmen_trajectory_to_map(end_point, &p2, carmen_planner_map);

  carmen_get_bresenham_parameters(p1.x, p1.y, p2.x, p2.y, &params);

  total_cost = 0;
  carmen_get_current_point(&params, &x, &y);
  *min_cost = carmen_conventional_get_cost(x, y);
  while (carmen_get_next_point(&params)) {
    carmen_get_current_point(&params, &x, &y);
    cur_cost = carmen_conventional_get_cost(x, y);
    total_cost += cur_cost;
    if (cur_cost < *min_cost)
      *min_cost = cur_cost;
  }

  *cost_along_path = total_cost;
}

static double
cost_of_path(void)
{
  int path_index;
  double cost, min_cost;
  double total_cost;

  total_cost = 0.0;
  for (path_index = 1; path_index < path.length;
       path_index++)
    {
      compute_cost(path_index-1, path_index, &cost, &min_cost);
      total_cost += cost;
    }

  return total_cost;
}


static void
smooth_path(carmen_navigator_config_t *nav_conf)
{
  int path_index;
  int new_path_counter;
  double cost_along_prev, cost_along_next;
  double min_cost_prev, min_cost_next;
  double new_cost, new_min_cost;
  double prev_cost;

  prev_cost = cost_of_path();

  new_path_counter = 0;
  path_index = 1;

  while (path.length > 2 && carmen_distance_traj
	 (&robot, carmen_planner_util_get_path_point(1, &path)) <
	 nav_conf->goal_size) {
    carmen_planner_util_delete_path_point(1, &path);
  }

  while (path_index < path.length-1)
    {
      compute_cost(path_index-1, path_index, &cost_along_prev, &min_cost_prev);
      compute_cost(path_index, path_index+1, &cost_along_next, &min_cost_next);
      compute_cost(path_index-1, path_index+1, &new_cost, &new_min_cost);

      if (cost_along_prev+cost_along_next+1e-6 < new_cost ||
	  min_cost_next < new_min_cost || min_cost_prev < new_min_cost)
	{
	  path_index++;
	}
      else
	{
	  carmen_planner_util_delete_path_point(path_index, &path);
	}
    }

  new_cost = cost_of_path();
}

static int find_nearest_free_point_to_goal(void)
{
  carmen_world_point_t goal_world;
  double *util_ptr;
  int x, y;
  double closest_free_dist;
  carmen_map_point_t closest_free;
  double dist;
  int goal_x, goal_y;

  goal_x = carmen_round(robot.x / carmen_planner_map->config.resolution);
  goal_y = carmen_round(robot.y / carmen_planner_map->config.resolution);

  carmen_conventional_dynamic_program(goal_x, goal_y);

  util_ptr = carmen_conventional_get_utility_ptr();

  if (util_ptr == NULL) {
    carmen_warn("No accessible goal.\n");
    return 0;
  }

  goal_x = carmen_round(requested_goal.x /
			carmen_planner_map->config.resolution);
  goal_y = carmen_round(requested_goal.y /
			carmen_planner_map->config.resolution);

  closest_free_dist = DBL_MAX;
  closest_free.map = carmen_planner_map;
  for (x = 0; x < carmen_planner_map->config.x_size; x++)
    for (y = 0; y < carmen_planner_map->config.y_size; y++) {
      dist = hypot(x-goal_x, y-goal_y);
      if (*util_ptr >= 0 && dist < closest_free_dist) {
	closest_free.x = x;
	closest_free.y = y;
	closest_free_dist = dist;
      }
      util_ptr++;
    }

  if (closest_free_dist > DBL_MAX/2) {
    carmen_warn("No accessible goal.\n");
    return 0;
  }

  carmen_conventional_dynamic_program(closest_free.x, closest_free.y);

  carmen_map_to_world(&closest_free, &goal_world);
  intermediate_goal.x = goal_world.pose.x;
  intermediate_goal.y = goal_world.pose.y;

  return 1;
}

static void
plan(carmen_navigator_config_t *nav_conf)
{
  static int old_goal_x, old_goal_y;
  static carmen_traj_point_t old_robot;
  static carmen_map_point_t map_pt;
  static double last_plan_time = 0;
  int goal_x, goal_y;

  if (nav_conf->replan_frequency > 0) {
    if (carmen_get_time() - last_plan_time <
	1.0/nav_conf->replan_frequency) {
      return;
    }
  }

  goal_x = carmen_round(requested_goal.x /
			carmen_planner_map->config.resolution);
  goal_y = carmen_round(requested_goal.y /
			carmen_planner_map->config.resolution);

  carmen_verbose("Doing DP to %d %d\n", goal_x, goal_y);
  carmen_conventional_dynamic_program(goal_x , goal_y);

  carmen_trajectory_to_map(&robot, &map_pt, carmen_planner_map);

  if (carmen_conventional_get_utility(map_pt.x, map_pt.y) < 0) {
    goal_is_accessible = 0;
    if (nav_conf->plan_to_nearest_free_point)
      have_plan = find_nearest_free_point_to_goal();
  } else {
    have_plan = 1;
    goal_is_accessible = 1;
  }

  last_plan_time = carmen_get_time();
  old_goal_x = goal_x;
  old_goal_y = goal_y;
  old_robot = robot;
}

//we need to make regenerate trajectory polymorphic somehow
static void
regenerate_trajectory(carmen_navigator_config_t *nav_conf)
{
  struct timeval start, end;
  int sec, msec;
  int index;
  carmen_traj_point_p path_point;

  gettimeofday(&start, NULL);
  if (extract_path_from_value_function() < 0)
    carmen_planner_util_clear_path(&path);

  else /* if (extract_path_from_value_function() < 0) ... */ {
    if (nav_conf->smooth_path)
      smooth_path(nav_conf);
    //	  Refine_Path_Using_Velocities();

    /* Add path orientations in */
    index = 1;
    while (index < path.length-1) {
      path.points[index].theta =
	atan2(path.points[index+1].y-path.points[index].y,
	      path.points[index+1].x-path.points[index].x);
      index++;
    }
    if (path.length > 1) {
      if (!goal_is_accessible || allow_any_orientation)
	path.points[path.length-1].theta =
	  path.points[path.length-2].theta;
      else
	path.points[path.length-1].theta = requested_goal.theta;
    }

    /* End of adding path orientations */

    gettimeofday(&end, NULL);
    msec = end.tv_usec - start.tv_usec;
    sec = end.tv_sec - start.tv_sec;
    if (msec < 0) {
      msec += 1e6;
      sec--;
    }
    //	  carmen_verbose("Took %d sec %d msec\n", sec, msec);
    for (index = 0; index < path.length; index++) {
      path_point = carmen_planner_util_get_path_point(index, &path);
      carmen_verbose("%.1f %.1f %.1f %.2f\n", path_point->x,
		     path_point->y,
		     carmen_radians_to_degrees(path_point->theta),
		     path_point->t_vel);
    }
  } /* if (extract_path_from_value_function() < 0) ... else ... */
}

int
carmen_planner_update_robot(carmen_traj_point_p new_position,
			    carmen_navigator_config_t *nav_conf)
{
  static carmen_traj_point_t old_position;
  static int first_time = 1;

  if (!carmen_planner_map)
    return 0;

  if (new_position->x < 0 || new_position->y < 0 ||
      new_position->x >
      carmen_planner_map->config.resolution*
      carmen_planner_map->config.x_size ||
      new_position->y >
      carmen_planner_map->config.resolution*carmen_planner_map->config.y_size)
    return 0;

  robot = *new_position;

  if (!first_time && carmen_distance_traj(new_position, &old_position) <
      carmen_planner_map->config.resolution)
    return 0;

  regenerate_trajectory(nav_conf);
  old_position = *new_position;

  return 1;
}

int
carmen_planner_update_goal(carmen_point_p new_goal, int any_orientation,
			   carmen_navigator_config_t *nav_conf)
{
  if (!carmen_planner_map)
    return 0;

  carmen_verbose("Set Goal to X: %f Y: %f (Max allowable %d %d)\n",
		 new_goal->x, new_goal->y, carmen_planner_map->config.x_size,
		 carmen_planner_map->config.y_size);

  requested_goal = *new_goal;
  allow_any_orientation = any_orientation;
  goal_set = 1;

  plan(nav_conf);

  regenerate_trajectory(nav_conf);

  return 1;
}

void carmen_planner_update_grid(carmen_map_p new_map,
				carmen_traj_point_p new_position,
				carmen_robot_config_t *robot_conf,
				carmen_navigator_config_t *nav_conf)
{
  carmen_world_point_t world_point;

  carmen_map_point_t map_point;
  carmen_planner_map = new_map;

  if (true_map != NULL)
    carmen_map_destroy(&true_map);

  true_map = carmen_map_copy(carmen_planner_map);

  world_point.pose.x = new_position->x;
  world_point.pose.y = new_position->y;
  world_point.pose.theta = new_position->theta;
  world_point.map = carmen_planner_map;

  carmen_world_to_map(&world_point, &map_point);

  carmen_conventional_build_costs(robot_conf, &map_point, nav_conf);

  if (!goal_set)
    return;

  plan(nav_conf);

  regenerate_trajectory(nav_conf);
}

void
carmen_planner_set_map(carmen_map_p new_map, carmen_robot_config_t *robot_conf)
{
  carmen_planner_map = new_map;

  if (true_map != NULL)
    carmen_map_destroy(&true_map);

  true_map = carmen_map_copy(carmen_planner_map);

  map_modify_clear(true_map, carmen_planner_map);
  carmen_conventional_build_costs(robot_conf, NULL, NULL);
}


void
carmen_planner_reset_map(carmen_robot_config_t *robot_conf)
{
  map_modify_clear(true_map, carmen_planner_map);
  carmen_conventional_build_costs(robot_conf, NULL, NULL);
}

void
carmen_planner_update_map(carmen_robot_laser_message *laser_msg,
			  carmen_navigator_config_t *nav_conf,
			  carmen_robot_config_t *robot_conf)
{
  carmen_world_point_t world_point;
  carmen_map_point_t map_point;

  if (carmen_planner_map == NULL)
    return;

  world_point.pose.x = laser_msg->laser_pose.x;
  world_point.pose.y = laser_msg->laser_pose.y;
  world_point.pose.theta = laser_msg->laser_pose.theta;
  world_point.map = carmen_planner_map;

  /// CYRILL: HIER UEBERGABE AENDERN! (laser cfg)

  map_modify_update(laser_msg, nav_conf, &world_point, true_map, carmen_planner_map);

  carmen_world_to_map(&world_point, &map_point);
  carmen_conventional_build_costs(robot_conf, &map_point, nav_conf);

  if (!goal_set)
    return;

  plan(nav_conf);

  regenerate_trajectory(nav_conf);
}

void
carmen_planner_get_status(carmen_planner_status_p status)
{
  int index;

  status->goal = requested_goal;
  status->robot = robot;
  status->goal_set = goal_set;
  status->path.length = path.length;
  if (status->path.length > 0) {
    status->path.points = (carmen_traj_point_p)
      calloc(status->path.length, sizeof(carmen_traj_point_t));
    carmen_test_alloc(status->path.points);
    for (index = 0; index < status->path.length; index++)
      status->path.points[index] = path.points[index];
  } else  {
    status->path.points = NULL;
  }
  return;
}

// Add a check for planners with goals-- if we're just tracking'
// then there is no goal

int
carmen_planner_next_waypoint(carmen_traj_point_p waypoint,
			     int* waypoint_index,
			     int *is_goal,
			     carmen_navigator_config_t *nav_conf)
{
  carmen_traj_point_p point;
  int next_point;
  double delta_dist, delta_theta;

  regenerate_trajectory(nav_conf);

  carmen_verbose("Finding waypoint: %.0f %.0f %.0f\n",
		 waypoint->x, waypoint->y,
		 carmen_radians_to_degrees(waypoint->theta));

  if (path.length <= 1)
    return -1;

  next_point = 0;
  do {
    next_point++;
    point = carmen_planner_util_get_path_point(next_point, &path);

    if (path.length - next_point < 2)
      break;
    delta_dist = carmen_distance_traj(waypoint, point);
  } while (delta_dist < nav_conf->waypoint_tolerance);

  delta_dist = carmen_distance_traj(waypoint, point);

  if (delta_dist < nav_conf->goal_size && path.length - next_point == 1) {
    if (allow_any_orientation || !goal_is_accessible)
      return 1;
    delta_theta = fabs(waypoint->theta - requested_goal.theta) ;
    if (delta_theta < nav_conf->goal_theta_tolerance)
      return 1;
  }

  if (path.length <= 2)
    *is_goal = 1;
  else
    *is_goal = 0;

  *waypoint = *point;
  *waypoint_index = next_point;

  carmen_verbose("Waypoint is: %.0f %.0f %.0f (goal %d)\n",
		 waypoint->x, waypoint->y,
		 carmen_radians_to_degrees(waypoint->theta), *is_goal);

  return 0;
}

carmen_navigator_map_message *
carmen_planner_get_map_message(carmen_navigator_map_t map_type)
{
  carmen_navigator_map_message *reply;
  int size, x_Size, y_Size;
  float *map_ptr;
  double *dbl_ptr;

  reply = (carmen_navigator_map_message *)
    calloc(1, sizeof(carmen_navigator_map_message));
  carmen_test_alloc(reply);

  x_Size = carmen_planner_map->config.x_size;
  y_Size = carmen_planner_map->config.y_size;
  size = x_Size * y_Size;

  reply->data = (unsigned char *)calloc(size, sizeof(float));
  carmen_test_alloc(reply->data);

  reply->config = carmen_planner_map->config;

  switch (map_type) {
  case CARMEN_NAVIGATOR_MAP_v:
    if (true_map)
      map_ptr = true_map->complete_map;
    else
      map_ptr = carmen_planner_map->complete_map;
    break;
  case CARMEN_NAVIGATOR_COST_v:
    dbl_ptr = carmen_conventional_get_costs_ptr();
    if (dbl_ptr == NULL) {
      reply->size = 0;
      reply->map_type = -1;
      return reply;
    }
    {
      int index;
      float *mp;
      double *db_p;
      map_ptr = (float *)calloc(size, sizeof(float));
      carmen_test_alloc(map_ptr);
      mp = map_ptr;
      db_p = dbl_ptr;
      for (index = 0; index < size; index++)
	*(mp++) = (float)*(dbl_ptr++);
    }
    break;
  case CARMEN_NAVIGATOR_UTILITY_v:
    dbl_ptr = carmen_conventional_get_utility_ptr();
    if (dbl_ptr == NULL) {
      reply->size = 0;
      reply->map_type = -1;
      return reply;
    }
    {
      int index;
      float *mp;
      double *db_p;
      map_ptr = (float *)calloc(size, sizeof(float));
      carmen_test_alloc(map_ptr);
      mp = map_ptr;
      db_p = dbl_ptr;
      for (index = 0; index < size; index++)
	*(mp++) = (float)*(dbl_ptr++);
    }
    break;
  default:
    carmen_warn("Request for unsupported data type : %d.\n", map_type);
    reply->size = 0;
    reply->map_type = -1;
    return reply;
  }
  reply->size = size*sizeof(float);
  reply->map_type = map_type;
  memcpy(reply->data, map_ptr, size*sizeof(float));
  if (map_type == CARMEN_NAVIGATOR_UTILITY_v)
    free(map_ptr);
  return reply;
}

double *carmen_planner_get_utility(void)
{
  return carmen_conventional_get_utility_ptr();
}

int carmen_planner_goal_reachable(void)
{
  if (goal_set && goal_is_accessible)
    return 1;

  return 0;
}
