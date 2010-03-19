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

#include "navigator.h"

#include "navigator_messages.h"
#include "planner.h"

carmen_map_p nav_map;
carmen_map_placelist_t placelist;
carmen_robot_config_t robot_config;
carmen_navigator_config_t nav_config;

int cheat = 0;
int autonomous_status = 0;

carmen_traj_point_t robot_position;

carmen_map_placelist_p
carmen_navigator_get_places(void)
{
  return &placelist;
}

void
carmen_navigator_set_max_velocity(double vel)
{
  robot_config.max_t_vel = vel;
}

void
carmen_navigator_goal(double x, double y)
{
  carmen_point_t point;

  point.x = x;
  point.y = y;

  carmen_planner_update_goal(&point, 1, &nav_config);
}

void
carmen_navigator_goal_triplet(carmen_point_p point)
{
  carmen_planner_update_goal(point, 0, &nav_config);
}

int
carmen_navigator_goal_place(char *name)
{
  int index;
  carmen_point_t goal;

  for (index = 0; index < placelist.num_places; index++)
    {
      if (strcmp(name, placelist.places[index].name) == 0)
	break;
    }

  if (index == placelist.num_places)
    return -1;

  goal.x = placelist.places[index].x;
  goal.y = placelist.places[index].y;

  if (placelist.places[index].type == CARMEN_NAMED_POSITION_TYPE)
    carmen_planner_update_goal(&goal, 1, &nav_config);
  else
    {
      goal.theta = placelist.places[index].theta;
      carmen_planner_update_goal(&goal, 0, &nav_config);
    }

  return 0;
}

void
carmen_navigator_start_autonomous(void)
{
  autonomous_status = 1;
  carmen_planner_reset_map(&robot_config);
  generate_next_motion_command();
}

void
carmen_navigator_stop_autonomous(void)
{
  autonomous_status = 0;
  carmen_navigator_publish_autonomous_stopped(CARMEN_NAVIGATOR_USER_STOPPED_v);
  carmen_robot_velocity_command(0, 0);
}

int
carmen_navigator_autonomous_status(void)
{
  return autonomous_status;
}

void
generate_next_motion_command(void)
{
  carmen_traj_point_t waypoint;
  int waypoint_index;
  int waypoint_status;
  int is_goal;
  command_t command;
  carmen_planner_status_t status;

  command.tv = 0;
  command.rv = 0;

  waypoint = robot_position;

  waypoint_status =
    carmen_planner_next_waypoint(&waypoint, &waypoint_index,
         &is_goal, &nav_config);

  /* goal is reached */

  if (waypoint_status > 0)
    {
      autonomous_status = 0;
      carmen_navigator_publish_autonomous_stopped
  (CARMEN_NAVIGATOR_GOAL_REACHED_v);

      carmen_verbose("Autonomous off Motion command %.2f %.2f\n", command.tv,
         carmen_radians_to_degrees(command.rv));

      command.tv = 0;
      command.rv = 0;
      carmen_robot_velocity_command(command.tv, command.rv);
      return;
    }

  if (waypoint_status < 0)
    {
      command.tv = 0;
      command.rv = 0;
      carmen_robot_velocity_command(command.tv, command.rv);
      return;
    }

  carmen_planner_get_status(&status);
  /*   carmen_robot_follow_trajectory(status.path.points+1, status.path.length-1, &robot_position); */


  carmen_robot_follow_trajectory(status.path.points+waypoint_index,
         status.path.length-waypoint_index,
         &robot_position);

  if (status.path.length > 0)
    free(status.path.points);

  carmen_verbose("Current pos: %.1f %.1f %.1f next %.1f %.1f\n",
     robot_position.x, robot_position.y,
     carmen_radians_to_degrees(robot_position.theta),
     waypoint.x, waypoint.y);
}
