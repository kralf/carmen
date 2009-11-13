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

#include "global_graphics.h"

#include "map_io.h"

#include "navigator.h"
#include "navigator_graphics.h"
#include "navigator_panel.h"

static carmen_map_p map;

void navigator_create_map(void)
{
  map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(map);
  carmen_map_get_gridmap(map);
  if (map->map == NULL)
    exit(0);

  navigator_graphics_change_map(map);
}

carmen_map_p navigator_get_map(void)
{
  return map;
}

void navigator_change_map(carmen_map_p new_map)
{
  carmen_map_destroy(&map);
  map = carmen_map_copy(new_map);

  navigator_graphics_change_map(map);
}

void navigator_destroy_map(void)
{
  carmen_map_destroy(&map);
}

void navigator_display_map(carmen_navigator_map_t type)
{
  carmen_map_t new_map;
  int index;

  memset(&new_map, 0, sizeof(carmen_map_t));

  if (type == CARMEN_LOCALIZE_LMAP_v || type == CARMEN_LOCALIZE_GMAP_v) {
    if (type == CARMEN_LOCALIZE_LMAP_v)
      carmen_localize_get_map(0, &new_map);
    else
      carmen_localize_get_map(1, &new_map);

    if (new_map.complete_map == NULL)
      return;

    for (index = 0; index < new_map.config.x_size*new_map.config.y_size;
	 index++)
      new_map.complete_map[index] = exp(new_map.complete_map[index]);
  } else {
    carmen_navigator_get_map(type, &new_map);

    if (new_map.complete_map == NULL)
      return;

  }

  if (map && strcmp(map->config.map_name, new_map.config.map_name) == 0) {
    navigator_graphics_display_map(new_map.complete_map, type);
    return;
  }

  if (map)
    carmen_map_destroy(&map);

  map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(map);

  *map = new_map;

  navigator_graphics_change_map(map);
}

void navigator_update_robot(carmen_world_point_p robot)
{
  carmen_point_t std = {0.2, 0.2, carmen_degrees_to_radians(4.0)};

  if (robot == NULL) {
    carmen_localize_initialize_uniform_command();
  } else {
    carmen_verbose("Set robot position to %d %d %f\n",
		   carmen_round(robot->pose.x),
		   carmen_round(robot->pose.y),
		carmen_radians_to_degrees(robot->pose.theta));

    carmen_localize_initialize_gaussian_command(robot->pose, std);
  }
}

void navigator_set_goal(double x, double y)
{
  carmen_verbose("Set goal to %.1f %.1f\n", x, y);
  carmen_navigator_set_goal(x, y);
}

void navigator_set_goal_by_place(carmen_place_p place)
{
  carmen_navigator_set_goal_place(place->name);
}

void navigator_stop_moving(void)
{
  if (!carmen_navigator_stop())
    carmen_verbose("Said stop\n");
  else
    carmen_verbose("Could not say stop\n");
}

void navigator_start_moving(void)
{
  if (!carmen_navigator_go())
    carmen_verbose("Said go!\n");
  else
    carmen_verbose("could not say go!\n");

}
