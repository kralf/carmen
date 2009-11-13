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

#ifndef NAVIGATOR_PANEL_H
#define NAVIGATOR_PANEL_H

#include "navigator_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct {
    double initial_map_zoom;
    int track_robot;
    int draw_waypoints;
    int show_particles;
    int show_gaussians;
    int show_lasers;
    int show_simulator_objects;
    int show_true_pos;
    int show_tracked_objects;
  } carmen_navigator_panel_config_t;

  void navigator_create_map(void);
  carmen_map_p navigator_get_map(void);
  void navigator_change_map(carmen_map_p new_map);
  void navigator_destroy_map(void);

  void navigator_update_robot(carmen_world_point_p robot);
  void navigator_set_goal(double x, double y);
  void navigator_set_goal_by_place(carmen_place_p place);
  void navigator_stop_moving(void);
  void navigator_start_moving(void);
  void navigator_display_map(carmen_navigator_map_t New_Display);

#ifdef __cplusplus
}
#endif

#endif
