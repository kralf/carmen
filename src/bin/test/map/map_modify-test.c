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

#include "map_graphics.h"

#include "navigator.h"
#include "map_interface.h"
#include "map_modify.h"

static carmen_map_p map, true_map;
static carmen_localize_globalpos_message globalpos;

static GtkWidget *window;
static GtkMapViewer *map_view;

static void delete_event(GtkWidget *widget, GdkEvent *event, gpointer data) 
{
  widget = widget; event = event; data = data;
  exit(0);
}

static void initialize_people_graphics(int argc, char *argv[])
{
  /* GtkWidget is the storage type for widgets */
  GtkWidget *main_box;

  gtk_init (&argc, &argv);

  carmen_graphics_setup_colors();
  
  /* Create a new window */
  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

  gtk_signal_connect (GTK_OBJECT (window), "destroy", 
		      GTK_SIGNAL_FUNC (gtk_main_quit), 
		      "WM destroy");

  gtk_window_set_title (GTK_WINDOW (window), "Test Map Modify");
  gtk_signal_connect (GTK_OBJECT (window), "delete_event",
		      GTK_SIGNAL_FUNC (delete_event), NULL);
  gtk_container_set_border_width (GTK_CONTAINER (window), 0);

  main_box = gtk_vbox_new(FALSE, 0);
  gtk_container_border_width (GTK_CONTAINER (main_box), 0); 
  gtk_container_add (GTK_CONTAINER (window), main_box);

  map_view = carmen_map_graphics_new_viewer(400, 400, 100.0);
  gtk_box_pack_start(GTK_BOX (main_box), map_view->map_box, FALSE, FALSE, 0);

  assert (map != NULL);
  assert (map->map != NULL);
  carmen_map_graphics_add_map(map_view, map, 0);

  gtk_widget_show_all (window);
  gtk_widget_grab_focus(window);

  return;
}

static gint 
handle_ipc(gpointer *data __attribute__ ((unused)), 
           gint source __attribute__ ((unused)), 
           GdkInputCondition condition __attribute__ ((unused))) 
{
  carmen_ipc_sleep(0.01);
  
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)handle_ipc);

  return 1;
}

void 
robot_frontlaser_handler(carmen_robot_laser_message *msg) 
{
  carmen_world_point_t world_point;
  carmen_navigator_config_t nav_conf;
  
  carmen_localize_correct_laser(msg, &globalpos);
  
  world_point.pose.x = msg->laser_pose.x;
  world_point.pose.y = msg->laser_pose.y;
  world_point.pose.theta = msg->laser_pose.theta;
  world_point.map = map;

  nav_conf.map_update_radius = 3.0;
  nav_conf.num_lasers_to_use = 361;
  nav_conf.map_update_freespace = 1;
  nav_conf.map_update_obstacles = 1;

  
  map_modify_update(msg, &nav_conf, &world_point, true_map, map);

  carmen_map_graphics_modify_map(map_view, map->complete_map, 0);
  carmen_map_graphics_adjust_scrollbars(map_view, &world_point);    
  carmen_map_graphics_redraw(map_view);
}

int main(int argc, char *argv[])
{
  carmen_map_t global_map = {{0, 0, 0, 0}, 0, 0};
  carmen_robot_laser_message laser_msg; 
  
  laser_msg.range = NULL;
  laser_msg.tooclose = NULL;
  
  carmen_randomize(&argc, &argv);
  carmen_ipc_initialize(argc, argv);
  
  map = &global_map;
  carmen_map_get_gridmap(map);
  true_map = carmen_map_copy(map);
  
  carmen_robot_subscribe_frontlaser_message
    (&laser_msg, (carmen_handler_t)robot_frontlaser_handler,
     CARMEN_SUBSCRIBE_LATEST);
  
  carmen_localize_subscribe_globalpos_message
    (&globalpos, NULL, CARMEN_SUBSCRIBE_LATEST);
  
  initialize_people_graphics(argc, argv);
  
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)handle_ipc);
  gtk_main();
  
  return 0;
}
