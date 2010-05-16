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

#include "param_interface.h"
#include "simulator_interface.h"
#include "localize_interface.h"
#include "navigator_interface.h"
#include "map_interface.h"

#include "navigator_graphics.h"
#include "navigator_panel.h"

static carmen_map_placelist_t gui_placelist;
static double last_navigator_status = 0.0;

void navigator_status_handler(carmen_navigator_status_message *msg)
{
  carmen_traj_point_t new_robot;
  carmen_world_point_t new_goal;

  carmen_verbose("Got Status message: Robot %.1f %.1f %.2f Goal: %.0f %.0f\n",
		 msg->robot.x, msg->robot.y, msg->robot.theta,
		 msg->goal.x, msg->goal.y);

  last_navigator_status = msg->timestamp;
  new_robot = msg->robot;

  new_goal.map = navigator_get_map();

  if (msg->goal_set) {
    new_goal.pose = msg->goal;
    navigator_graphics_update_display(&new_robot, &new_goal, msg->autonomous);
  } else
    navigator_graphics_update_display(&new_robot, NULL, msg->autonomous);
}

void navigator_plan_handler(carmen_navigator_plan_message *plan)
{
  navigator_graphics_update_plan(plan->path, plan->path_length);
}

void map_update_handler(carmen_map_t *new_map)
{
  navigator_change_map(new_map);
}

static void nav_shutdown(int signo __attribute__ ((unused)))
{
  static int done = 0;

  if(!done) {
    done = 1;
    carmen_ipc_disconnect();
    exit(-1);
  }
}

static gint handle_ipc(gpointer *data __attribute__ ((unused)),
		       gint source __attribute__ ((unused)),
		       GdkInputCondition condition __attribute__ ((unused)))
{
  carmen_ipc_sleep(0.01);

  carmen_graphics_update_ipc_callbacks((GdkInputFunction)handle_ipc);

  return 1;
}

static void globalpos_handler(carmen_localize_globalpos_message *msg)
{
  carmen_traj_point_t new_robot;

  if (msg->timestamp - last_navigator_status < 30)
    return;

  new_robot.x = msg->globalpos.x;
  new_robot.y = msg->globalpos.y;
  new_robot.theta = msg->globalpos.theta;
  new_robot.t_vel = 0;
  new_robot.r_vel = 0;

  navigator_graphics_update_display(&new_robot, NULL, 0);
}

static void truepos_handler(carmen_simulator_truepos_message *msg)
{
  navigator_graphics_update_simulator_truepos(msg->truepose);
}

static void objects_handler(carmen_simulator_objects_message *msg)
{
  navigator_graphics_update_simulator_objects
    (msg->num_objects, msg->objects_list);
}

static void display_config_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
				   void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err = IPC_OK;
  FORMATTER_PTR formatter;
  carmen_navigator_display_config_message msg;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_navigator_display_config_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall",
			 IPC_msgInstanceName(msgRef));

  if (msg.reset_all_to_defaults)
    navigator_graphics_reset();
  else
    navigator_graphics_display_config
      (msg.attribute, msg.value, msg.status_message);
  free(msg.attribute);
  if (msg.status_message)
    free(msg.status_message);
}

static void read_parameters(int argc, char *argv[],
			    carmen_robot_config_t *robot_config,
			    carmen_navigator_config_t *nav_config,
			    carmen_navigator_panel_config_t
			    *navigator_panel_config)
{
  int num_items;

  carmen_param_t param_list[] = {
    {"robot", "max_t_vel", CARMEN_PARAM_DOUBLE,
     &(robot_config->max_t_vel), 1, NULL},
    {"robot", "max_r_vel", CARMEN_PARAM_DOUBLE,
     &(robot_config->max_r_vel), 1, NULL},
    {"robot", "min_approach_dist", CARMEN_PARAM_DOUBLE,
     &(robot_config->approach_dist), 1, NULL},
    {"robot", "min_side_dist", CARMEN_PARAM_DOUBLE,
     &(robot_config->side_dist), 1, NULL},
    {"robot", "length", CARMEN_PARAM_DOUBLE,
     &(robot_config->length), 0, NULL},
    {"robot", "width", CARMEN_PARAM_DOUBLE,
     &(robot_config->width), 0, NULL},
    {"robot", "acceleration", CARMEN_PARAM_DOUBLE,
     &(robot_config->acceleration), 1, NULL},
    {"robot", "reaction_time", CARMEN_PARAM_DOUBLE,
     &(robot_config->reaction_time), 0, NULL},
    {"robot", "rectangular", CARMEN_PARAM_INT,
     &(robot_config->rectangular), 1, NULL},

    {"navigator", "map_update_radius", CARMEN_PARAM_INT,
     &(nav_config->map_update_radius), 1, NULL},
    {"navigator", "goal_size", CARMEN_PARAM_DOUBLE,
     &(nav_config->goal_size), 1, NULL},
    {"navigator", "goal_theta_tolerance", CARMEN_PARAM_DOUBLE,
     &(nav_config->goal_theta_tolerance), 1, NULL},

    {"navigator_panel", "initial_map_zoom", CARMEN_PARAM_DOUBLE,
     &(navigator_panel_config->initial_map_zoom), 1, NULL},
    {"navigator_panel", "track_robot", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->track_robot), 1, NULL},
    {"navigator_panel", "draw_waypoints", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->draw_waypoints), 1, NULL},
    {"navigator_panel", "show_particles", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->show_particles), 1, NULL},
    {"navigator_panel", "show_gaussians", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->show_gaussians), 1, NULL},
    {"navigator_panel", "show_laser", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->show_lasers), 1, NULL},
    {"navigator_panel", "show_simulator_objects", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->show_simulator_objects), 1, NULL},
    {"navigator_panel", "show_true_pos", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->show_true_pos), 1, NULL},
    {"navigator_panel", "show_tracked_objects", CARMEN_PARAM_ONOFF,
     &(navigator_panel_config->show_tracked_objects), 1, NULL}
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);

  carmen_param_install_params(argc, argv, param_list, num_items);
}

int main(int argc, char **argv)
{
  carmen_robot_config_t robot_config;
  carmen_navigator_config_t nav_config;
  carmen_navigator_panel_config_t nav_panel_config;

  carmen_navigator_status_message status;
  carmen_localize_globalpos_message globalpos;
  carmen_navigator_plan_message *plan;
  IPC_RETURN_TYPE err;

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, nav_shutdown);

  read_parameters(argc, argv, &robot_config, &nav_config, &nav_panel_config);

  carmen_navigator_subscribe_status_message
    (&status, (carmen_handler_t)navigator_status_handler,
     CARMEN_SUBSCRIBE_LATEST);
  carmen_navigator_subscribe_plan_message
    (NULL, (carmen_handler_t)navigator_plan_handler, CARMEN_SUBSCRIBE_LATEST);
  /*  carmen_localize_subscribe_people_message
      (&people_message, (carmen_handler_t)people_handler,CARMEN_SUBSCRIBE_LATEST);*/
  carmen_localize_subscribe_globalpos_message
    (&globalpos, (carmen_handler_t)globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

  carmen_simulator_subscribe_truepos_message
    (NULL, (carmen_handler_t)truepos_handler, CARMEN_SUBSCRIBE_LATEST);

  carmen_simulator_subscribe_objects_message
    (NULL, (carmen_handler_t)objects_handler, CARMEN_SUBSCRIBE_LATEST);

  carmen_map_subscribe_gridmap_update_message
    (NULL, (carmen_handler_t)map_update_handler, CARMEN_SUBSCRIBE_LATEST);

  err = IPC_defineMsg
    (CARMEN_NAVIGATOR_DISPLAY_CONFIG_NAME, IPC_VARIABLE_LENGTH,
     CARMEN_NAVIGATOR_DISPLAY_CONFIG_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_DISPLAY_CONFIG_NAME);

  err = IPC_subscribe(CARMEN_NAVIGATOR_DISPLAY_CONFIG_NAME,
		      display_config_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe message",
		       CARMEN_NAVIGATOR_DISPLAY_CONFIG_NAME);

  navigator_graphics_init(argc, argv, &globalpos, &robot_config, &nav_config,
			  &nav_panel_config);
  navigator_graphics_add_ipc_handler((GdkInputFunction)handle_ipc);

  navigator_create_map();

  carmen_map_get_placelist(&gui_placelist);

  navigator_graphics_add_placelist(&gui_placelist);

  carmen_navigator_query_plan(&plan);

#ifdef USE_DOT
  initialize_dynamics();
#endif

  if (plan && plan->path_length > 0) {
    navigator_graphics_update_plan(plan->path, plan->path_length);
    free(plan->path);
    free(plan);
  }

  navigator_graphics_start();

  navigator_destroy_map();

  return 0;
}

