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

#include "global.h"
#include "robot_interface.h"

#ifndef COMPILE_WITHOUT_LASER_SUPPORT
void robot_frontlaser_handler(carmen_robot_laser_message *front_laser)
{
  carmen_warn("front_laser\n");
  carmen_warn("%.2f %.2f %.2f\n", front_laser->robot_pose.x, 
	      front_laser->robot_pose.y, 
	      carmen_radians_to_degrees(front_laser->robot_pose.theta));
}

void robot_rearlaser_handler(carmen_robot_laser_message *rear_laser)
{
  carmen_warn("\nrear_laser\n");
  carmen_warn("%.2f %.2f %.2f\n", rear_laser->robot_pose.x, 
	      rear_laser->robot_pose.y, 
	      carmen_radians_to_degrees(rear_laser->robot_pose.theta));
}
#endif

void robot_sonar_handler(carmen_robot_sonar_message *sonar_message)
{
  int i;
  carmen_warn("\nrobot_sonar\n");

  carmen_warn("num_sonars %i\n",sonar_message->num_sonars);

  carmen_warn("cone_angle %.3f\n",sonar_message->cone_angle);

  carmen_warn("range ");
  for(i=0; i<sonar_message->num_sonars; i++)
    carmen_warn("%.1f ",sonar_message->ranges[i]);
  carmen_warn("\n");

  carmen_warn("robot_pose (%.3f,%.3f,%.3f)\n",
	  sonar_message->robot_pose.x,
	  sonar_message->robot_pose.y,
	  sonar_message->robot_pose.theta);

  carmen_warn("sonar sonar_offsets ");
  for(i=0; i<sonar_message->num_sonars; i++)
    carmen_warn("(%.3f,%.3f,%.3f) ",
	    sonar_message->sonar_offsets[i].x,
	    sonar_message->sonar_offsets[i].y,
	    sonar_message->sonar_offsets[i].theta);
  carmen_warn("\n");
					       
  carmen_warn("timestamp %f\n", sonar_message->timestamp);

  carmen_warn("host %s\n\n", sonar_message->host);
}

void base_odometry_handler(carmen_base_odometry_message *odometry)
{
  carmen_warn("%.2f %.2f %.2f\n", odometry->x, odometry->y, 
	    carmen_radians_to_degrees(odometry->theta));
}


void shutdown_module(int x)
{
  if(x == SIGINT) {
    carmen_robot_velocity_command(0, 0);
    carmen_ipc_disconnect();
    printf("Disconnected from robot.\n");
    exit(0);
  }
}

int main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);
  
  signal(SIGINT, shutdown_module);

#ifndef COMPILE_WITHOUT_LASER_SUPPORT
  carmen_robot_subscribe_frontlaser_message
    (NULL, (carmen_handler_t)robot_frontlaser_handler,
     CARMEN_SUBSCRIBE_LATEST);
  carmen_robot_subscribe_rearlaser_message
    (NULL,(carmen_handler_t)robot_rearlaser_handler,
     CARMEN_SUBSCRIBE_LATEST);
#endif
  carmen_robot_subscribe_sonar_message
    (NULL, (carmen_handler_t)robot_sonar_handler,
     CARMEN_SUBSCRIBE_LATEST);
  carmen_base_subscribe_odometry_message
    (NULL, (carmen_handler_t)base_odometry_handler,
     CARMEN_SUBSCRIBE_LATEST);

  carmen_robot_move_along_vector(0, M_PI/2);
  
  sleep(10);
  carmen_robot_move_along_vector(2, 0);

  while(1) {
    carmen_ipc_sleep(0.1);
    carmen_warn(".");
  }
  return 0;
}
