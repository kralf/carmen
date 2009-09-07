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

#include <carmen/carmen.h>

#ifndef COMPILE_WITHOUT_LASER_SUPPORT
void 
carmen_robot_subscribe_frontlaser_message(carmen_robot_laser_message *laser,
					  carmen_handler_t handler,
					  carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_ROBOT_FRONTLASER_NAME, 
			   CARMEN_ROBOT_FRONTLASER_FMT,
			   laser, sizeof(carmen_robot_laser_message),
			   handler, subscribe_how);
}

void 
carmen_robot_subscribe_rearlaser_message(carmen_robot_laser_message *laser,
					 carmen_handler_t handler,
					 carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_ROBOT_REARLASER_NAME, 
			   CARMEN_ROBOT_REARLASER_FMT,
			   laser, sizeof(carmen_robot_laser_message),
			   handler, subscribe_how);
}
#endif

void
carmen_robot_subscribe_sonar_message(carmen_robot_sonar_message *sonar,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how) 
{
  carmen_subscribe_message(CARMEN_ROBOT_SONAR_NAME, 
			   CARMEN_ROBOT_SONAR_FMT,
			   sonar, sizeof(carmen_robot_sonar_message),
			   handler, subscribe_how);
}

void
carmen_robot_subscribe_vector_status_message
(carmen_robot_vector_status_message *status, carmen_handler_t handler, 
 carmen_subscribe_t subscribe_how) 
{
  carmen_subscribe_message(CARMEN_ROBOT_VECTOR_STATUS_NAME, 
			   CARMEN_ROBOT_VECTOR_STATUS_FMT,
			   status, sizeof(carmen_robot_vector_status_message),
			   handler, subscribe_how);
}

void
carmen_robot_subscribe_base_binary_data_message
 (carmen_base_binary_data_message *base_data, carmen_handler_t handler,
  carmen_subscribe_t subscribe_how) 
{
  carmen_subscribe_message(CARMEN_BASE_BINARY_DATA_NAME, 
			   CARMEN_BASE_BINARY_DATA_FMT,
			   base_data, sizeof(carmen_base_binary_data_message),
			   handler, subscribe_how);
}

void carmen_robot_subscribe_follow_trajectory_message
(carmen_robot_follow_trajectory_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_FOLLOW_TRAJECTORY_NAME,
				CARMEN_ROBOT_FOLLOW_TRAJECTORY_FMT,
				msg, sizeof(carmen_robot_follow_trajectory_message),
				handler, subscribe_how);	
}
 
void carmen_robot_subscribe_vector_move_message
(carmen_robot_vector_move_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_VECTOR_MOVE_NAME,
				CARMEN_ROBOT_VECTOR_MOVE_FMT,
				msg, sizeof(carmen_robot_vector_move_message),
				handler, subscribe_how);
}

void carmen_robot_subscribe_velocity_message
(carmen_robot_velocity_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_VELOCITY_NAME,
				CARMEN_ROBOT_VELOCITY_FMT,
				msg, sizeof(carmen_robot_velocity_message),
				handler, subscribe_how);
}





void 
carmen_robot_velocity_command(double tv, double rv)
{
  IPC_RETURN_TYPE err;
  static carmen_robot_velocity_message v;

  err = IPC_defineMsg(CARMEN_ROBOT_VELOCITY_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_ROBOT_VELOCITY_FMT);
  carmen_test_ipc_exit(err, "Could not define message", 
		       CARMEN_ROBOT_VELOCITY_NAME);

  v.tv = tv;
  v.rv = rv;
  v.host = carmen_get_host();
  v.timestamp = carmen_get_time();
  
  err = IPC_publishData(CARMEN_ROBOT_VELOCITY_NAME, &v);
  carmen_test_ipc(err, "Could not publish", CARMEN_ROBOT_VELOCITY_NAME);
}

void
carmen_robot_move_along_vector(double distance, double theta)
{
  IPC_RETURN_TYPE err;
  static carmen_robot_vector_move_message msg;

  err = IPC_defineMsg(CARMEN_ROBOT_VECTOR_MOVE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_ROBOT_VECTOR_MOVE_FMT);
  carmen_test_ipc_exit(err, "Could not define message", 
		       CARMEN_ROBOT_VECTOR_MOVE_NAME);
  msg.distance = distance;
  msg.theta = theta;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_ROBOT_VECTOR_MOVE_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_ROBOT_VECTOR_MOVE_NAME);
}

void
carmen_robot_follow_trajectory(carmen_traj_point_t *trajectory, 
			       int trajectory_length,
			       carmen_traj_point_t *robot)
{
  IPC_RETURN_TYPE err;
  static carmen_robot_follow_trajectory_message msg;

  err = IPC_defineMsg(CARMEN_ROBOT_FOLLOW_TRAJECTORY_NAME, 
		      IPC_VARIABLE_LENGTH, 
		      CARMEN_ROBOT_FOLLOW_TRAJECTORY_FMT);
  carmen_test_ipc_exit(err, "Could not define message", 
		       CARMEN_ROBOT_FOLLOW_TRAJECTORY_NAME);

  msg.trajectory = trajectory;
  msg.trajectory_length = trajectory_length;
  msg.robot_position = *robot;

  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_ROBOT_FOLLOW_TRAJECTORY_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", 
		  CARMEN_ROBOT_FOLLOW_TRAJECTORY_NAME);
}

void 
carmen_robot_send_base_binary_command(unsigned char *data, int length)
{
  IPC_RETURN_TYPE err;
  static carmen_base_binary_data_message msg;

  err = IPC_defineMsg(CARMEN_BASE_BINARY_COMMAND_NAME, 
		      IPC_VARIABLE_LENGTH, 
		      CARMEN_BASE_BINARY_COMMAND_FMT);
  carmen_test_ipc_exit(err, "Could not define message", 
		       CARMEN_BASE_BINARY_COMMAND_NAME);

  msg.data = data;
  msg.size = length;

  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_BASE_BINARY_COMMAND_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", 
		  CARMEN_BASE_BINARY_COMMAND_NAME);
}
