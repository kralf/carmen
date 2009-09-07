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

/** @addtogroup robot librobot_interface **/
// @{

/** \file robot_interface.h
 * \brief Definition of the interface of the module robot.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef CARMEN_ROBOT_INTERFACE_H
#define CARMEN_ROBOT_INTERFACE_H

#include <carmen/robot_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMPILE_WITHOUT_LASER_SUPPORT
void carmen_robot_subscribe_frontlaser_message
(carmen_robot_laser_message *laser, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_subscribe_rearlaser_message
(carmen_robot_laser_message *laser, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);
#endif

void carmen_robot_subscribe_sonar_message
(carmen_robot_sonar_message *sonar, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_subscribe_vector_status_message
(carmen_robot_vector_status_message *vector, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_subscribe_base_binary_data_message
(carmen_base_binary_data_message *base_data, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);
 
void carmen_robot_subscribe_follow_trajectory_message
(carmen_robot_follow_trajectory_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);
 
void carmen_robot_subscribe_vector_move_message
(carmen_robot_vector_move_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_subscribe_velocity_message
(carmen_robot_velocity_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);
 

void carmen_robot_velocity_command(double tv, double rv);
void carmen_robot_move_along_vector(double distance, double theta);
void carmen_robot_follow_trajectory(carmen_traj_point_t *trajectory, int trajectory_length,
				    carmen_traj_point_t *robot);
void carmen_robot_send_base_binary_command(unsigned char *data, int length);

#ifdef __cplusplus
}
#endif

#endif
// @}
