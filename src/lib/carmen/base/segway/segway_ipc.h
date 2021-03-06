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

#ifndef SEGWAY_IPC_H
#define SEGWAY_IPC_H

#include "segway_core.h"

extern segway_t segway;
extern double segway_command_tv, segway_command_rv;
extern double segway_last_command;
extern int segway_quit_signal;

void carmen_segway_register_messages(void);

void carmen_segway_publish_odometry(segway_p segway, double timestamp);

void carmen_segway_publish_pose(segway_p segway, double timestamp);

void carmen_segway_publish_battery(segway_p segway);

void carmen_segway_publish_alive(void);

#endif
