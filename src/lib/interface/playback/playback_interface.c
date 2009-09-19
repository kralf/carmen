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

#include "global.h"

#include "playback_interface.h"

void carmen_playback_command(int command, int argument, float speed)
{
  IPC_RETURN_TYPE err;
  carmen_playback_command_message playback_msg;

  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_PLAYBACK_COMMAND_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_PLAYBACK_COMMAND_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_PLAYBACK_COMMAND_NAME);
    }

  playback_msg.cmd = command;
  playback_msg.arg = argument;
  playback_msg.speed = speed;

  err = IPC_publishData(CARMEN_PLAYBACK_COMMAND_NAME, &playback_msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_PLAYBACK_COMMAND_NAME);
}
