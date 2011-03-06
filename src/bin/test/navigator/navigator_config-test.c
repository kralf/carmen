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

#include "navigator_interface.h"

void
navigator_display_config(char *attribute, int value)
{
  carmen_navigator_display_config_message msg;
  IPC_RETURN_TYPE err;

  if (strncmp(attribute, "reset all", 9) == 0)
    {
      msg.attribute = attribute;
      msg.reset_all_to_defaults = 1;
    }
  else
    {
      msg.timestamp = carmen_get_time();
      msg.host = carmen_get_host();
      msg.attribute = attribute;
      msg.value = value;
      msg.reset_all_to_defaults = 0;
    }

  err = IPC_publishData(CARMEN_NAVIGATOR_DISPLAY_CONFIG_NAME, &msg);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_NAVIGATOR_DISPLAY_CONFIG_NAME);
}

int
main (int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  navigator_display_config("robot colour", 0xff << 16 | 218 << 8 | 185);
  sleep(3);
  navigator_display_config("robot colour", -1);
  navigator_display_config("track robot", 0);
  navigator_display_config("show particles", 1);
  sleep(3);
  navigator_display_config("reset all", 1);

  return 0;
}
