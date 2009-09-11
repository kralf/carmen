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
#include "localize_interface.h"

int main(int argc, char **argv)
{
  /* connect to IPC */
  carmen_ipc_initialize(argc, argv);

  if(argc < 2)
    carmen_die("Error: not enough arguments.\n"
	       "Usage: %s location\n"
	       "  Use 'global' for global localization.\n", argv[0]);
  if(strncmp(argv[1], "global", 6) == 0) {
    /* send uniform initialization command */
    carmen_localize_initialize_uniform_command();
  }
  else
    carmen_localize_initialize_placename_command(argv[1]);
  return 0;
}
