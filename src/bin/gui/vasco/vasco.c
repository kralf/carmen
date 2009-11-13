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

#include "vasco.h"
#include "gui.h"
#include "laserscans.h"

int main(int argc, char *argv[])
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  gtk_init(&argc, &argv);

  graphics_init();

  if (argc >= 2) {
    strncpy(logfilename, argv[1], 1023);
    load_logfile(NULL);
  }

  laser_scans_display(NULL);
  vascocore_init(argc, argv);

  gtk_main();

  return 0;
}
