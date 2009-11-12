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
#include "laserscans.h"
#include "gui.h"


static gint end_scan_matching(gpointer data) {

  long cancelled = (long) data;

  g_mutex_lock(laserscans_mutex);
  scan_matching = 0;
  g_mutex_unlock(laserscans_mutex);

  gtk_progress_set_percentage(GTK_PROGRESS(progress_bar), 0.0);

  if (!cancelled) {
    laser_scans_update();
    status_print("Scan matching...complete", "scanmatch");
  }
  else {
    laser_scans_restore();
    status_print("Scan matching...cancelled", "scanmatch");
  }

  return FALSE;
}

static gint scan_match(gpointer first) {

  static int scan = 0;
  int stop;
  carmen_point_t pos, cpos;
  carmen_laser_laser_message laserscan;

  g_mutex_lock(laserscans_mutex);
  stop = !scan_matching;
  g_mutex_unlock(laserscans_mutex);
  if (stop)
    return FALSE;

  if ((long)first) {
    if (scan != 0)
      vascocore_reset();
    scan = scan_range_min;
  }

  while (scan_mask[scan] == 0 && scan <= scan_range_max)
    scan++;

  if (scan > scan_range_max) {
    g_idle_add(end_scan_matching, 0);
    return FALSE;
  }

  laserscan.config = scan_list[scan].config;
  laserscan.num_readings = scan_list[scan].num_readings;
  laserscan.range = scan_list[scan].range;
  laserscan.timestamp = 0.0;
  laserscan.host = (char *)calloc(1, sizeof(char));
  pos.x = scan_list[scan].laser_pose.x;
  pos.y = scan_list[scan].laser_pose.y;
  pos.theta = scan_list[scan].laser_pose.theta;

  cpos = vascocore_scan_match(laserscan, pos);

  scan_list[scan].laser_pose = cpos;
/*   scan_list[scan].x = cpos.x; */
/*   scan_list[scan].y = cpos.y; */
/*   scan_list[scan].theta = cpos.theta; */

  scan++;

  gtk_progress_set_percentage(GTK_PROGRESS(progress_bar),
			      (scan - scan_range_min) /
			      (double) (scan_range_max - scan_range_min + 1));

  while (gtk_events_pending())
    gtk_main_iteration_do(0);

  if ((long)first) {
    g_idle_add(scan_match, 0);
    return FALSE;
  }

  return TRUE;
}

static gint begin_scan_matching(gpointer data __attribute__ ((unused))) {

  g_mutex_lock(laserscans_mutex);
  scan_matching = 1;
  g_mutex_unlock(laserscans_mutex);

  status_print("Scan matching...", "scanmatch");

  g_idle_add(scan_match, (gpointer) 1);

  return FALSE;
}

void do_scan_matching() {

  int stop;

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "tools");
    return;
  }

  if (!laserscans_mutex)
    laserscans_mutex = g_mutex_new();

  g_mutex_lock(laserscans_mutex);
  stop = scan_matching;
  g_mutex_unlock(laserscans_mutex);

  if (!stop)
    g_idle_add(begin_scan_matching, NULL);
  else
    g_idle_add(end_scan_matching, (gpointer) 1);
}
