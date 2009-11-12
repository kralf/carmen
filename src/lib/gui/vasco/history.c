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

#include "history.h"
#include "laserscans.h"

#include "robot_messages.h"

carmen_robot_laser_message* scan_list_history[HISTORY_SIZE];
int history_pos = 0;
static int *scan_mask_history[HISTORY_SIZE];
static int scan_display_ratio_history[HISTORY_SIZE];
static int history_start_pos = 0;
static int history_end_pos = 0;

void history_init() {

  int i;

  for (i = 0; i < HISTORY_SIZE; i++) {
    scan_list_history[i] =
      (carmen_robot_laser_message*) calloc(num_scans, sizeof(carmen_robot_laser_message));
    carmen_test_alloc(scan_list_history[i]);
  }
}

void history_shutdown() {

  int i;

  for (i = 0; i < HISTORY_SIZE; i++)
    free(scan_list_history[i]);
}

void history_restore() {

  memcpy(scan_list, scan_list_history[history_pos],
	 num_scans * sizeof(carmen_robot_laser_message));
}

void history_undo(GtkWidget *w __attribute__ ((unused)),
		  gpointer p __attribute__ ((unused))) {
  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching || loading_scans;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore)
    return;

  if (history_pos != history_start_pos) {
    history_pos--;
    if (history_pos < 0)
      history_pos += HISTORY_SIZE;
    memcpy(scan_list, scan_list_history[history_pos],
	   num_scans * sizeof(carmen_robot_laser_message));
    if (scan_mask_history[history_pos])
      memcpy(scan_mask, scan_mask_history[history_pos],
	     num_scans * sizeof(int));
    scan_display_ratio = scan_display_ratio_history[history_pos];
  }

  center_map();
  laser_scans_display(NULL);
}

void history_redo(GtkWidget *w __attribute__ ((unused)),
		  gpointer p __attribute__ ((unused))) {
  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching || loading_scans;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore)
    return;

  if (history_pos != history_end_pos) {
    history_pos++;
    history_pos %= HISTORY_SIZE;
    memcpy(scan_list, scan_list_history[history_pos],
	   num_scans * sizeof(carmen_robot_laser_message));
    if (scan_mask_history[history_pos])
      memcpy(scan_mask, scan_mask_history[history_pos],
	     num_scans * sizeof(int));
    scan_display_ratio = scan_display_ratio_history[history_pos];
  }

  center_map();
  laser_scans_display(NULL);
}

void history_add() {

  static int history_is_empty = 1;

  history_pos++;
  history_pos %= HISTORY_SIZE;
  history_end_pos = history_pos;

  if (history_start_pos == history_pos) {
    history_start_pos++;
    history_start_pos %= HISTORY_SIZE;
  }

  if (history_is_empty) {
    history_start_pos++;
    history_start_pos %= HISTORY_SIZE;
    history_is_empty = 0;
  }

  memcpy(scan_list_history[history_pos], scan_list,
	 num_scans * sizeof(carmen_robot_laser_message));
  if (scan_mask_history[history_pos])
    free(scan_mask_history[history_pos]);
  if (scan_mask) {
    scan_mask_history[history_pos] = (int *) calloc(num_scans, sizeof(int));
    carmen_test_alloc(scan_mask_history[history_pos]);
    memcpy(scan_mask_history[history_pos], scan_mask,
	   num_scans * sizeof(int));
  }
  else
    scan_mask_history[history_pos] = NULL;
  scan_display_ratio_history[history_pos] = scan_display_ratio;
}
