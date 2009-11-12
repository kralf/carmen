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

#ifndef laserscans_h
#define laserscans_h

#include <carmen/robot_messages.h>

extern char logfilename[];

extern carmen_robot_laser_message* scan_list;
extern int num_scans;
extern int *scan_mask;
extern int scan_range_min, scan_range_mid, scan_range_max;
extern int scan_display_ratio, unmasked_scan_display_ratio;

extern GMutex *laserscans_mutex;
extern int scan_matching;
extern int loading_scans;

extern double frontlaser_offset;
extern double max_range;

void laser_scans_shutdown();

gint laser_scans_display(gpointer p);
void center_map();

void laser_scans_store_pose_init();
void laser_scans_update();
void laser_scans_restore();
void laser_scans_delete();

gint load_logfile(gpointer p);
int save_logfile();

void set_scan_range_min(GtkAdjustment *adj, GtkObject *obj);
void set_scan_range_mid(GtkAdjustment *adj, GtkObject *obj);
void set_scan_range_max(GtkAdjustment *adj, GtkObject *obj);
void set_scan_range(int min, int max);
void scan_range_buf_store(int num);
void scan_range_buf_recall(int num);

#endif
