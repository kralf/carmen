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
#include "carmen_stdio.h"
#include "robot_messages.h"

#include "laserscans.h"
#include "gui.h"
#include "tools.h"
#include "history.h"

/********** Laser Scans **********/

int carmen_logger_nogz = 0;  //dbug
int carmen_playback_nogz = 0;  //dbug

char logfilename[1024];
//carmen_logger_file_p logfile;
carmen_FILE* logfile;

carmen_robot_laser_message* scan_list = NULL;
int num_scans = 0;
int *scan_mask = NULL;
int scan_range_min = 0;
int scan_range_mid = 0;
int scan_range_max = 0;
int scan_range_buf[9][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};

int scan_matching = 0;
int loading_scans = 0;
GMutex *laserscans_mutex = NULL;

double frontlaser_offset;

void laser_scans_shutdown() {

  if (scan_list)
    free(scan_list);
}

/* Display */

int unmasked_scan_display_ratio = 4;
int scan_display_ratio = 4;
/*static double scan_point_radius_ratio = */

typedef enum {
  SCAN_NORMAL,
  SCAN_RANGE,
  SCAN_FIXED,
  SCAN_RANGE_X
} scan_type;

static void display_scan(int scan, scan_type type) {

  int num_readings_per_scan;
  int reading;
  double x, y, theta;
  double lx, ly;
  double rx, ry, rtheta, rd;
  double px, py;
  double angle_delta;

  px = py = 0.0;
  num_readings_per_scan = scan_list[0].num_readings;

  x = scan_list[scan].laser_pose.x;
  y = scan_list[scan].laser_pose.y;
  theta = scan_list[scan].laser_pose.theta;

  if (type == SCAN_NORMAL)
    gdk_gc_set_foreground(drawing_gc, &gdkcolor_black);
  else if (type == SCAN_RANGE)
    gdk_gc_set_foreground(drawing_gc, &gdkcolor_red);
  else {
    if (type == SCAN_FIXED)
      gdk_gc_set_foreground(drawing_gc, &gdkcolor_blue);
    else if (type == SCAN_RANGE_X)
      gdk_gc_set_foreground(drawing_gc, &gdkcolor_green);
    px = (x / map_width) *
      (canvas_width < canvas_height ? canvas_width : canvas_height);
    py = (1.0 - (y / map_height)) *
      (canvas_width < canvas_height ? canvas_width : canvas_height) +
      (canvas_width < canvas_height) * (canvas_height - canvas_width);
    gdk_draw_arc(pixmap, drawing_gc, TRUE, px, py, 6, 6, 0,
		64 * carmen_radians_to_degrees(2.0 * M_PI));
  }

  lx = x; // + frontlaser_offset * cos(theta);
  ly = y; // + frontlaser_offset * sin(theta);

  /*printf("\n------------------------------------------------\n"); dbug*/
  /*printf("position of robot at scan %d:\n", scan); dbug*/
  /*printf("x = %g, y = %g, theta = %g, lx = %g, ly = %g\n", dbug*/
  /*	   x, y, theta, lx, ly); dbug*/

  angle_delta = scan_list[scan].config.angular_resolution;


  for (reading = 0; reading < num_readings_per_scan; reading++) {

    rtheta = theta + scan_list[scan].config.start_angle + reading * angle_delta;
    rd = scan_list[scan].range[reading];
    rx = lx + rd * cos(rtheta);
    ry = ly + rd * sin(rtheta);

    px = (rx / map_width) *
      (canvas_width < canvas_height ? canvas_width : canvas_height);
    py = (1.0 - (ry / map_height)) *
      (canvas_width < canvas_height ? canvas_width : canvas_height) +
      (canvas_width < canvas_height) * (canvas_height - canvas_width);

    /*printf("rtheta = %g, rd = %g, rx = %g, ry = %g\n", dbug*/
    /*	     rtheta, rd, rx, ry); dbug*/

    /*printf("plotting point at %g, %g\n", dbug*/
    /*	     (rx / map_width) * canvas_width, dbug*/
    /*	     (ry / map_height) * canvas_height); dbug*/

    if (rd < scan_list[scan].config.maximum_range)
      gdk_draw_point(pixmap, drawing_gc, px, py);
  }

  /*printf("--------------------------------------------------\n"); dbug*/
}

gint laser_scans_display(gpointer p __attribute__ ((unused))) {

  int scan, ignore = 0;

  if (!scan_list)
    return FALSE;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore)
    return FALSE;

  gdk_draw_rectangle(pixmap, canvas->style->white_gc, TRUE, 0, 0,
		     canvas_width, canvas_height);

  if (num_scans == 0)
    return FALSE;

  for (scan = 0; scan < num_scans; scan++) {
    if (scan == scan_range_min) {
      scan = scan_range_max;
      continue;
    }
    if (scan_mask[scan] == 0)
      continue;
    if ((scan % scan_display_ratio) && ((scan < scan_range_min - 10) ||
					(scan > scan_range_max + 10)))
      continue;
    display_scan(scan, SCAN_NORMAL);
  }

  if (scan_mask[scan_range_min]) {
    for (scan = scan_range_min; scan <= scan_range_max; scan++) {
      if (scan_mask[scan] == 0)
	continue;
      if ((scan % scan_display_ratio) && ((scan > scan_range_min + 10) &&
					  (scan < scan_range_max + 10)))
	continue;
      display_scan(scan, SCAN_RANGE);
    }
    display_scan(scan_range_min, SCAN_RANGE_X);
    display_scan(scan_range_max, SCAN_RANGE_X);
    display_scan(scan_range_mid, SCAN_FIXED);
  }

  gdk_draw_pixmap(canvas->window,
		  canvas->style->fg_gc[GTK_WIDGET_STATE(canvas)],
		  pixmap, 0, 0, 0, 0, canvas_width, canvas_height);

  return FALSE;
}

void center_map() {

  int num_readings_per_scan;
  int scan, reading;
  double x, y, theta;
  double lx, ly;
  double rx, ry, rtheta, rd;
  double minx, miny, maxx, maxy;
  double origin_offset_x, origin_offset_y;
  double angle_delta;

  num_readings_per_scan = scan_list[0].num_readings;

  minx = maxx = scan_list[0].laser_pose.x;
  miny = maxy = scan_list[0].laser_pose.y;

  for (scan = 0; scan < num_scans; scan++) {

    if (scan_mask && (scan_mask[scan] == 0))
      continue;

    x = scan_list[scan].laser_pose.x;
    y = scan_list[scan].laser_pose.y;
    theta = scan_list[scan].laser_pose.theta;

    lx = x + frontlaser_offset * cos(theta);
    ly = y + frontlaser_offset * sin(theta);

    angle_delta = scan_list[scan].config.angular_resolution;

    for (reading = 0; reading < num_readings_per_scan; reading++) {

      rtheta = theta + scan_list[scan].config.start_angle + reading * angle_delta;
      rd = scan_list[scan].range[reading];

      if (rd < scan_list[scan].config.maximum_range) {

	rx = lx + rd * cos(rtheta);
	ry = ly + rd * sin(rtheta);

	if (rx < minx)
	  minx = rx;
	else if (rx > maxx)
	  maxx = rx;
	else if (ry < miny)
	  miny = ry;
	else if (ry > maxy)
	  maxy = ry;
      }
    }
  }

  if (maxx - minx > maxy - miny) {
    map_width = map_height = maxx - minx;
    origin_offset_x = -minx;
    origin_offset_y = -miny + (map_height - maxy + miny) / 2.0;
  }
  else {
    map_width = map_height = maxy - miny;
    origin_offset_x = -minx + (map_width - maxx + minx) / 2.0;
    origin_offset_y = -miny;
  }

  origin_offset_x += .15 * map_width;
  origin_offset_y += .15 * map_height;
  map_width *= 1.3;
  map_height *= 1.3;

  for (scan = 0; scan < num_scans; scan++) {
    scan_list[scan].laser_pose.x += origin_offset_x;
    scan_list[scan].laser_pose.y += origin_offset_y;
  }

  /*
  fprintf(stderr, "map width = %g, map height = %g\n", map_width, map_height);
  fprintf(stderr, "origin x-offset = %g, origin y-offset = %g\n",
	  origin_offset_x, origin_offset_y);
  fprintf(stderr, "minx = %g, maxx = %g, miny = %g, maxy = %g\n",
 	 minx, maxx, miny, maxy);
  */
}


/***** Loading Laser Scans *****/

static char *next_word(char *str) {

  char *mark = str;

  while(mark[0] != '\0' && mark[0] != ' ')
    mark++;
  if(mark[0] == ' ')
    mark++;
  return mark;
}

/* read_next_laser_message - parses a log file, returning the next
   robot_laser_message struct */

static int read_next_laser_message(carmen_FILE* fp,
				   carmen_robot_laser_message *
				   robot_frontlaser)
{

  char message_name[100];
  char line[9999], *mark;
  int frontlaser_offset_flag = 1;

  while(!carmen_feof(fp)) {
    carmen_fgets(line, 9999, fp);
    if (sscanf(line, "%s", message_name) < 1)  // ignore bad sscanf's
      continue;
    if(strcmp(message_name, "FLASER") == 0) {
      carmen_string_to_robot_laser_message_orig(line, robot_frontlaser);
      return 1;
    } else if(strcmp(message_name, "ROBOTLASER1") == 0) {
      carmen_string_to_robot_laser_message(line, robot_frontlaser);
      return 1;
    } else if ((frontlaser_offset_flag) &&
	       (strcmp(message_name, "PARAM") == 0)) {
      mark = next_word(line);
      sscanf(mark, "%s", message_name);
      if (strcmp(message_name, "robot_frontlaser_offset") == 0) {
	sscanf(next_word(mark), "%lf", &frontlaser_offset);
	frontlaser_offset_flag = 0;
	/*printf("front laser offset = %g\n", frontlaser_offset);*/
      }
    }
  }
  return 0;
}

static gint load_logfile_end(gpointer p) {

  char buf[1024];
  int i, cancelled = (long) p;


  carmen_fclose(logfile);

  if (cancelled) {
    history_restore();
    status_print("Loading logfile...cancelled", "laserscans");
    g_idle_add(load_logfile, NULL);
  }
  else {
    scan_list = (carmen_robot_laser_message*) realloc(scan_list, num_scans *
					      sizeof(carmen_robot_laser_message));
    carmen_test_alloc(scan_list);
    sprintf(buf, "Loading logfile...read %d scans", num_scans);
    status_print(buf, "laserscans");

    scan_range_min = 0;
    scan_range_mid = 0;
    scan_range_max = num_scans-1;

    if (scan_mask)
      free(scan_mask);
    scan_mask = (int *) calloc(num_scans, sizeof(int));
    carmen_test_alloc(scan_mask);
    for (i = 0; i < num_scans; i++)
      scan_mask[i] = 1;

    if (scan_range_min_adj && scan_range_mid_adj && scan_range_max_adj) {
      (GTK_ADJUSTMENT(scan_range_min_adj))->lower = 0;
      (GTK_ADJUSTMENT(scan_range_min_adj))->upper = num_scans-1;
      (GTK_ADJUSTMENT(scan_range_mid_adj))->lower = 0;
      (GTK_ADJUSTMENT(scan_range_mid_adj))->upper = num_scans-1;
      (GTK_ADJUSTMENT(scan_range_max_adj))->lower = 0;
      (GTK_ADJUSTMENT(scan_range_max_adj))->upper = num_scans-1;
      gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_min_adj), scan_range_min);
      gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_mid_adj), scan_range_mid);
      gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_max_adj), scan_range_max);
      gtk_adjustment_changed(GTK_ADJUSTMENT(scan_range_min_adj));
      gtk_adjustment_changed(GTK_ADJUSTMENT(scan_range_mid_adj));
      gtk_adjustment_changed(GTK_ADJUSTMENT(scan_range_max_adj));
    }

    center_map();

    history_init();
    history_add();
  }

  g_mutex_lock(laserscans_mutex);
  loading_scans = 0;
  g_mutex_unlock(laserscans_mutex);
  g_mutex_free(laserscans_mutex);
  laserscans_mutex = NULL;

  gtk_progress_set_activity_mode(GTK_PROGRESS(progress_bar), FALSE);
  gtk_progress_set_percentage(GTK_PROGRESS(progress_bar), 0.0);

  laser_scans_display(NULL);

  return FALSE;
}

static gint load_scan(gpointer p)
{
  carmen_robot_laser_message front_laser;
  int array_length = *((int *)p);
  int cancel = 0;
  int valid;

  g_mutex_lock(laserscans_mutex);
  cancel = !loading_scans;
  g_mutex_unlock(laserscans_mutex);

  if (cancel) {
    g_idle_add(load_logfile_end, (gpointer) 1);
    free(p);
    return FALSE;
  }

  memset(&front_laser, 0, sizeof(carmen_robot_laser_message));

  if (num_scans % 25 == 0)
    gtk_progress_set_value(GTK_PROGRESS(progress_bar),
      gtk_progress_get_value(GTK_PROGRESS(progress_bar)) + 0.01);

  if (!carmen_feof(logfile)) {
    valid = read_next_laser_message(logfile, &front_laser);
    if(valid) {
      if(num_scans >= array_length) {
	array_length += 1000;
	if (scan_list) {
	  scan_list = (carmen_robot_laser_message*)
            realloc(scan_list, array_length * sizeof(carmen_robot_laser_message));
	  carmen_test_alloc(scan_list);
	}
	else {
	  scan_list = (carmen_robot_laser_message*)
	    calloc(array_length, sizeof(carmen_robot_laser_message));
	  carmen_test_alloc(scan_list);
	}
      }
      scan_list[num_scans].laser_pose.x = front_laser.laser_pose.x;
      scan_list[num_scans].laser_pose.y = front_laser.laser_pose.y;
      scan_list[num_scans].laser_pose.theta = front_laser.laser_pose.theta;
      scan_list[num_scans].num_readings = front_laser.num_readings;
      scan_list[num_scans].range = front_laser.range;
      scan_list[num_scans].config = front_laser.config;
      scan_list[num_scans].timestamp = front_laser.timestamp;
      num_scans++;
      carmen_erase_structure(&front_laser, sizeof(carmen_robot_laser_message));
    }

    *((int *) p) = array_length;
    return TRUE;
  }
  else {
    free(p);
    g_idle_add(load_logfile_end, 0);
    return FALSE;
  }
}

static gint load_logfile_begin(gpointer p __attribute__ ((unused))) {

  char buf[1024];
  int *array_length_p, cancel = 0;

  if (scan_list)
    free(scan_list);

  scan_list = NULL;
  num_scans = 0;

  gtk_progress_set_activity_mode(GTK_PROGRESS(progress_bar), TRUE);

  if (!laserscans_mutex)
    laserscans_mutex = g_mutex_new();

  g_mutex_lock(laserscans_mutex);
  cancel = loading_scans;
  loading_scans = !loading_scans;
  g_mutex_unlock(laserscans_mutex);

  if (cancel)
    return FALSE;

  logfile = carmen_fopen(logfilename, "r");
  if(logfile == NULL) {
    sprintf(buf, "Error: could not open file %s for reading.", logfilename);
    status_print(buf, "laserscans");
    return FALSE;
  }

  array_length_p = (int *) calloc(1, sizeof(int));
  carmen_test_alloc(array_length_p);
  *array_length_p = 0;
  g_idle_add(load_scan, array_length_p);

  return FALSE;
}

gint load_logfile(gpointer p __attribute__ ((unused))) {

  if(!carmen_file_exists(logfilename)) {
    status_print("Error: no such file.", "laserscans");
    return FALSE;
  }
  if(carmen_file_extension(logfilename) == NULL ||
     (strcmp(carmen_file_extension(logfilename), ".log") != 0 &&
      strcmp(carmen_file_extension(logfilename), ".gz") != 0 &&
      strcmp(carmen_file_extension(logfilename), ".clf") != 0) ) {
    status_print("Error: logfile must have .clf, .log or .gz extension.",
		 "laserscans");
    return FALSE;
  }

  status_print("Loading logfile...", "laserscans");
  g_idle_add(load_logfile_begin, NULL);

  return FALSE;
}

int save_logfile() {

  int i;
  char buf[128];

  if (scan_list == NULL) {
    status_print("Error: Nothing to save.", "laserscans");
    return -1;
  }

#ifndef NO_ZLIB
  if (strcmp(carmen_file_extension(logfilename), ".gz"))
    strcat(logfilename, ".gz");
#endif

  if ((logfile = carmen_fopen(logfilename, "w")) == NULL) {
    sprintf(buf, "Error: Couldn't save logfile: %s.", logfilename);
    status_print(buf, "laserscans");
    return -1;
  }

  sprintf(buf, "%.2f", frontlaser_offset);
  carmen_logwrite_write_param("robot", "frontlaser_offset", buf, scan_list[0].timestamp,
			      "nohost", logfile, scan_list[0].timestamp);

  for (i = 0; i < num_scans; i++) {
    if (!scan_mask[i])
      continue;

    scan_list[i].host = carmen_new_string("nohost");
    carmen_logwrite_write_robot_laser(&scan_list[i], 1, logfile, scan_list[i].timestamp);
  }

  carmen_fprintf(logfile, "\n");

  carmen_fclose(logfile);

  sprintf(buf, "Saved logfile: %s.", logfilename);
  status_print(buf, "laserscans");

  return 0;
}

void set_scan_range_min(GtkAdjustment *adj __attribute__ ((unused)),
			GtkObject *obj __attribute__ ((unused))) {

  int min, i, stop, ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching || loading_scans;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore) {
    (GTK_ADJUSTMENT(scan_range_min_adj))->value = scan_range_min;
    return;
  }

  min = (GTK_ADJUSTMENT(scan_range_min_adj))->value;

  if (min > scan_range_max)
    min = scan_range_max;

  if (scan_mask && !scan_mask[min]) {
    for (i = 1; 1; i++) {
      stop = 1;
      if (min-i >= 0 && min-i <= scan_range_max) {
	stop = 0;
	if (scan_mask[min-i]) {
	  min -= i;
	  break;
	}
      }
      if (min+i >= 0 && min+i <= scan_range_max) {
	stop = 0;
	if (scan_mask[min+i]) {
	  min += i;
	  break;
	}
      }
      if (stop) {
	min = 0;
	break;
      }
    }
  }

  (GTK_ADJUSTMENT(scan_range_min_adj))->value = scan_range_min = min;

  if (scan_mask && !scan_mask[scan_range_max])
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_max_adj), scan_range_max+1);

  if (scan_range_mid < scan_range_min)
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_mid_adj), scan_range_min);

  g_idle_add(laser_scans_display, NULL);
}

void set_scan_range_mid(GtkAdjustment *adj __attribute__ ((unused)),
			GtkObject *obj __attribute__ ((unused))) {

  int mid, i, stop, ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching || loading_scans;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore) {
    (GTK_ADJUSTMENT(scan_range_mid_adj))->value = scan_range_mid;
    return;
  }

  mid = (GTK_ADJUSTMENT(scan_range_mid_adj))->value;

  if (mid > scan_range_max)
    mid = scan_range_max;
  else if (mid < scan_range_min)
    mid = scan_range_min;

  if (scan_mask && !scan_mask[mid]) {
    for (i = 1; 1; i++) {
      stop = 1;
      if (mid+i >= scan_range_min && mid+i <= scan_range_max) {
	stop = 0;
	if (scan_mask[mid+i]) {
	  mid += i;
	  break;
	}
      }
      if (mid-i >= scan_range_min && mid-i <= scan_range_max) {
	stop = 0;
	if (scan_mask[mid-i]) {
	  mid -= i;
	  break;
	}
      }
      if (stop) {
	mid = scan_range_min;
	break;
      }
    }
  }

  (GTK_ADJUSTMENT(scan_range_mid_adj))->value = scan_range_mid = mid;

  g_idle_add(laser_scans_display, NULL);
}

void set_scan_range_max(GtkAdjustment *adj __attribute__ ((unused)),
			GtkObject *obj __attribute__ ((unused))) {

  int max, i, stop, ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching || loading_scans;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore) {
    (GTK_ADJUSTMENT(scan_range_max_adj))->value = scan_range_max;
    return;
  }

  max = (GTK_ADJUSTMENT(scan_range_max_adj))->value;

  if (max < scan_range_min)
    max = scan_range_min;

  if (scan_mask && !scan_mask[max]) {
    for (i = 1; 1; i++) {
      stop = 1;
      if (max+i >= scan_range_min && max+i < num_scans) {
	stop = 0;
	if (scan_mask[max+i]) {
	  max += i;
	  break;
	}
      }
      if (max-i >= scan_range_min && max-i < num_scans) {
	stop = 0;
	if (scan_mask[max-i]) {
	  max -= i;
	  break;
	}
      }
      if (stop) {
	max = num_scans-1;
	break;
      }
    }
  }

  (GTK_ADJUSTMENT(scan_range_max_adj))->value = scan_range_max = max;

  if (scan_mask && !scan_mask[scan_range_min])
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_min_adj), scan_range_min+1);

  if (scan_range_mid > scan_range_max)
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_mid_adj), scan_range_max);

  g_idle_add(laser_scans_display, NULL);
}

void set_scan_range(int min, int max) {

  if (min < 0 || max >= num_scans || min > max)
    return;

  if (scan_range_max >= min) {
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_min_adj), min);
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_max_adj), max);
  }
  else {
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_max_adj), max);
    gtk_adjustment_set_value(GTK_ADJUSTMENT(scan_range_min_adj), min);
  }
}

void scan_range_buf_store(int num) {

  char buf[128];

  scan_range_buf[num-1][0] = scan_range_min;
  scan_range_buf[num-1][1] = scan_range_max;

  sprintf(buf, "Stored scan range in buffer %d.", num);
  status_print(buf, "laserscans");
}

void scan_range_buf_recall(int num) {

  char buf[128];

  set_scan_range(scan_range_buf[num-1][0], scan_range_buf[num-1][1]);

  sprintf(buf, "Recalled scan range from buffer %d.", num);
  status_print(buf, "laserscans");
}

void laser_scans_update() {

  rotate_scans(carmen_normalize_theta
	       (scan_list_history[history_pos][scan_range_min].laser_pose.theta -
		scan_list[scan_range_min].laser_pose.theta));
  shift_scans(scan_list_history[history_pos][scan_range_min].laser_pose.x -
	      scan_list[scan_range_min].laser_pose.x,
	      scan_list_history[history_pos][scan_range_min].laser_pose.y -
	      scan_list[scan_range_min].laser_pose.y);

  center_map();
  history_add();

  laser_scans_display(NULL);
}

void laser_scans_restore() {

  history_restore();
}

void laser_scans_delete() {

  int i;

  if (scan_list == NULL)
    return;

  for (i = scan_range_min; i <= scan_range_max; i++)
    scan_mask[i] = 0;

  history_add();

  laser_scans_display(NULL);
}

/********** End Laser Scans **********/
