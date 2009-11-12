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

#include "tools.h"
#include "laserscans.h"
#include "gui.h"

long tool = TOOL_NONE;

double button1_x, button1_y;

GtkWidget *shift_button;
GtkWidget *rotate_button;
GtkWidget *stretch_button;
GtkWidget *bend_button;
GtkWidget *zoom_in_button;
GtkWidget *zoom_out_button;

void shift_scans(double x, double y) {

  int scan;

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "tools");
    return;
  }

  if (!scan_mask[scan_range_min]) {
    status_print("Select a scan range first.", "tools");
    return;
  }

  for (scan = scan_range_min; scan <= scan_range_max; scan++) {
    if (scan_mask[scan]) {
      scan_list[scan].laser_pose.x += x;
      scan_list[scan].laser_pose.y += y;
    }
  }
}

void shift_scans_by_motion(int x, int y) {

  double x_1, y_1;

  x_1 = (x / (double) MIN(canvas_width, canvas_height)) * map_width;
  y_1 = (1 - (y / (double) MIN(canvas_width, canvas_height))) * map_height;

  shift_scans(x_1 - button1_x, y_1 - button1_y);

  button1_x = x_1;
  button1_y = y_1;
}

void rotate_scans(double radians) {

  int scan;
  double x_1, x_2, y_1, y_2, alpha, d;

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "tools");
    return;
  }

  if (!scan_mask[scan_range_min]) {
    status_print("Select a scan range first.", "tools");
    return;
  }

  x_1 = scan_list[scan_range_mid].laser_pose.x;
  y_1 = scan_list[scan_range_mid].laser_pose.y;

  for (scan = scan_range_min; scan <= scan_range_max; scan++) {

    if (scan_mask[scan]) {

      x_2 = scan_list[scan].laser_pose.x;
      y_2 = scan_list[scan].laser_pose.y;

      d = hypot(x_2 - x_1, y_2 - y_1);
      alpha = atan2(y_2 - y_1, x_2 - x_1);    
    
      scan_list[scan].laser_pose.x = x_1 + d * cos(alpha + radians);
      scan_list[scan].laser_pose.y = y_1 + d * sin(alpha + radians);
      scan_list[scan].laser_pose.theta =
	carmen_normalize_theta(scan_list[scan].laser_pose.theta + radians);
    }
  }
}

void rotate_scans_by_motion(int x, int y) {

  double x_1, y_1, x_2, y_2, alpha1, alpha2;

  x_1 = button1_x;
  y_1 = button1_y;
  x_2 = (x / (double) MIN(canvas_width, canvas_height)) * map_width;
  y_2 = (1 - (y / (double) MIN(canvas_width, canvas_height))) * map_height;

  alpha1 = atan2(y_1 - scan_list[scan_range_mid].laser_pose.y,
		 x_1 - scan_list[scan_range_mid].laser_pose.x);

  alpha2 = atan2(y_2 - scan_list[scan_range_mid].laser_pose.y,
		 x_2 - scan_list[scan_range_mid].laser_pose.x);

  /*
  printf("rotate_scans_by_motion: alpha1 = %g, alpha2 = %g\n",
    alpha1, alpha2);
  printf("                        x1 = %g, y1 = %g, x2 = %g, y2 = %g\n",
	 x_1, y_1, x_2, y_2);
  printf("                        fixed scan x = %g, fixed scan y = %g\n",
	 scan_list[scan_range_mid].x, scan_list[scan_range_mid].y);
  */

  rotate_scans(alpha2 - alpha1);

  button1_x = x_2;
  button1_y = y_2;
}

void stretch_scans(double x, double y) {

  int min, mid;

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "tools");
    return;
  }

  if (!scan_mask[scan_range_min]) {
    status_print("Select a scan range first.", "tools");
    return;
  }

  min = scan_range_min;
  mid = scan_range_mid;

  while (scan_range_min < mid) {
    scan_range_min++;
    scan_range_mid = scan_range_min;
    shift_scans(x / (double)(mid - min), y / (double)(mid - min));
  }

  scan_range_min = min;
  scan_range_mid = mid;
}

void stretch_scans_by_motion(int x, int y) {

  double x_1, y_1, x_2, y_2;

  x_1 = button1_x;
  y_1 = button1_y;
  x_2 = (x / (double) MIN(canvas_width, canvas_height)) * map_width;
  y_2 = (1 - (y / (double) MIN(canvas_width, canvas_height))) * map_height;

  stretch_scans(x_2 - x_1, y_2 - y_1);

  button1_x = x_2;
  button1_y = y_2;
}

void bend_scans(double radians) {

  int min, mid;

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "tools");
    return;
  }

  if (!scan_mask[scan_range_min]) {
    status_print("Select a scan range first.", "tools");
    return;
  }

  min = scan_range_min;
  mid = scan_range_mid;

  while (scan_range_min < mid) {
    scan_range_min++;
    scan_range_mid = scan_range_min;
    rotate_scans(2.0 * radians / (double)(mid - min));
  }

  scan_range_min = min;
  scan_range_mid = mid;
}

void bend_scans_by_motion(int x, int y) {

  double x_1, y_1, x_2, y_2, alpha1, alpha2;

  x_1 = button1_x;
  y_1 = button1_y;
  x_2 = (x / (double) MIN(canvas_width, canvas_height)) * map_width;
  y_2 = (1 - (y / (double) MIN(canvas_width, canvas_height))) * map_height;

  alpha1 = atan2(y_1 - scan_list[scan_range_min].laser_pose.y,
		 x_1 - scan_list[scan_range_min].laser_pose.x);

  alpha2 = atan2(y_2 - scan_list[scan_range_min].laser_pose.y,
		 x_2 - scan_list[scan_range_min].laser_pose.x);

  //  printf("bend_scans_by_motion((%.2f, %.2f) -> (%.2f, %.2f))\n",
  //	 x_1, y_1, x_2, y_2);
  //  printf("alpha1 = %.2f, alpha2 = %.2f\n", alpha1, alpha2);

  bend_scans(carmen_normalize_theta(alpha2 - alpha1));

  button1_x = x_2;
  button1_y = y_2;
}

void zoom_in(int x, int y) {

  GtkAdjustment *hadj, *vadj;
  double hratio, vratio;

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "tools");
    return;
  }

  hadj =
    gtk_scrolled_window_get_hadjustment(GTK_SCROLLED_WINDOW(scrolled_window));
  vadj =
    gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(scrolled_window));

  hratio = x / (double) canvas_width;
  vratio = y / (double) canvas_height;

  canvas_width *= 4.0/3.0;
  canvas_height *= 4.0/3.0;

  gtk_widget_set_usize(canvas, canvas_width, canvas_height);

  hadj->value = (hratio * canvas_width) - (hadj->page_size / 2);
  vadj->value = (vratio * canvas_height) - (vadj->page_size / 2);

  if (pixmap)
    gdk_pixmap_unref(pixmap);

  pixmap = gdk_pixmap_new(canvas->window, canvas_width, canvas_height, -1);

  gdk_draw_rectangle(pixmap, canvas->style->white_gc, TRUE, 0, 0,
		     canvas_width, canvas_height);

  gdk_draw_pixmap(canvas->window,
		  canvas->style->fg_gc[GTK_WIDGET_STATE(canvas)],
		  pixmap, 0, 0, 0, 0, canvas_width, canvas_height);

  laser_scans_display(NULL);
}

void zoom_out(int x, int y) {

  GtkAdjustment *hadj, *vadj;
  double hratio, vratio;

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "tools");
    return;
  }

  hadj =
    gtk_scrolled_window_get_hadjustment(GTK_SCROLLED_WINDOW(scrolled_window));
  vadj =
    gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(scrolled_window));

  hratio = x / (double) canvas_width;
  vratio = y / (double) canvas_height;

  canvas_width /= 4.0/3.0;
  canvas_height /= 4.0/3.0;

  gtk_widget_set_usize(canvas, canvas_width, canvas_height);

  hadj->value = (hratio * canvas_width) - (hadj->page_size / 2);
  vadj->value = (vratio * canvas_height) - (vadj->page_size / 2);

  if (pixmap)
    gdk_pixmap_unref(pixmap);

  pixmap = gdk_pixmap_new(canvas->window, canvas_width, canvas_height, -1);

  gdk_draw_rectangle(pixmap, canvas->style->white_gc, TRUE, 0, 0,
		     canvas_width, canvas_height);

  gdk_draw_pixmap(canvas->window,
		  canvas->style->fg_gc[GTK_WIDGET_STATE(canvas)],
		  pixmap, 0, 0, 0, 0, canvas_width, canvas_height);

  laser_scans_display(NULL);
}

void set_tool(GtkWidget *widget __attribute__ ((unused)), gpointer data) {

  int display = 1;

  if (tool != (long) data) {
    if (tool == TOOL_SHIFT)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(shift_button), FALSE);
    else if (tool == TOOL_ROTATE)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rotate_button), FALSE);
    else if (tool == TOOL_STRETCH)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(stretch_button), FALSE);
    else if (tool == TOOL_BEND)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(bend_button), FALSE);
    else if (tool == TOOL_ZOOM_IN)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(zoom_in_button), FALSE);
    else if (tool == TOOL_ZOOM_OUT)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(zoom_out_button), FALSE);
    tool = (long) data;
  }
  else
    tool = TOOL_NONE;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    display = !scan_matching && !loading_scans;
    g_mutex_unlock(laserscans_mutex);
  }

  if (display)
    g_idle_add(laser_scans_display, NULL);
}
