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
#include "map_io.h"

#include "egrid.h"
#include "vegrid.h"
#include "laserscans.h"
#include "gui.h"

int making_egrid = 0;
GMutex *egrid_mutex = NULL;

static evidence_grid *egrid = NULL;
//static carmen_laser_scan_p egrid_scan_list = NULL;
static carmen_robot_laser_message* egrid_scan_list = NULL;
static int egrid_num_scans = 0;

static char egrid_filename[1024];
static char egrid_origin[1024];

void egrid_shutdown() {

  if (egrid)
    carmen_mapper_free_evidence_grid(egrid);
}

/* Layout */

static int egrid_map_width;
static int egrid_map_height;
static int egrid_canvas_width;
static int egrid_canvas_height;
static double egrid_offset_x;
static double egrid_offset_y;

/* Parameters */

#define       DEFAULT_RESOLUTION            0.025
#define       DEFAULT_DOWNSAMPLE            4
#define       DEFAULT_START_ANGLE           0
#define       DEFAULT_WALL_THICKNESS        0.2
#define       DEFAULT_BORDER                20

#define       DEFAULT_POSITIVE_PROB         0.55
#define       DEFAULT_P_OCC                 0.5
#define       DEFAULT_NEGATIVE_PROB         0.45
#define       DEFAULT_MAX_PROB              0.95
#define       DEFAULT_CLOSE_RANGE           5.0
#define       DEFAULT_LONG_RANGE            10.0

#define       NUM_EGRID_PARAMS              11

#define       RESOLUTION                    0
#define       DOWNSAMPLE                    1
#define       START_ANGLE                   2
#define       WALL_THICKNESS                3
#define       BORDER                        4
#define       POSITIVE_PROB                 5
#define       P_OCC                         6
#define       NEGATIVE_PROB                 7
#define       MAX_PROB                      8
#define       CLOSE_RANGE                   9
#define       LONG_RANGE                    10

static double egrid_params[NUM_EGRID_PARAMS] =
  {DEFAULT_RESOLUTION, DEFAULT_DOWNSAMPLE, DEFAULT_START_ANGLE,
   DEFAULT_WALL_THICKNESS, DEFAULT_BORDER, DEFAULT_POSITIVE_PROB,
   DEFAULT_P_OCC, DEFAULT_NEGATIVE_PROB, DEFAULT_MAX_PROB,
   DEFAULT_CLOSE_RANGE, DEFAULT_LONG_RANGE};

static const char *egrid_param_strings[NUM_EGRID_PARAMS] =
  {"resolution (m)", "downsample", "start angle", "wall thickness (m)",
   "border", "occupied evidence", "occupied prior", "empty evidence",
   "max probability", "close range", "long range"};

static const unsigned int egrid_param_int_mask =
  (1 << DOWNSAMPLE) | (1 << BORDER);

/* Display */

static void egrid_dialog_init();
static void egrid_window_init();

static GtkWidget *egrid_window;
static GtkWidget *egrid_canvas;
static GtkWidget *egrid_file_save_window;
static GtkWidget *egrid_file_save_description_dialog;
static GtkWidget *egrid_dialog;
static GtkWidget *egrid_param_entries[NUM_EGRID_PARAMS];
static GtkWidget *egrid_param_labels[NUM_EGRID_PARAMS];
static GdkPixmap *egrid_pixmap = NULL;
static GtkWidget *egrid_progress_bar;

static void egrid_display() {

  if (!egrid)
    return;

  gdk_draw_pixmap(egrid_canvas->window,
	 	  egrid_canvas->style->fg_gc[GTK_WIDGET_STATE(egrid_canvas)],
		  egrid_pixmap, 0, 0, 0, 0, egrid_canvas_width,
		  egrid_canvas_height);
}

static void egrid_to_image() {

  int x, y, x2, y2;
  int c;
  GdkColor color;

  for(x = 0; x < egrid_canvas_width; x++) {
    for(y = 0; y < egrid_canvas_height; y++) {
      x2 = (int) ((x / (double) egrid_canvas_width) *
		  (egrid_map_width / 2.0) + egrid_offset_x);
      y2 = (int) ((1.0 - (y / (double) egrid_canvas_height)) *
		  (egrid_map_height / 2.0) + egrid_offset_y);
      /*
      if ((egrid_offset_x == 0) && ((y == 0) || y == egrid_canvas_height - 1))
	printf("x2 = %d, y2 = %d\n", x2, y2);
      */
      if(egrid->prob[x2][y2] == -1)
	gdk_gc_set_foreground(drawing_gc, &gdkcolor_blue);
      else {
	c = 255 - egrid->prob[x2][y2] * 255;
	color = carmen_graphics_add_color_rgb(c, c, c);
	gdk_gc_set_foreground(drawing_gc, &color);
      }
      /*
      if ((egrid_offset_x == 0) && ((y == 0) || y == egrid_canvas_height - 1))
	printf("---------\nx = %d, y = %d\n", x, y);
      */
      gdk_draw_point(egrid_pixmap, drawing_gc, x, y);
    }
  }
}

static void egrid_size_finish() {

  egrid_map_width = 2.0 * egrid->size_x;
  egrid_map_height = 2.0 * egrid->size_y;
  /*
  egrid_canvas_width = (int) egrid->size_x;
  egrid_canvas_height = (int) egrid->size_y;
  */
  egrid_offset_x = 0;
  egrid_offset_y = 0;
}

static gint scan_to_egrid(gpointer scan_num) {

  int scan = *((int *) scan_num);
  int stop;

  g_mutex_lock(egrid_mutex);
  stop = !making_egrid;
  g_mutex_unlock(egrid_mutex);

  if (stop) {
    free(scan_num);
    return FALSE;
  }

  if (scan < egrid_num_scans) {

    if (!(scan_mask && (scan_mask[scan] == 0))) {

      carmen_mapper_update_evidence_grid(egrid,
					 egrid_scan_list[scan].laser_pose.x,
					 egrid_scan_list[scan].laser_pose.y,
					 egrid_scan_list[scan].laser_pose.theta,
					 egrid_scan_list[scan].num_readings,
					 egrid_scan_list[scan].range,
					 egrid_scan_list[scan].config.angular_resolution,
					 egrid_scan_list[scan].config.start_angle);
      if (scan % 10 == 0) {
	gtk_progress_set_percentage(GTK_PROGRESS(egrid_progress_bar),
				    scan / (double) egrid_num_scans);
	if (scan % 50 == 0) {
	  egrid_to_image();
	  egrid_display();
	}
      }
    }

    (*((int *) scan_num))++;
    return TRUE;
  }

  else {

    egrid_to_image();
    egrid_display();

    /*  only use this function right before saving egrid to disk! */
    carmen_mapper_finish_evidence_grid(egrid, (int) egrid_params[DOWNSAMPLE],
				       (int) egrid_params[BORDER]);
    egrid_size_finish();
    egrid_to_image();
    egrid_display();

    gtk_progress_set_percentage(GTK_PROGRESS(egrid_progress_bar), 0.0);
    status_print("Making evidence grid...completed", "vegrid");

    g_mutex_lock(egrid_mutex);
    making_egrid = 0;
    g_mutex_unlock(egrid_mutex);

    free(scan_num);

    return FALSE;
  }
}

static void egrid_size() {

  int scan, reading, num_readings_per_scan;
  double x, y, theta;
  double lx, ly;
  double rx, ry, rtheta, rd;
  double maxx, maxy, minx, miny;
  double angle_delta;

  scan = 0;
  maxx = minx = egrid_scan_list[0].laser_pose.x;
  maxy = miny = egrid_scan_list[0].laser_pose.y;
  num_readings_per_scan = egrid_scan_list[0].num_readings;

  for (scan = 0; scan < egrid_num_scans; scan++) {

    if (scan_mask && (scan_mask[scan] == 0))
      continue;

    x = egrid_scan_list[scan].laser_pose.x;
    y = egrid_scan_list[scan].laser_pose.y;
    theta = egrid_scan_list[scan].laser_pose.theta;

    lx = x + frontlaser_offset * cos(theta);
    ly = y + frontlaser_offset * sin(theta);

    angle_delta = egrid_scan_list[scan].config.angular_resolution;

    for (reading = 0; reading < num_readings_per_scan; reading++) {

      rtheta = theta + egrid_scan_list[scan].config.start_angle + reading * angle_delta;
      rd = egrid_scan_list[scan].range[reading];

      if (rd < egrid_scan_list[scan].config.maximum_range) {

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

  egrid_map_width =
    (int) (2.0 * (maxx - minx) / egrid_params[RESOLUTION]);
  egrid_map_height =
    (int) (2.0 * (maxy - miny) / egrid_params[RESOLUTION]);

  egrid_canvas_width = (int) ((maxx - minx) / (egrid_params[RESOLUTION] *
					       egrid_params[DOWNSAMPLE]) +
			      egrid_params[BORDER]);
  egrid_canvas_height =  (int) ((maxy - miny) / (egrid_params[RESOLUTION] *
						 egrid_params[DOWNSAMPLE]) +
				egrid_params[BORDER]);

  egrid_offset_x = (maxx - egrid_scan_list[0].laser_pose.x) / egrid_params[RESOLUTION] +
    egrid_params[BORDER] * egrid_params[RESOLUTION];
  egrid_offset_y = (maxy - egrid_scan_list[0].laser_pose.y) / egrid_params[RESOLUTION] +
    egrid_params[BORDER] * egrid_params[RESOLUTION];
}

static gint scans_to_egrid(gpointer p __attribute__ ((unused))) {

  int *scan_p = (int *) calloc(1, sizeof(int));
  carmen_test_alloc(scan_p);
  *scan_p = 0;

  status_print("Making evidence grid...", "vegrid");

  if (!egrid_mutex)
    egrid_mutex = g_mutex_new();

  g_mutex_lock(egrid_mutex);
  making_egrid = 1;
  g_mutex_unlock(egrid_mutex);

  egrid_num_scans = num_scans;

  if (!egrid_scan_list) {
    egrid_scan_list = (carmen_robot_laser_message*)
      calloc(egrid_num_scans, sizeof(carmen_robot_laser_message));
    carmen_test_alloc(egrid_scan_list);
  }

  memcpy(egrid_scan_list, scan_list,
	 egrid_num_scans * sizeof(carmen_robot_laser_message));

  strncpy(egrid_origin, logfilename, 1023);

  egrid_size();

  if (!egrid_window)
    egrid_window_init();
  else
    gtk_widget_set_usize(egrid_canvas,
			 egrid_canvas_width, egrid_canvas_height);

  gtk_widget_show_all(egrid_window);

  if (egrid_pixmap)
    gdk_pixmap_unref(egrid_pixmap);

  egrid_pixmap =
    gdk_pixmap_new(egrid_canvas->window, egrid_canvas_width,
		   egrid_canvas_height, -1);

  gdk_draw_rectangle(egrid_pixmap, egrid_canvas->style->white_gc, TRUE, 0, 0,
		     egrid_canvas_width, egrid_canvas_height);

  if (!egrid) {
    egrid = (evidence_grid *) calloc(1, sizeof(evidence_grid));
    carmen_test_alloc(egrid);
  } else
    carmen_mapper_free_evidence_grid(egrid);

  carmen_mapper_initialize_evidence_grid(egrid, egrid_map_width,
				       egrid_map_height,
				       egrid_params[RESOLUTION],
				       egrid_params[START_ANGLE],
				       egrid_params[P_OCC],
				       egrid_params[POSITIVE_PROB],
				       egrid_params[NEGATIVE_PROB],
				       egrid_params[MAX_PROB],
				       egrid_params[CLOSE_RANGE],
				       egrid_params[LONG_RANGE],
				       egrid_params[WALL_THICKNESS]);

  g_idle_add(scan_to_egrid, scan_p);

  return FALSE;
}

void egrid_dialog_popup(GtkWidget *w __attribute__ ((unused)),
			gpointer p __attribute__ ((unused))) {

  int param, show_window;
  char buf[1000];

  if (!scan_list) {
    status_print("Ctrl-O opens a logfile", "vegrid");
    return;
  }

  if (egrid_mutex) {
    g_mutex_lock(egrid_mutex);
    show_window = !making_egrid;
    g_mutex_unlock(egrid_mutex);
    if (!show_window)
      return;
  }

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    show_window = !scan_matching && !loading_scans;
    g_mutex_unlock(laserscans_mutex);
    if (!show_window)
      return;
  }

  if (!egrid_dialog)
    egrid_dialog_init();

  egrid_params[START_ANGLE] = scan_list[0].laser_pose.theta;

  for (param = 0; param < NUM_EGRID_PARAMS; param++) {
    if (param == START_ANGLE)
      continue;
    if (egrid_param_int_mask & (1 << param))
      sprintf(buf, "%d", (int) egrid_params[param]);
    else
      sprintf(buf, "%g", egrid_params[param]);
    gtk_entry_set_text(GTK_ENTRY(egrid_param_entries[param]), buf);
  }

  gtk_widget_show_all(egrid_dialog);
}

static int check_egrid_entries() {

  char *s;
  int param, entry_mask, dot, bad;

  entry_mask = dot = bad = 0;

  for (param = 0; param < NUM_EGRID_PARAMS; dot = bad = 0, param++) {

    if (param == START_ANGLE)
      continue;

    s = gtk_editable_get_chars(GTK_EDITABLE(egrid_param_entries[param]),
			       0, -1);
    while (*s == ' ')
      s++;
    if (*s == '-')
      s++;
    for (; *s != '\0'; s++) {
      if ((*s == '.') && !dot)
	dot = 1;
      else if (*s == ' ')
	break;
      else if (!isdigit(*s)) {
	entry_mask |= 1 << param;
	bad = 1;
	break;
      }
    }
    if (!bad) {
      while(*s == ' ')
	s++;
      if (*s != '\0')
	entry_mask |= 1 << param;
    }
  }

  return entry_mask;
}

static void egrid_dialog_submit(GtkWidget *w __attribute__ ((unused)),
				gpointer p __attribute__ ((unused))) {
  GtkEditable *e;
  int param;
  int entry_mask;

  entry_mask = check_egrid_entries();

  if (entry_mask) {
    for (param = 0; param < NUM_EGRID_PARAMS; param++) {
      if (param == START_ANGLE)
	continue;
      if (entry_mask & (1 << param)) {
	gtk_editable_select_region(GTK_EDITABLE(egrid_param_entries[param]),
				   0, -1);
      }
    }
  }
  else {
    for (param = 0; param < NUM_EGRID_PARAMS; param++) {
      if (param == START_ANGLE)
	continue;
      e = GTK_EDITABLE(egrid_param_entries[param]);
      if (egrid_param_int_mask & (1 << param))
	egrid_params[param] = (double) atoi(gtk_editable_get_chars(e, 0, -1));
      else
	egrid_params[param] = atof(gtk_editable_get_chars(e, 0, -1));
    }

    gtk_widget_hide(egrid_dialog);

    g_idle_add(scans_to_egrid, NULL);
  }
}

static void egrid_dialog_cancel(GtkWidget *w __attribute__ ((unused)),
				gpointer p __attribute__ ((unused))) {

  gtk_widget_hide(egrid_dialog);
}

static void egrid_dialog_destroy(GtkWidget *widget __attribute__ ((unused)),
				 gpointer p __attribute__ ((unused))) {

  egrid_dialog = NULL;
}

int save_egrid(const gchar *description) {

  carmen_FILE *fp;
  char *filename = egrid_filename;
  char buf[1024];

#ifndef NO_ZLIB
  char *s = carmen_file_extension(filename);
  if (strcmp(s, ".gz")) {
    *(s + strcspn(s, " \t")) = '\0';
    s += strlen(s);
    strcpy(s, ".gz");
  }
#endif

  fp = carmen_fopen(filename, "w");
  if(fp == NULL) {
    sprintf(buf, "Error: Could not open file %s for writing.", filename);
    status_print(buf, "vegrid");
    return -1;
  }
  if(carmen_map_write_all(fp, egrid->prob, egrid->size_x, egrid->size_y,
			  egrid->resolution, egrid_origin,
			  (gchar *)description, "", "", NULL, 0, NULL,
			  0, NULL, 0) < 0) {
    carmen_fclose(fp);
    return -1;
  }
  carmen_fclose(fp);
  return 0;
}

static void egrid_file_save_window_destroy(GtkWidget *w
					   __attribute__ ((unused)),
					   gpointer p
					   __attribute__ ((unused))) {
  egrid_file_save_window = NULL;
}

static gint egrid_window_expose(GtkWidget *widget, GdkEventExpose *event) {

  if (egrid_pixmap)
    gdk_draw_pixmap(widget->window,
		    widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
		    egrid_pixmap, event->area.x, event->area.y,
		    event->area.x, event->area.y,
		    event->area.width, event->area.height);

  return TRUE;
}

static void egrid_dialog_init() {

  GtkWidget *egrid_table;
  GtkWidget *egrid_ok_button;
  GtkWidget *egrid_cancel_button;
  char buf[1000];
  int param;

  egrid_dialog = gtk_dialog_new();
  gtk_window_set_policy(GTK_WINDOW(egrid_dialog), FALSE, FALSE, TRUE);
  egrid_table = gtk_table_new(NUM_EGRID_PARAMS, 2, FALSE);

  for (param = 0; param < NUM_EGRID_PARAMS; param++) {
    if (param == START_ANGLE)
      continue;
    sprintf(buf, " %s: ", egrid_param_strings[param]);
    egrid_param_labels[param] = gtk_label_new(buf);
    egrid_param_entries[param] = gtk_entry_new();
  }

  egrid_ok_button = gtk_button_new_with_label(" Ok ");
  egrid_cancel_button = gtk_button_new_with_label(" Cancel ");

  gtk_signal_connect(GTK_OBJECT(egrid_ok_button), "clicked",
		     GTK_SIGNAL_FUNC(egrid_dialog_submit), NULL);
  gtk_signal_connect(GTK_OBJECT(egrid_cancel_button), "clicked",
		     GTK_SIGNAL_FUNC(egrid_dialog_cancel), NULL);

  for (param = 0; param < NUM_EGRID_PARAMS; param++) {
    if (param == START_ANGLE)
      continue;
    gtk_table_attach(GTK_TABLE(egrid_table), egrid_param_labels[param],
		     0, 1, param, param + 1, GTK_FILL, GTK_FILL, 10, 2);
    gtk_table_attach(GTK_TABLE(egrid_table), egrid_param_entries[param],
		     1, 2, param, param + 1, GTK_FILL, GTK_FILL, 10, 2);
  }

  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(egrid_dialog)->vbox),
		     egrid_table, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(egrid_dialog)->action_area),
		     egrid_ok_button, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(egrid_dialog)->action_area),
		     egrid_cancel_button, TRUE, TRUE, 0);

  gtk_signal_connect(GTK_OBJECT(egrid_dialog), "destroy",
		     GTK_SIGNAL_FUNC(egrid_dialog_destroy), NULL);
}

static void
egrid_file_save_description_dialog_destroy(GtkWidget *w
					   __attribute__ ((unused)),
					   gpointer p
					   __attribute__ ((unused))) {

  egrid_file_save_description_dialog = NULL;
}

static void
egrid_file_save_description_dialog_ok_sel(GtkWidget *w
					   __attribute__ ((unused)),
					  GtkEntry *entry) {
  char buf[1024];

  if (!save_egrid(gtk_entry_get_text(entry))) {
    sprintf(buf, "saved map to file %s.", egrid_filename);
    status_print(buf, "vegrid");
  }

  gtk_widget_hide(egrid_file_save_description_dialog);
}

static void
egrid_file_save_description_dialog_cancel_sel(GtkWidget *w
					      __attribute__ ((unused)),
					      gpointer p
					      __attribute__ ((unused))) {

  gtk_widget_hide(egrid_file_save_description_dialog);
}

static GtkWidget *egrid_file_save_description_dialog_init() {

  GtkWidget *label, *entry;
  GtkWidget *ok_button, *cancel_button;

  egrid_file_save_description_dialog = gtk_dialog_new();
  label = gtk_label_new(" Enter a description: ");
  entry = gtk_entry_new_with_max_length(1000);
  ok_button = gtk_button_new_with_label(" Ok ");
  cancel_button = gtk_button_new_with_label(" Cancel ");

  gtk_box_pack_start(
    GTK_BOX(GTK_DIALOG(egrid_file_save_description_dialog)->vbox), label,
    TRUE, TRUE, 5);
  gtk_box_pack_start(
    GTK_BOX(GTK_DIALOG(egrid_file_save_description_dialog)->vbox), entry,
    TRUE, TRUE, 5);
  gtk_box_pack_start(
    GTK_BOX(GTK_DIALOG(egrid_file_save_description_dialog)->action_area),
    ok_button, TRUE, TRUE, 0);
  gtk_box_pack_start(
    GTK_BOX(GTK_DIALOG(egrid_file_save_description_dialog)->action_area),
    cancel_button, TRUE, TRUE, 0);

  gtk_signal_connect(GTK_OBJECT(egrid_file_save_description_dialog), "destroy",
    GTK_SIGNAL_FUNC(egrid_file_save_description_dialog_destroy), NULL);
  gtk_signal_connect(GTK_OBJECT(ok_button), "clicked",
    GTK_SIGNAL_FUNC(egrid_file_save_description_dialog_ok_sel), entry);
  gtk_signal_connect(GTK_OBJECT(cancel_button), "clicked",
    GTK_SIGNAL_FUNC(egrid_file_save_description_dialog_cancel_sel), NULL);

  return egrid_file_save_description_dialog;
}

static gint
egrid_file_save_description_dialog_popup(gpointer p
					 __attribute__ ((unused))) {

  if (!egrid_file_save_description_dialog)
    egrid_file_save_description_dialog_init();

  gtk_widget_show_all(egrid_file_save_description_dialog);

  return FALSE;
}

static int test_filename() {

  char *s = egrid_filename + strlen(egrid_filename) - 1;

  while (*s == ' ')
    s--;

  return (*s != '/');
}

static void egrid_file_ok_sel(GtkWidget *w __attribute__ ((unused)),
			      gpointer p __attribute__ ((unused))) {

  sprintf(egrid_filename, "%s",
	  gtk_file_selection_get_filename(
	    GTK_FILE_SELECTION(egrid_file_save_window)));

  if(!test_filename()) {
    status_print("Error: You must type a file name.", "vegrid");
    return;
  }

  gtk_widget_hide(egrid_file_save_window);
  g_idle_add(egrid_file_save_description_dialog_popup, NULL);
}

static void egrid_file_cancel_sel(GtkWidget *w __attribute__ ((unused)),
				  gpointer p __attribute__ ((unused))) {

  gtk_widget_hide(egrid_file_save_window);
}

static void egrid_window_destroy(GtkWidget *widget __attribute__ ((unused)),
				 gpointer p __attribute__ ((unused))) {

  g_mutex_lock(egrid_mutex);
  if (making_egrid)
    status_print("Making evidence grid...cancelled", "vegrid");
  making_egrid = 0;
  g_mutex_unlock(egrid_mutex);

  egrid_window = NULL;
}

static GtkWidget *egrid_file_save_window_init() {

  egrid_file_save_window = gtk_file_selection_new("Save File");

  gtk_signal_connect(GTK_OBJECT(egrid_file_save_window), "destroy",
		     GTK_SIGNAL_FUNC(egrid_file_save_window_destroy), NULL);

  gtk_signal_connect(
	 GTK_OBJECT(GTK_FILE_SELECTION(egrid_file_save_window)->ok_button),
	 "clicked", GTK_SIGNAL_FUNC(egrid_file_ok_sel), NULL);

  gtk_signal_connect(
	 GTK_OBJECT(GTK_FILE_SELECTION(egrid_file_save_window)->cancel_button),
	 "clicked", GTK_SIGNAL_FUNC(egrid_file_cancel_sel), NULL);

  return egrid_file_save_window;
}

static void egrid_file_save(GtkWidget *w __attribute__ ((unused)),
			    gpointer p __attribute__ ((unused))) {

  int show_window;

  g_mutex_lock(egrid_mutex);
  show_window = !making_egrid;
  g_mutex_unlock(egrid_mutex);

  if (!egrid_file_save_window)
    egrid_file_save_window_init();
  if (show_window)
    gtk_widget_show(egrid_file_save_window);
}

static void egrid_window_close() {

  g_mutex_lock(egrid_mutex);
  if (making_egrid)
    status_print("Making evidence grid...cancelled", "vegrid");
  making_egrid = 0;
  g_mutex_unlock(egrid_mutex);

  if (egrid_window)
    gtk_widget_hide(egrid_window);
  if (egrid_file_save_window)
    gtk_widget_hide(egrid_file_save_window);
  if (egrid_file_save_description_dialog)
    gtk_widget_hide(egrid_file_save_description_dialog);
}

static GtkWidget *egrid_menubar_init()
{
  GtkActionEntry action_entries[] = {
    {"FileMenu", NULL, "_File", NULL, NULL, NULL},
    {"Save", GTK_STOCK_SAVE, "_Save", "<control>S", NULL,
     G_CALLBACK(egrid_file_save)},
    {"Close", GTK_STOCK_QUIT, "_Close", "<control>C", NULL,
     G_CALLBACK(egrid_window_close)},
  };

  const char *ui_description =
    "<ui>"
    "  <menubar name='MainMenu'>"
    "    <menu action='FileMenu'>"
    "      <menuitem action='Save'/>"
    "      <separator/>"
    "      <menuitem action='Close'/>"
    "    </menu>"
    "  </menubar>"
    "</ui>";

  GtkWidget *menubar;
  GtkActionGroup *action_group;
  GtkAccelGroup *accel_group;
  GError *error;
  GtkUIManager *ui_manager;

  action_group = gtk_action_group_new ("MenuActions");
  gtk_action_group_add_actions (action_group, action_entries,
				G_N_ELEMENTS (action_entries), egrid_window);
  ui_manager = gtk_ui_manager_new ();
  gtk_ui_manager_insert_action_group (ui_manager, action_group, 0);

  accel_group = gtk_ui_manager_get_accel_group (ui_manager);
  gtk_window_add_accel_group (GTK_WINDOW (egrid_window), accel_group);

  error = NULL;
  if (!gtk_ui_manager_add_ui_from_string (ui_manager, ui_description, -1,
					  &error)) {
    g_message ("building menus failed: %s", error->message);
    g_error_free (error);
    exit (EXIT_FAILURE);
  }

  menubar = gtk_ui_manager_get_widget (ui_manager, "/MainMenu");

  return menubar;
}

static GtkWidget *egrid_statusbar_init() {

  GtkWidget *hbox;

  hbox = gtk_hbox_new(FALSE, 0);
  egrid_progress_bar = gtk_progress_bar_new();
  gtk_box_pack_start(GTK_BOX(hbox), egrid_progress_bar, TRUE, TRUE, 5);

  return hbox;
}

static void egrid_window_init() {

  GtkWidget *vbox;

  egrid_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_policy(GTK_WINDOW(egrid_window), FALSE, FALSE, TRUE);
  gtk_window_set_title(GTK_WINDOW(egrid_window), "Evidence Grid Display");
  gtk_container_set_border_width(GTK_CONTAINER(egrid_window), 0);

  vbox = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), egrid_menubar_init(), FALSE, TRUE, 0);

  egrid_canvas = gtk_drawing_area_new();
  gtk_widget_set_usize(egrid_canvas, egrid_canvas_width, egrid_canvas_height);
  gtk_box_pack_start(GTK_BOX(vbox), egrid_canvas, FALSE, FALSE, 0);

  gtk_box_pack_start(GTK_BOX(vbox), egrid_statusbar_init(), TRUE, TRUE, 5);

  gtk_container_add(GTK_CONTAINER(egrid_window), vbox);

  gtk_signal_connect(GTK_OBJECT(egrid_window), "destroy",
		     GTK_SIGNAL_FUNC(egrid_window_destroy), NULL);
  gtk_signal_connect(GTK_OBJECT(egrid_canvas), "expose_event",
		     GTK_SIGNAL_FUNC(egrid_window_expose), NULL);
}
