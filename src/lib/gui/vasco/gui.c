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

#include <gdk/gdkkeysyms.h>

#include "global_graphics.h"

#include "gui.h"
#include "vscanmatch.h"
#include "vegrid.h"
#include "tools.h"
#include "laserscans.h"
#include "history.h"

GdkGC *drawing_gc = NULL;

int button1 = 0;

GdkColor gdkcolor_black;
GdkColor gdkcolor_red;
GdkColor gdkcolor_blue;
GdkColor gdkcolor_green;

GtkWidget *canvas;
GtkWidget *scrolled_window;
GdkPixmap *pixmap = NULL;

int canvas_width = DEFAULT_DRAWING_AREA_WIDTH;
int canvas_height = DEFAULT_DRAWING_AREA_HEIGHT;

double map_width;
double map_height;

GtkWidget *scan_range_min_scale;
GtkWidget *scan_range_mid_scale;
GtkWidget *scan_range_max_scale;
GtkObject *scan_range_min_adj = NULL;
GtkObject *scan_range_mid_adj = NULL;
GtkObject *scan_range_max_adj = NULL;
GtkWidget *scan_range_min_spin;
GtkWidget *scan_range_mid_spin;
GtkWidget *scan_range_max_spin;

GtkWidget *status_bar;
GtkWidget *progress_bar;

GtkWidget *help_window = NULL;

static const unsigned int status_message_max_size = 44;

static void check_status_message(char *checked_message, const char *message) {

  strncpy(checked_message + 1, message, status_message_max_size);
  checked_message[0] = ' ';
  checked_message[status_message_max_size + 1] = '\0';

  for  (; *checked_message != '\0'; checked_message++)
    if (!isprint(*checked_message))
      *checked_message = ' ';
}

guint status_print(const char *message, const char *context) {

  char checked_message[1024];
  guint message_id, context_id;

  check_status_message(checked_message, message);

  context_id = gtk_statusbar_get_context_id(GTK_STATUSBAR(status_bar),
					    context);
  message_id = gtk_statusbar_push(GTK_STATUSBAR(status_bar),
				  context_id, checked_message);
  return message_id;
}

static void global_graphics_init() { /* canvas must already be initialized! */

  GtkWidget *tmp;

  if (drawing_gc == NULL) {
    tmp = canvas;
    drawing_gc = gdk_gc_new(tmp->window);
    gdkcolor_black = carmen_graphics_add_color("black");
    gdkcolor_red = carmen_graphics_add_color("red");
    gdkcolor_blue = carmen_graphics_add_color("blue");
    gdkcolor_green = carmen_graphics_add_color("green");
  }
}

void click_scan_match_button(GtkWidget *widget __attribute__ ((unused)),
			     gpointer data __attribute__ ((unused))) {
  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = loading_scans;
    g_mutex_unlock(laserscans_mutex);
  }

  if (!ignore)
    do_scan_matching();
}

static GtkWidget *file_window = NULL;

static void main_window_destroy(GtkWidget *widget __attribute__ ((unused)),
				gpointer data __attribute__ ((unused))) {

  laser_scans_shutdown();
  egrid_shutdown();
  history_shutdown();
  /*fprintf(stderr, "\n");*/
  gtk_main_quit();
}

static gint main_window_configure(GtkWidget *widget,
				  gpointer p __attribute__ ((unused))) {

  int display = 1;

  canvas_width = widget->allocation.width;
  canvas_height = widget->allocation.height;

  if (pixmap)
    gdk_pixmap_unref(pixmap);

  pixmap = gdk_pixmap_new(widget->window, canvas_width, canvas_height, -1);

  gdk_draw_rectangle(pixmap, widget->style->white_gc, TRUE, 0, 0,
		     canvas_width, canvas_height);

  gdk_draw_pixmap(widget->window,
		  widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
		  pixmap, 0, 0, 0, 0, canvas_width, canvas_height);

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    display = !loading_scans && !scan_matching;
    g_mutex_unlock(laserscans_mutex);
  }
  if (display)
    laser_scans_display(NULL);

  return TRUE;
}

static gint main_window_expose(GtkWidget *widget, GdkEventExpose *event) {

  gdk_draw_pixmap(widget->window,
		  widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
		  pixmap, event->area.x, event->area.y,
		  event->area.x, event->area.y,
		  event->area.width, event->area.height);

  return TRUE;
}

static gint button_press_event(GtkWidget *widget __attribute__ ((unused)),
			       GdkEventButton *event) {

  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching || loading_scans;
    g_mutex_unlock(laserscans_mutex);
    if (ignore)
      return TRUE;
  }

  if (scan_list && (event->button == 1)) {

    button1 = 1;
    button1_x = (event->x / (double) MIN(canvas_width, canvas_height)) *
      map_width;
    button1_y = (1 - (event->y / (double) MIN(canvas_width, canvas_height))) *
      map_height;

    if (tool == TOOL_ZOOM_IN)
      zoom_in(event->x, event->y);
    else if (tool == TOOL_ZOOM_OUT)
      zoom_out(event->x, event->y);
  }

  else if (event->button == 2)
    zoom_in(event->x, event->y);

  else if (event->button == 3)
    zoom_out(event->x, event->y);

  return TRUE;
}

static gint button_release_event(GtkWidget *widget __attribute__ ((unused)),
				 GdkEventButton *event) {

  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = loading_scans || scan_matching;
    g_mutex_unlock(laserscans_mutex);
  }

  if (!ignore) {
    if (scan_list && (event->button == 1)) {
      button1 = 0;
      if (tool != TOOL_NONE && tool != TOOL_ZOOM_IN && tool != TOOL_ZOOM_OUT)
	history_add();
    }
    laser_scans_display(NULL);
  }

  return TRUE;
}

static gint motion_notify_event(GtkWidget *widget __attribute__ ((unused)),
				GdkEventMotion *event) {

  int x, y;
  GdkModifierType state;

  if (button1) {

    if (event->is_hint)
      gdk_window_get_pointer(event->window, &x, &y, &state);
    else {
      x = event->x;
      y = event->y;
      state = event->state;
    }

    if (tool == TOOL_SHIFT) {
      shift_scans_by_motion(x, y);
      laser_scans_display(NULL);
    }

    else if (tool == TOOL_ROTATE) {
      rotate_scans_by_motion(x, y);
      laser_scans_display(NULL);
    }

    else if (tool == TOOL_STRETCH) {
      stretch_scans_by_motion(x, y);
      laser_scans_display(NULL);
    }

    else if (tool == TOOL_BEND) {
      bend_scans_by_motion(x, y);
      laser_scans_display(NULL);
    }
  }

  return TRUE;
}

static int key_control_l_down = 0;
static int key_control_r_down = 0;
static int key_alt_l_down = 0;
static int key_alt_r_down = 0;
static int key_1_down = 0;
static int key_2_down = 0;
static int key_3_down = 0;
static int key_4_down = 0;
static int key_5_down = 0;
static int key_6_down = 0;
static int key_7_down = 0;
static int key_8_down = 0;
static int key_9_down = 0;

static gint key_press_event(GtkWidget *widget __attribute__ ((unused)),
			    GdkEventKey *event) {

  if (event->keyval == GDK_Control_L)
    key_control_l_down = 1;
  else if (event->keyval == GDK_Control_R)
    key_control_r_down = 1;
  else if (event->keyval == GDK_Alt_L)
    key_alt_l_down = 1;
  else if (event->keyval == GDK_Alt_R)
    key_alt_r_down = 1;
  else if (event->keyval == GDK_1 && !key_1_down) {
    key_1_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(1);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(1);
  }
  else if (event->keyval == GDK_2 && !key_2_down) {
    key_2_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(2);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(2);
  }
  else if (event->keyval == GDK_3 && !key_3_down) {
    key_3_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(3);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(3);
  }
  else if (event->keyval == GDK_4 && !key_4_down) {
    key_4_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(4);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(4);
  }
  else if (event->keyval == GDK_5 && !key_5_down) {
    key_5_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(5);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(5);
  }
  else if (event->keyval == GDK_6 && !key_6_down) {
    key_6_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(6);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(6);
  }
  else if (event->keyval == GDK_7 && !key_7_down) {
    key_7_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(7);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(7);
  }
  else if (event->keyval == GDK_8 && !key_8_down) {
    key_8_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(8);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(8);
  }
  else if (event->keyval == GDK_9 && !key_9_down) {
    key_9_down = 1;
    if (key_control_l_down || key_control_r_down)
      scan_range_buf_store(9);
    else if (key_alt_l_down || key_alt_r_down)
      scan_range_buf_recall(9);
  }

  return TRUE;
}

static gint key_release_event(GtkWidget *widget __attribute__ ((unused)),
			      GdkEventKey *event) {

  if (event->keyval == GDK_Control_L)
    key_control_l_down = 0;
  else if (event->keyval == GDK_Control_R)
    key_control_r_down = 0;
  else if (event->keyval == GDK_Alt_L)
    key_alt_l_down = 0;
  else if (event->keyval == GDK_Alt_R)
    key_alt_r_down = 0;
  else if (event->keyval == GDK_1)
    key_1_down = 0;
  else if (event->keyval == GDK_2)
    key_2_down = 0;
  else if (event->keyval == GDK_3)
    key_3_down = 0;
  else if (event->keyval == GDK_4)
    key_4_down = 0;
  else if (event->keyval == GDK_5)
    key_5_down = 0;
  else if (event->keyval == GDK_6)
    key_6_down = 0;
  else if (event->keyval == GDK_7)
    key_7_down = 0;
  else if (event->keyval == GDK_8)
    key_8_down = 0;
  else if (event->keyval == GDK_9)
    key_9_down = 0;

  return TRUE;
}

static void help_window_destroy(GtkWidget *w __attribute__ ((unused)),
				gpointer p __attribute__ ((unused))) {
  help_window = NULL;
}

static char *help_file() {

  char *s;
  int n, pos;
  const int i = 1024;
  FILE *f;
  char *path;

  path = carmen_file_find(HELP_FILE_NAME);
  if (path == NULL)
    return NULL;
  f = fopen(path, "r");

  if (!f)
    return NULL;

  n = pos = 0;
  s = NULL;

  while (!feof(f)) {
    if (n - pos < i) {
      s = (char *) realloc(s, (n + i) * sizeof(char));
      carmen_test_alloc(s);
      n += i;
    }
    if (!fgets(s + pos, i, f)) {
      free(s);
      return NULL;
    }
    pos += strlen(s + pos);
  }

  return s;
}

static char *sp_error_msg() {

  int i, num_paths, n, size;
  char *errmes, **sp = carmen_get_search_path(&num_paths);

  errmes = (char *) calloc(1024, sizeof(char));
  size = 1024;
  carmen_test_alloc(errmes);

  sprintf(errmes, "Couldn't find file: %s.\nSearch path:\n", HELP_FILE_NAME);
  n = strlen(errmes) + 1;

  for (i = n = size = 0; i < num_paths; i++) {
    n += strlen(sp[i]) + 1;
    while (n >= size) {
      errmes = (char *) realloc(errmes, (size + 1024) * sizeof(char));
      carmen_test_alloc(errmes);
      size += 1024;
    }
    strcat(errmes, sp[i]);
    strcat(errmes, "\n");
    free(sp[i]);
  }
  free(sp);

  return errmes;
}

static int help_init() {

  GtkWidget *swin, *label;
  char *s;

  s = help_file();

  if (s == NULL)
    s = sp_error_msg();

  help_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT(help_window), "destroy",
		     GTK_SIGNAL_FUNC(help_window_destroy), NULL);

  swin = gtk_scrolled_window_new(NULL, NULL);
  gtk_widget_set_usize(swin, 550, 400);

  label = gtk_label_new(s);
  free(s);

  gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(swin), label);
  gtk_container_add(GTK_CONTAINER(help_window), swin);

  return 0;
}

static void help_local(GtkWidget *w __attribute__ ((unused)),
		       gpointer p __attribute__ ((unused))) {

  if (!help_window)
    if (help_init() < 0) {
      status_print("help file not found\n", "gui");
      return;
    }

  gtk_widget_show_all(help_window);
}

static void file_cancel(GtkWidget *w __attribute__ ((unused)),
			gpointer p __attribute__ ((unused))) {

  gtk_widget_hide(file_window);
}

static void file_window_destroy(GtkWidget *w __attribute__ ((unused)),
				gpointer p __attribute__ ((unused))) {

  file_window = NULL;
}

static void file_open_ok(GtkWidget *w __attribute__ ((unused)),
			 gpointer p __attribute__ ((unused))) {

  sprintf(logfilename, "%s", gtk_file_selection_get_filename(
			       GTK_FILE_SELECTION(file_window)));
  load_logfile(NULL);

  gtk_widget_hide(file_window);
}

static void file_save_as_ok(GtkWidget *w __attribute__ ((unused)),
			    gpointer p __attribute__ ((unused))) {

  sprintf(logfilename, "%s", gtk_file_selection_get_filename(
			       GTK_FILE_SELECTION(file_window)));
  save_logfile();
  gtk_widget_hide(file_window);
}

static void file_window_open_init() {

  if (!file_window) {
    file_window = gtk_file_selection_new("Open File");

    gtk_signal_connect(
	   GTK_OBJECT(GTK_FILE_SELECTION(file_window)->cancel_button),
	   "clicked", GTK_SIGNAL_FUNC(file_cancel), NULL);

    gtk_signal_connect(GTK_OBJECT(file_window), "destroy",
	  	       GTK_SIGNAL_FUNC(file_window_destroy), NULL);
  }
  else {
    gtk_window_set_title(GTK_WINDOW(file_window), "Open File");

    gtk_signal_disconnect_by_func(
           GTK_OBJECT(GTK_FILE_SELECTION(file_window)->ok_button),
           GTK_SIGNAL_FUNC(file_save_as_ok), NULL);
  }

  gtk_signal_connect(
         GTK_OBJECT(GTK_FILE_SELECTION(file_window)->ok_button),
         "clicked", GTK_SIGNAL_FUNC(file_open_ok), NULL);
}

static void file_open(GtkWidget *w __attribute__ ((unused)),
		      gpointer p __attribute__ ((unused))) {
  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore)
    return;

  file_window_open_init();

  gtk_widget_show(file_window);
}

static void file_window_save_as_init() {

  if (!file_window) {
    file_window = gtk_file_selection_new("Open File");

    gtk_signal_connect(
	   GTK_OBJECT(GTK_FILE_SELECTION(file_window)->cancel_button),
	   "clicked", GTK_SIGNAL_FUNC(file_cancel), NULL);

    gtk_signal_connect(GTK_OBJECT(file_window), "destroy",
	  	       GTK_SIGNAL_FUNC(file_window_destroy), NULL);
  }
  else {
    gtk_window_set_title(GTK_WINDOW(file_window), "Save File");

    gtk_signal_disconnect_by_func(
           GTK_OBJECT(GTK_FILE_SELECTION(file_window)->ok_button),
           GTK_SIGNAL_FUNC(file_open_ok), NULL);
  }

  gtk_signal_connect(
         GTK_OBJECT(GTK_FILE_SELECTION(file_window)->ok_button),
         "clicked", GTK_SIGNAL_FUNC(file_save_as_ok), NULL);
}

static void file_save(GtkWidget *w __attribute__ ((unused)),
		      gpointer p __attribute__ ((unused))) {
  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore)
    return;

  save_logfile();
}

static void file_save_as(GtkWidget *w __attribute__ ((unused)),
			 gpointer p __attribute__ ((unused))) {
  int ignore = 0;

  if (laserscans_mutex) {
    g_mutex_lock(laserscans_mutex);
    ignore = scan_matching;
    g_mutex_unlock(laserscans_mutex);
  }
  if (ignore)
    return;

  if (scan_list == NULL) {
    status_print("Error: Nothing to save.", "gui");
    return;
  }

  file_window_save_as_init();

  gtk_widget_show(file_window);
}

static GtkWidget *main_menubar_init(GtkWidget *main_window)
{
  GtkActionEntry action_entries[] = {
    {"FileMenu", NULL, "_File", NULL, NULL, NULL},
    {"Open", GTK_STOCK_OPEN, "_Open", "<control>O", NULL,
     G_CALLBACK(file_open)},
    {"Save", GTK_STOCK_SAVE, "_Save", "<control>S", NULL,
     G_CALLBACK(file_save)},
    {"SaveAs", GTK_STOCK_SAVE_AS, "Save _As", NULL, NULL,
     G_CALLBACK(file_save_as)},
    {"Quit", GTK_STOCK_QUIT, "_Quit", "<control>Q", NULL,
     G_CALLBACK(main_window_destroy)},

    {"EditMenu", NULL, "_Edit", NULL, NULL, NULL},
    {"Undo", GTK_STOCK_UNDO, "_Undo", "<control>Z", NULL,
     G_CALLBACK(history_undo)},
    {"Redo", GTK_STOCK_REDO, "_Redo", "<control>Y", NULL,
     G_CALLBACK(history_redo)},
    {"Delete", GTK_STOCK_DELETE, "_Delete", "<control>X", NULL,
     G_CALLBACK(laser_scans_delete)},

    {"HelpMenu", NULL, "_Help", NULL, NULL, NULL},
    {"LocalHelp", GTK_STOCK_HELP, "_Undo", "<control>H", NULL,
     G_CALLBACK(help_local)},
  };

  const char *ui_description =
    "<ui>"
    "  <menubar name='MainMenu'>"
    "    <menu action='FileMenu'>"
    "      <menuitem action='Open'/>"
    "      <menuitem action='Save'/>"
    "      <menuitem action='SaveAs'/>"
    "      <separator/>"
    "      <menuitem action='Quit'/>"
    "    </menu>"
    "    <menu action='EditMenu'>"
    "      <menuitem action='Undo'/>"
    "      <menuitem action='Redo'/>"
    "      <separator/>"
    "      <menuitem action='Delete'/>"
    "    </menu>"
    "    <menu action='HelpMenu'>"
    "      <menuitem action='LocalHelp'/>"
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
				G_N_ELEMENTS (action_entries), main_window);
  ui_manager = gtk_ui_manager_new ();
  gtk_ui_manager_insert_action_group (ui_manager, action_group, 0);

  accel_group = gtk_ui_manager_get_accel_group (ui_manager);
  gtk_window_add_accel_group (GTK_WINDOW (main_window), accel_group);

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

static GtkWidget *toolbar_init() {

  GtkWidget *hbox, *sep1, *sep2;
  GtkWidget *scan_match_button, *egrid_button;

  hbox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(hbox), 10);

  shift_button = gtk_toggle_button_new_with_label(" Shift ");
  rotate_button = gtk_toggle_button_new_with_label(" Rotate ");
  stretch_button = gtk_toggle_button_new_with_label(" Stretch ");
  bend_button = gtk_toggle_button_new_with_label(" Bend ");
  sep1 = gtk_vseparator_new();
  zoom_in_button = gtk_toggle_button_new_with_label(" Zoom In ");
  zoom_out_button = gtk_toggle_button_new_with_label(" Zoom Out ");
  sep2 = gtk_vseparator_new();
  scan_match_button = gtk_button_new_with_label(" Scan Match ");
  egrid_button = gtk_button_new_with_label(" Make Evidence Grid ");

  gtk_signal_connect(GTK_OBJECT(egrid_button), "clicked",
		     GTK_SIGNAL_FUNC(egrid_dialog_popup), NULL);

  gtk_signal_connect(GTK_OBJECT(shift_button), "toggled",
		     GTK_SIGNAL_FUNC(set_tool), (gpointer) TOOL_SHIFT);

  gtk_signal_connect(GTK_OBJECT(rotate_button), "toggled",
		     GTK_SIGNAL_FUNC(set_tool), (gpointer) TOOL_ROTATE);

  gtk_signal_connect(GTK_OBJECT(stretch_button), "toggled",
		     GTK_SIGNAL_FUNC(set_tool), (gpointer) TOOL_STRETCH);

  gtk_signal_connect(GTK_OBJECT(bend_button), "toggled",
		     GTK_SIGNAL_FUNC(set_tool), (gpointer) TOOL_BEND);

  gtk_signal_connect(GTK_OBJECT(zoom_in_button), "toggled",
		     GTK_SIGNAL_FUNC(set_tool), (gpointer) TOOL_ZOOM_IN);

  gtk_signal_connect(GTK_OBJECT(zoom_out_button), "toggled",
		     GTK_SIGNAL_FUNC(set_tool), (gpointer) TOOL_ZOOM_OUT);

  gtk_signal_connect(GTK_OBJECT(scan_match_button), "clicked",
		     GTK_SIGNAL_FUNC(click_scan_match_button), NULL);

  gtk_box_pack_start(GTK_BOX(hbox), shift_button, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), rotate_button, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), stretch_button, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), bend_button, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), sep1, FALSE, FALSE, 5);
  gtk_box_pack_start(GTK_BOX(hbox), zoom_in_button, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), zoom_out_button, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), sep2, FALSE, FALSE, 5);
  gtk_box_pack_start(GTK_BOX(hbox), scan_match_button, FALSE, FALSE, 2);
  gtk_box_pack_start(GTK_BOX(hbox), egrid_button, FALSE, FALSE, 2);

  return hbox;
}

static void main_window_init() {

  GtkWidget *main_window, *vbox;
  GtkWidget *hbox1, *hbox2, *hbox3, *hbox4, *hbox5;
  GtkWidget *table;

  main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_policy(GTK_WINDOW(main_window), FALSE, TRUE, FALSE);
  gtk_container_set_border_width(GTK_CONTAINER(main_window), 0);

  gtk_widget_add_events(main_window, GDK_KEY_PRESS_MASK | GDK_KEY_RELEASE_MASK);

  gtk_signal_connect(GTK_OBJECT(main_window), "destroy",
		     GTK_SIGNAL_FUNC(main_window_destroy), NULL);

  gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
		     GTK_SIGNAL_FUNC(key_press_event), NULL);

  gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
		     GTK_SIGNAL_FUNC(key_release_event), NULL);

  vbox = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox),
		     main_menubar_init(main_window), FALSE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), toolbar_init(), FALSE, FALSE, 0);

  canvas = gtk_drawing_area_new();

  gtk_drawing_area_size((GtkDrawingArea *) canvas,
			(gint) canvas_width,
			(gint) canvas_height);

  gtk_signal_connect(GTK_OBJECT(canvas), "expose_event",
		     GTK_SIGNAL_FUNC(main_window_expose), NULL);

  gtk_signal_connect(GTK_OBJECT(canvas), "configure_event",
		     GTK_SIGNAL_FUNC(main_window_configure), NULL);

  gtk_signal_connect(GTK_OBJECT(canvas), "button_press_event",
		     GTK_SIGNAL_FUNC(button_press_event), NULL);

  gtk_signal_connect(GTK_OBJECT(canvas), "button_release_event",
		     GTK_SIGNAL_FUNC(button_release_event), NULL);

  gtk_signal_connect(GTK_OBJECT(canvas), "motion_notify_event",
		     GTK_SIGNAL_FUNC(motion_notify_event), NULL);

  gtk_widget_set_events(canvas, GDK_EXPOSURE_MASK | GDK_BUTTON_PRESS_MASK |
			GDK_KEY_PRESS_MASK | GDK_BUTTON1_MOTION_MASK |
		 	GDK_BUTTON_RELEASE_MASK |
			GDK_POINTER_MOTION_HINT_MASK);

  scrolled_window = gtk_scrolled_window_new(NULL, NULL);
  gtk_container_set_border_width(GTK_CONTAINER(scrolled_window), 10);
  gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
				 GTK_POLICY_ALWAYS, GTK_POLICY_ALWAYS);
  gtk_widget_set_usize(scrolled_window, 425, 425);
  gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(scrolled_window),
					canvas);

  gtk_box_pack_start(GTK_BOX(vbox), scrolled_window, TRUE, TRUE, 0);

  scan_range_min_adj = gtk_adjustment_new(scan_range_min, 0, num_scans,
					  1, 1, 1);
  scan_range_mid_adj = gtk_adjustment_new(scan_range_mid, 0, num_scans,
					  1, 1, 1);
  scan_range_max_adj = gtk_adjustment_new(scan_range_max, 0, num_scans,
					  1, 1, 1);
  gtk_signal_connect(GTK_OBJECT(scan_range_min_adj), "value_changed",
		     GTK_SIGNAL_FUNC(set_scan_range_min), NULL);
  gtk_signal_connect(GTK_OBJECT(scan_range_mid_adj), "value_changed",
		     GTK_SIGNAL_FUNC(set_scan_range_mid), NULL);
  gtk_signal_connect(GTK_OBJECT(scan_range_max_adj), "value_changed",
		     GTK_SIGNAL_FUNC(set_scan_range_max), NULL);
  scan_range_min_scale = gtk_hscale_new(GTK_ADJUSTMENT(scan_range_min_adj));
  gtk_scale_set_digits(GTK_SCALE(scan_range_min_scale), 0);
  scan_range_mid_scale = gtk_hscale_new(GTK_ADJUSTMENT(scan_range_mid_adj));
  gtk_scale_set_digits(GTK_SCALE(scan_range_mid_scale), 0);
  scan_range_max_scale = gtk_hscale_new(GTK_ADJUSTMENT(scan_range_max_adj));
  gtk_scale_set_digits(GTK_SCALE(scan_range_max_scale), 0);
  scan_range_min_spin =
    gtk_spin_button_new(GTK_ADJUSTMENT(scan_range_min_adj), 1, 0);
  scan_range_mid_spin =
    gtk_spin_button_new(GTK_ADJUSTMENT(scan_range_mid_adj), 1, 0);
  scan_range_max_spin =
    gtk_spin_button_new(GTK_ADJUSTMENT(scan_range_max_adj), 1, 0);

  hbox1 = gtk_hbox_new(FALSE, 0);
  hbox2 = gtk_hbox_new(FALSE, 0);
  hbox3 = gtk_hbox_new(FALSE, 0);

  gtk_box_pack_start(GTK_BOX(hbox1), scan_range_min_spin, TRUE, TRUE, 20);
  gtk_box_pack_start(GTK_BOX(hbox1), scan_range_min_scale, TRUE, TRUE, 20);
  gtk_box_pack_start(GTK_BOX(hbox2), scan_range_mid_spin, TRUE, TRUE, 20);
  gtk_box_pack_start(GTK_BOX(hbox2), scan_range_mid_scale, TRUE, TRUE, 20);
  gtk_box_pack_start(GTK_BOX(hbox3), scan_range_max_spin, TRUE, TRUE, 20);
  gtk_box_pack_start(GTK_BOX(hbox3), scan_range_max_scale, TRUE, TRUE, 20);

  gtk_box_pack_start(GTK_BOX(vbox), hbox1, FALSE, TRUE, 2);
  gtk_box_pack_start(GTK_BOX(vbox), hbox2, FALSE, TRUE, 10);
  gtk_box_pack_start(GTK_BOX(vbox), hbox3, FALSE, TRUE, 10);

  table = gtk_table_new(1, 1, FALSE);

  progress_bar = gtk_progress_bar_new();
  status_bar = gtk_statusbar_new();

  gtk_table_attach(GTK_TABLE(table), progress_bar, 0, 1, 0, 1,
		   GTK_EXPAND, GTK_EXPAND, 0, 0);

  hbox4 = gtk_hbox_new(FALSE, 3);
  hbox5 = gtk_hbox_new(FALSE, 0);

  gtk_box_pack_start(GTK_BOX(hbox4), table, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox4), status_bar, TRUE, TRUE, 0);

  gtk_box_pack_start(GTK_BOX(hbox5), hbox4, TRUE, TRUE, 3);

  gtk_box_pack_start(GTK_BOX(vbox), hbox5, FALSE, TRUE, 5);

  gtk_container_add(GTK_CONTAINER(main_window), vbox);

  gtk_widget_show_all(main_window);
}

void graphics_init() {

  main_window_init();
  global_graphics_init();
}
