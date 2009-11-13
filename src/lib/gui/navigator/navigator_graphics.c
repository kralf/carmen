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

#include <X11/Xlib.h>
#include <X11/cursorfont.h>
#include <ctype.h>

#include "param_interface.h"
#include "simulator_interface.h"

#include "navigator_graphics.h"

#include "map_graphics.h"

Display *gdk_x11_cursor_get_xdisplay (GdkCursor *cursor);
Cursor gdk_x11_cursor_get_xcursor (GdkCursor *cursor);

#define BUTTON_WIDTH 275
#define BUTTON_HEIGHT 30
#define GRADIENT_COLORS 40

#define ALWAYS_REDRAW 1

#define DEFAULT_ROBOT_COLOUR carmen_red
#define DEFAULT_GOAL_COLOUR carmen_yellow
#define DEFAULT_PATH_COLOUR carmen_blue

#define DEFAULT_PEOPLE_COLOUR carmen_orange

#define DEFAULT_TRACK_ROBOT 1
#define DEFAULT_DRAW_WAYPOINTS 1
#define DEFAULT_SHOW_PARTICLES 0
#define DEFAULT_SHOW_GAUSSIANS 0
#define DEFAULT_SHOW_LASER 0
#define DEFAULT_SHOW_SIMULATOR 0

#define DEFAULT_SHOW_TRACKED_OBJECTS 1

typedef void *(*void_func)(void *);

typedef enum {NO_PLACEMENT, PLACING_ROBOT, ORIENTING_ROBOT,
	      PLACING_GOAL, PLACING_PERSON, ORIENTING_PERSON,
	      PLACING_SIMULATOR, ORIENTING_SIMULATOR} placement_t;

static int is_graphics_up = 0;

static carmen_navigator_map_t display;
static carmen_map_placelist_p placelist = NULL;
static carmen_list_t *place_action_uids = NULL;
static carmen_list_t *goal_actions = NULL;
static carmen_list_t *start_actions = NULL;

static double time_of_last_redraw = 0;
static int display_needs_updating = 0;
static int num_path_points;
static carmen_world_point_t *path = NULL;
static carmen_world_point_t goal;
static carmen_world_point_t robot;
static carmen_traj_point_t  robot_traj;
static carmen_world_point_t last_robot;
static carmen_world_point_t new_person;
static carmen_world_point_t new_simulator;

static GdkColor robot_colour, goal_colour, people_colour, path_colour;

static int black_and_white = 0;
static int is_filming = 0;
static guint filming_timeout = 0;

static placement_t placement_status = 0;
static carmen_world_point_t cursor_pos;
static double last_navigator_update = 0;
static double last_simulator_update = 0;

static carmen_localize_globalpos_message *globalpos;
static carmen_localize_particle_message particle_msg;
static carmen_localize_sensor_message sensor_msg;
GdkColor RedBlueGradient[GRADIENT_COLORS];

static int ignore_click;

static GtkUIManager *ui_manager;
static GtkActionGroup *goal_action_group;
static GtkActionGroup *start_action_group;

static GtkMapViewer *map_view;

static GtkWidget *window;
static GtkWidget *autonomous_button;
static GtkWidget *place_robot_button;
static GtkWidget *place_goal_button;
static GtkWidget *robot_status_label;
static GtkWidget *robot_speed_label;
static GtkWidget *goal_status_label;
static GtkWidget *cursor_status_label;
static GtkWidget *value_label;
static GtkWidget *map_status_label;
static GtkWidget *simulator_box;
static GtkWidget *filler_box;
static GtkWidget *place_simulator_button;
static GtkWidget *sync_mode_button;
static GtkWidget *next_tick_button;

static carmen_world_point_t simulator_trueposition = {{0, 0, 0}, NULL};
static double time_of_simulator_update = 0;
static double simulator_hidden;

static carmen_list_t *simulator_objects = NULL;
static carmen_list_t *people = NULL;

static carmen_robot_config_t *robot_config;
static carmen_navigator_config_t *nav_config;
static carmen_navigator_panel_config_t *nav_panel_config;

static void switch_display(GtkAction *action, gpointer user_data
			   __attribute__ ((unused)));
static void switch_localize_display(GtkAction *action,
				    gpointer user_data
				    __attribute__ ((unused)));
static void set_location(GtkAction *action, gpointer user_data
			 __attribute__ ((unused)));
static void start_filming(GtkWidget *w __attribute__ ((unused)),
			  int arg __attribute__ ((unused)));
static gint save_image(gpointer data, guint action, GtkWidget *widget);
static int colour_equals(GdkColor colour1, GdkColor colour2);

static void sync_mode_change_handler(char *module, char *variable,
				     char *value);

static void
delete_event(GtkWidget *widget, GdkEvent *event, gpointer data)
{
  widget = widget; event = event; data = data;

  gtk_main_quit ();
}

static void
label_autonomy_button(char *str)
{
  GtkWidget *label;

  label = GTK_BIN(autonomous_button)->child;
  gtk_label_set_text(GTK_LABEL(label), str);
}

static void
go_autonomous(GtkWidget *widget __attribute__ ((unused)),
	      gpointer data __attribute__ ((unused)))
{
  GtkWidget *label;

  if (!ignore_click)
    {
      if (GTK_TOGGLE_BUTTON (autonomous_button)->active)
	{
	  label = GTK_BIN(autonomous_button)->child;
	  gtk_label_set_text(GTK_LABEL(label), "Stop");
	  navigator_start_moving();
	}
      else
	{
	  label = GTK_BIN(autonomous_button)->child;
	  gtk_label_set_text(GTK_LABEL(label), "Go");
	  navigator_stop_moving();
	}
    }
  else
    ignore_click = 0;
}

static void next_tick(GtkWidget *widget __attribute__ ((unused)),
		      gpointer data __attribute__ ((unused)))
{
  if (GTK_TOGGLE_BUTTON(sync_mode_button)->active)
    carmen_simulator_next_tick();
}

static void sync_mode(GtkWidget *widget __attribute__ ((unused)),
		      gpointer data __attribute__ ((unused)))
{
  carmen_param_set_module(NULL);
  carmen_param_set_onoff
    ("simulator_sync_mode", GTK_TOGGLE_BUTTON(sync_mode_button)->active,
     NULL);
}

static void change_cursor(GdkColor *fg, GdkColor *bg)
{
  XColor xfg, xbg;
  Display *xdisplay;
  Cursor xcursor;
  GdkCursor *cursor = gdk_cursor_new(GDK_DOT);

  xfg.pixel = fg->pixel;
  xfg.red = fg->red;
  xfg.blue = fg->blue;
  xfg.green = fg->green;

  xbg.pixel = bg->pixel;
  xbg.red = bg->red;
  xbg.blue = bg->blue;
  xbg.green = bg->green;

  xdisplay = gdk_x11_cursor_get_xdisplay (cursor);
  xcursor = gdk_x11_cursor_get_xcursor (cursor);
  XRecolorCursor(xdisplay, xcursor, &xfg, &xbg);

  gdk_window_set_cursor (map_view->image_widget->window, cursor);
}

static void
place_robot(GtkWidget *widget __attribute__ ((unused)),
	    gpointer data __attribute__ ((unused)))
{
  change_cursor(&carmen_red, &carmen_black);
  placement_status = PLACING_ROBOT;
}

static void
place_simulator(GtkWidget *widget __attribute__ ((unused)),
		gpointer data __attribute__ ((unused)))
{
  change_cursor(&carmen_red, &carmen_black);
  placement_status = PLACING_SIMULATOR;
}

static void
place_person(GtkWidget *w __attribute__ ((unused)),
	     int arg __attribute__ ((unused)))
{
  change_cursor(&carmen_orange, &carmen_black);
  placement_status = PLACING_PERSON;
}

static void
clear_objects(GtkWidget *w __attribute__ ((unused)),
	      int arg __attribute__ ((unused)))
{
  carmen_simulator_clear_objects();
}

static void
place_goal(GtkWidget *widget __attribute__ ((unused)),
	    gpointer data __attribute__ ((unused)))
{
  change_cursor(&carmen_yellow, &carmen_black);
  placement_status = PLACING_GOAL;
}

static GtkActionEntry action_entries[] = {
  {"FileMenu", NULL, "_File", NULL, NULL, NULL},
  {"ScreenShot", GTK_STOCK_SAVE, "_Screen Shot", "<control>S", NULL,
   G_CALLBACK(save_image)},
  {"StartFilming", NULL, "Start Filming", NULL, NULL,
   G_CALLBACK(start_filming)},
  {"StopFilming", NULL, "Stop Filming", NULL, NULL, G_CALLBACK(start_filming)},
  {"Quit", GTK_STOCK_QUIT, "_Quit", "<control>Q", NULL,
   G_CALLBACK(gtk_main_quit)},
  {"MapMenu", NULL, "_Maps", NULL, NULL, NULL},
  {"DisplayMenu", NULL, "_Display", NULL, NULL, NULL},
  {"SimulatorMenu", NULL, "_Simulator", NULL, NULL, NULL},
  {"SimAddPerson", NULL, "Add Person", NULL, NULL,
   G_CALLBACK(place_person)},
  {"SimClearObjects", NULL, "Clear Objects", NULL, NULL,
   G_CALLBACK(clear_objects)},
  {"StartLocationMenu", NULL, "_Start Location", NULL, NULL, NULL},
  {"GlobalLocalization", NULL, "Global Localization", NULL, NULL,
   G_CALLBACK(set_location)},
  {"GoalMenu", NULL, "_Goals", NULL, NULL, NULL},
  {"HelpMenu", NULL, "_Help", NULL, NULL, NULL},
  {"HelpAbout", NULL, "_About", NULL, NULL, NULL}
};

static GtkToggleActionEntry toggle_entries[] = {
  {"TrackRobot", NULL, "Track Robot", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE},
  {"DrawWaypoints", NULL, "Draw Waypoints", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE},
  {"ShowParticles", NULL, "Show Particles", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE},
  {"ShowGaussians", NULL, "Show Gaussians", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE},
  {"ShowLaserData", NULL, "Show Laser Data", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE},
  {"BlackWhite", NULL, "Black&White", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE},
  {"SimShowTruePosition", NULL, "Show True Position", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE},
  {"SimShowObjects", NULL, "Show Objects", NULL, NULL,
   G_CALLBACK(switch_localize_display), FALSE}
};

static GtkRadioActionEntry radio_entries[] = {
  {"Map", NULL, "_Map", "<control>M", NULL, CARMEN_NAVIGATOR_MAP_v},
  {"Utility", NULL, "_Utility", NULL, NULL, CARMEN_NAVIGATOR_UTILITY_v},
  {"Costs", NULL, "_Costs", NULL, NULL, CARMEN_NAVIGATOR_COST_v},
  {"Likelihood", NULL, "_Likelihood", NULL, NULL, CARMEN_LOCALIZE_LMAP_v},
  {"GLikelihood", NULL, "_Global Likelihood", NULL, NULL,
   CARMEN_LOCALIZE_GMAP_v}
};

const char *ui_description =
    "<ui>"
    "  <menubar name='MainMenu'>"
    "    <menu action='FileMenu'>"
    "      <menuitem action='ScreenShot'/>"
    "      <separator/>"
    "      <menuitem action='StartFilming'/>"
    "      <menuitem action='StopFilming'/>"
    "      <separator/>"
    "      <menuitem action='Quit'/>"
    "    </menu>"
    "    <menu action='MapMenu'>"
    "      <menuitem action='Map'/>"
    "      <menuitem action='Utility'/>"
    "      <menuitem action='Costs'/>"
    "      <menuitem action='Likelihood'/>"
    "      <menuitem action='GLikelihood'/>"
    "    </menu>"
    "    <menu action='DisplayMenu'>"
    "      <menuitem action='TrackRobot'/>"
    "      <menuitem action='DrawWaypoints'/>"
    "      <menuitem action='ShowParticles'/>"
    "      <menuitem action='ShowGaussians'/>"
    "      <menuitem action='ShowLaserData'/>"
    "      <separator/>"
    "      <menuitem action='BlackWhite'/>"
    "    </menu>"
    "    <menu action='SimulatorMenu'>"
    "      <menuitem action='SimShowTruePosition'/>"
    "      <menuitem action='SimShowObjects'/>"
    "      <separator/>"
    "      <menuitem action='SimAddPerson'/>"
    "      <menuitem action='SimClearObjects'/>"
    "    </menu>"
    "    <menu action='StartLocationMenu'>"
    "      <menuitem action='GlobalLocalization'/>"
    "    </menu>"
    "    <menu action='GoalMenu'>"
    "    </menu>"
    "    <menu action='HelpMenu'>"
    "      <menuitem action='HelpAbout'/>"
    "    </menu>"
    "  </menubar>"
    "</ui>";

static void
switch_localize_display(GtkAction *action,
			gpointer user_data __attribute__ ((unused)))

{
  char *name;
  GtkToggleAction *toggle;

  name = (char *)gtk_action_get_name(action);

  if (strcmp(name, "TrackRobot") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    nav_panel_config->track_robot = gtk_toggle_action_get_active(toggle);
    if (robot.map && nav_panel_config->track_robot)
      carmen_map_graphics_adjust_scrollbars(map_view, &robot);
  } else if (strcmp(name, "DrawWaypoints") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    nav_panel_config->draw_waypoints = gtk_toggle_action_get_active(toggle);
  } else if (strcmp(name, "ShowParticles") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    nav_panel_config->show_particles = gtk_toggle_action_get_active(toggle);
    if (nav_panel_config->show_particles == 1 &&
	!nav_panel_config->show_gaussians)
      carmen_localize_subscribe_particle_message
	(&particle_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
    else if (!nav_panel_config->show_particles &&
	     !nav_panel_config->show_gaussians)
      carmen_localize_subscribe_particle_message(NULL, NULL,
						 CARMEN_UNSUBSCRIBE);
  } else if (strcmp(name, "ShowGaussians") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    nav_panel_config->show_gaussians = gtk_toggle_action_get_active(toggle);
  } else if (strcmp(name, "ShowLaserData") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    nav_panel_config->show_lasers = gtk_toggle_action_get_active(toggle);
    if (nav_panel_config->show_lasers)
      carmen_localize_subscribe_sensor_message
	(&sensor_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
    else
      carmen_localize_subscribe_sensor_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
  } else if (strcmp(name, "SimShowTruePosition") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    nav_panel_config->show_true_pos = gtk_toggle_action_get_active(toggle);
  } else if (strcmp(name, "SimShowObjects") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    nav_panel_config->show_simulator_objects =
      gtk_toggle_action_get_active(toggle);
  } else if (strcmp(name, "BlackWhite") == 0) {
    toggle = GTK_TOGGLE_ACTION(action);
    black_and_white = gtk_toggle_action_get_active(toggle);
    if (black_and_white) {
      if (colour_equals(robot_colour, DEFAULT_ROBOT_COLOUR))
	robot_colour = carmen_grey;
      if (colour_equals(goal_colour, DEFAULT_GOAL_COLOUR))
	goal_colour = carmen_grey;
      if (colour_equals(path_colour, DEFAULT_PATH_COLOUR))
	path_colour = carmen_black;
    } else {
      if (colour_equals(robot_colour, carmen_grey))
	robot_colour = DEFAULT_ROBOT_COLOUR;
      if (colour_equals(goal_colour, carmen_grey))
	goal_colour = DEFAULT_GOAL_COLOUR;
      if (colour_equals(path_colour, carmen_black))
	path_colour = DEFAULT_PATH_COLOUR;
    }
  }
}

static gint
film_image(gpointer data)
{
  return save_image(data, 0, NULL);
}

static void start_filming(GtkWidget *w __attribute__ ((unused)),
			  int arg __attribute__ ((unused)))
{
  GtkWidget *menu_item;

  if (is_filming) {
    menu_item =
      gtk_ui_manager_get_widget(ui_manager,
				"/ui/MainMenu/FileMenu/StopFilming");
    gtk_widget_hide(menu_item);
    menu_item =
      gtk_ui_manager_get_widget(ui_manager,
				"/ui/MainMenu/FileMenu/StartFilming");
    gtk_widget_show(menu_item);
    gtk_timeout_remove(filming_timeout);
    is_filming = 0;
  } else {
    menu_item =
      gtk_ui_manager_get_widget(ui_manager,
				"/ui/MainMenu/FileMenu/StartFilming");
    gtk_widget_hide(menu_item);
    menu_item =
      gtk_ui_manager_get_widget(ui_manager,
				"/ui/MainMenu/FileMenu/StopFilming");
    gtk_widget_show(menu_item);
    is_filming = 1;
    filming_timeout = gtk_timeout_add
      (1000, (GtkFunction)film_image, NULL);
  }
}

static int
colour_equals(GdkColor colour1, GdkColor colour2)
{
  if (colour1.red == colour2.red &&
      colour1.blue == colour2.blue &&
      colour1.green == colour2.green)
    return 1;
  else
    return 0;
}


static void
assign_colour(GdkColor *colour, int new_colour)
{
  colour->pixel = new_colour;
  colour->blue = new_colour & 0xff;
  new_colour >>= 8;
  colour->green = new_colour & 0xff;
  new_colour >>= 8;
  colour->red = new_colour & 0xff;
  new_colour >>= 8;
}

static void
assign_variable(char *action_name, int value, int default_value)
{
  GtkAction *action;
  int state;

  if (value > 2)
    return;

  action = gtk_ui_manager_get_action(ui_manager, action_name);
  state = gtk_toggle_action_get_active(GTK_TOGGLE_ACTION(action));
  if (value == -1)
    value = default_value;
  if (state != value)
     gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(action), value);
}

void
navigator_graphics_reset(void)
{
  robot_colour = DEFAULT_ROBOT_COLOUR;
  goal_colour = DEFAULT_GOAL_COLOUR;
  path_colour = DEFAULT_PATH_COLOUR;
  people_colour = DEFAULT_PEOPLE_COLOUR;

  assign_variable("/ui/MainMenu/DisplayMenu/TrackRobot", -1,
		  DEFAULT_TRACK_ROBOT);
  assign_variable("/ui/MainMenu/DisplayMenu/DrawWaypoints", -1,
		  DEFAULT_DRAW_WAYPOINTS);
  assign_variable("/ui/MainMenu/DisplayMenu/ShowParticles", -1,
		  DEFAULT_SHOW_PARTICLES);
  assign_variable("/ui/MainMenu/DisplayMenu/ShowGaussians", -1,
		  DEFAULT_SHOW_GAUSSIANS);
  assign_variable("/ui/MainMenu/DisplayMenu/ShowLaserData", -1,
		  DEFAULT_SHOW_LASER);
  assign_variable("/ui/MainMenu/SimulatorMenu/SimShowTruePosition", -1,
		  DEFAULT_SHOW_SIMULATOR);
  assign_variable("/ui/MainMenu/SimulatorMenu/SimShowObjects", -1,
		  DEFAULT_SHOW_TRACKED_OBJECTS);
}

void
navigator_graphics_display_config
(char *attribute, int value, char *new_status_message __attribute__ ((unused)))
{
  if (strncmp(attribute, "robot colour", 12) == 0) {
    if (value == -1)
      robot_colour = DEFAULT_ROBOT_COLOUR;
    else
      assign_colour(&robot_colour, value);
  } else if (strncmp(attribute, "goal colour", 11) == 0) {
    if (value == -1)
      goal_colour = DEFAULT_GOAL_COLOUR;
    else
      assign_colour(&goal_colour, value);
  } else if (strncmp(attribute, "path colour", 11) == 0) {
    if (value == -1)
      path_colour = DEFAULT_PATH_COLOUR;
    else
      assign_colour(&path_colour, value);
  } else if (strncmp(attribute, "people colour", 11) == 0) {
    if (value == -1)
      path_colour = DEFAULT_PATH_COLOUR;
    else
      assign_colour(&people_colour, value);
  } else if (strncmp(attribute, "track robot", 11) == 0)
    assign_variable("/ui/MainMenu/DisplayMenu/TrackRobot",
		    value, DEFAULT_TRACK_ROBOT);
  else if (strncmp(attribute, "draw waypoints", 14) == 0)
    assign_variable("/ui/MainMenu/DisplayMenu/DrawWaypoints",
		    value, DEFAULT_DRAW_WAYPOINTS);
  else if (strncmp(attribute, "show particles", 14) == 0)
    assign_variable("/ui/MainMenu/DisplayMenu/ShowParticles",
		    value, DEFAULT_SHOW_PARTICLES);
  else if (strncmp(attribute, "show gaussians", 14) == 0)
    assign_variable("/ui/MainMenu/DisplayMenu/ShowGaussians",
		    value, DEFAULT_SHOW_GAUSSIANS);
  else if (strncmp(attribute, "show laser", 10) == 0)
    assign_variable("/ui/MainMenu/DisplayMenu/ShowLaserData",
		    value, DEFAULT_SHOW_LASER);
  else if (strncmp(attribute, "show simulator", 14) == 0)
    assign_variable("/ui/MainMenu/SimulatorMenu/SimShowTruePosition",
		    value, DEFAULT_SHOW_SIMULATOR);
  else if (strncmp(attribute, "show tracked objects", 20) == 0)
    assign_variable("/ui/MainMenu/SimulatorMenu/SimShowObjects",
		    value, DEFAULT_SHOW_TRACKED_OBJECTS);
  carmen_map_graphics_redraw(map_view);
}

static void switch_display(GtkAction *action, gpointer user_data
			   __attribute__ ((unused)))
{
  carmen_navigator_map_t new_display;

  new_display = gtk_radio_action_get_current_value(GTK_RADIO_ACTION(action));

  if (display == new_display)
    return;

  navigator_display_map(new_display);
}

static GtkWidget *get_main_menu(void)
{
  GtkWidget *menubar;
  GtkActionGroup *action_group;
  GtkAccelGroup *accel_group;
  GError *error;
  GtkAction* action;

  action_group = gtk_action_group_new ("MenuActions");
  gtk_action_group_add_actions (action_group, action_entries,
				G_N_ELEMENTS (action_entries), window);
  gtk_action_group_add_toggle_actions (action_group, toggle_entries,
				       G_N_ELEMENTS (toggle_entries), window);

  gtk_action_group_add_radio_actions (action_group, radio_entries,
				      G_N_ELEMENTS (radio_entries),
				      CARMEN_NAVIGATOR_MAP_v,
				      G_CALLBACK(switch_display), NULL);

  ui_manager = gtk_ui_manager_new ();
  gtk_ui_manager_insert_action_group (ui_manager, action_group, 0);

  accel_group = gtk_ui_manager_get_accel_group (ui_manager);
  gtk_window_add_accel_group (GTK_WINDOW (window), accel_group);

  error = NULL;
  if (!gtk_ui_manager_add_ui_from_string (ui_manager, ui_description, -1,
					  &error)) {
    g_message ("building menus failed: %s", error->message);
    g_error_free (error);
    exit (EXIT_FAILURE);
  }

  menubar = gtk_ui_manager_get_widget (ui_manager, "/MainMenu");

  action = gtk_action_group_get_action(action_group, "TrackRobot");
  gtk_toggle_action_set_active
    (GTK_TOGGLE_ACTION(action), nav_panel_config->track_robot);

  action = gtk_action_group_get_action(action_group, "DrawWaypoints");
  gtk_toggle_action_set_active
    (GTK_TOGGLE_ACTION(action), nav_panel_config->draw_waypoints);

  action = gtk_action_group_get_action(action_group, "ShowParticles");
  gtk_toggle_action_set_active
    (GTK_TOGGLE_ACTION(action), nav_panel_config->show_particles);

  action = gtk_action_group_get_action(action_group, "ShowGaussians");
  gtk_toggle_action_set_active
    (GTK_TOGGLE_ACTION(action), nav_panel_config->show_gaussians);

  action = gtk_action_group_get_action(action_group, "ShowLaserData");
  gtk_toggle_action_set_active
    (GTK_TOGGLE_ACTION(action), nav_panel_config->show_lasers);

  action = gtk_action_group_get_action(action_group, "SimShowTruePosition");
  gtk_toggle_action_set_active
    (GTK_TOGGLE_ACTION(action), nav_panel_config->show_true_pos);

  action = gtk_action_group_get_action(action_group, "SimShowObjects");
  gtk_toggle_action_set_active
    (GTK_TOGGLE_ACTION(action), nav_panel_config->show_simulator_objects);

  if (nav_panel_config->show_particles || nav_panel_config->show_gaussians)
    carmen_localize_subscribe_particle_message
      (&particle_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
  if (nav_panel_config->show_lasers)
    carmen_localize_subscribe_sensor_message(&sensor_msg, NULL,
					     CARMEN_SUBSCRIBE_LATEST);

  return menubar;
}

static GtkWidget *new_label(char *s, GtkWidget *box)
{
  GtkWidget *the_new_label;

  the_new_label = gtk_label_new(s);
  gtk_box_pack_start(GTK_BOX(box), the_new_label, TRUE, TRUE, 0);

  return the_new_label;
}

static GtkWidget *construct_status_frame(GtkWidget *parent)
{
  GtkWidget *status_frame;
  GtkWidget *status_box;

  status_frame = gtk_frame_new("Status");
  gtk_container_set_border_width (GTK_CONTAINER (status_frame), 10);
  gtk_container_add(GTK_CONTAINER(parent), status_frame);

  status_box = gtk_vbox_new(FALSE, 0);
  gtk_container_set_border_width (GTK_CONTAINER (status_box), 10);
  gtk_container_add(GTK_CONTAINER(status_frame), status_box);

  map_status_label = new_label("No map", status_box);

  return status_box;
}

static void do_redraw(void)
{
  if (display_needs_updating &&
      (carmen_get_time() - time_of_last_redraw > 0.3 || ALWAYS_REDRAW))
    {
      carmen_map_graphics_redraw(map_view);
      time_of_last_redraw = carmen_get_time();
      display_needs_updating = 0;
    }
}

static int received_robot_pose(void)
{
  return (robot.pose.x > 0 && robot.pose.y > 0 && robot.map != NULL);
}

static void draw_particles(GtkMapViewer *the_map_view, double pixel_size)
{
  int index;
  carmen_map_point_t map_particle;
  carmen_world_point_t particle;

  if (!nav_panel_config->show_particles || particle_msg.particles == NULL)
    return;

  map_particle.map = the_map_view->internal_map;

  for(index = 0; index < particle_msg.num_particles; index++) {
    particle.pose.x = particle_msg.particles[index].x;
    particle.pose.y = particle_msg.particles[index].y;
    particle.map = the_map_view->internal_map;
    carmen_map_graphics_draw_circle(the_map_view, &robot_colour, TRUE,
				    &particle, pixel_size);
  }

}

static void draw_gaussians(GtkMapViewer *the_map_view)
{
  carmen_world_point_t mean;

  if (!nav_panel_config->show_gaussians || particle_msg.particles == NULL)
    return;

  mean = robot;
  mean.pose.x = globalpos->globalpos.x;
  mean.pose.y = globalpos->globalpos.y;
  mean.pose.theta = globalpos->globalpos.theta;

  carmen_map_graphics_draw_ellipse
    (the_map_view, &carmen_black, &mean,
     carmen_square(globalpos->globalpos_std.x),
     globalpos->globalpos_xy_cov,
     carmen_square(globalpos->globalpos_std.y), 4);
}

static void draw_lasers(GtkMapViewer *the_map_view, double pixel_size)
{
  double dot_size;
  int index;
  carmen_world_point_t particle;
  double angle;

  dot_size = 3*pixel_size;

  if (!nav_panel_config->show_lasers)
    return;

  particle = robot;
  for(index = 0; index < sensor_msg.num_readings;
      index += sensor_msg.laser_skip) {

    // finale: doesn't this assume a 180 fov?
    // angle = sensor_msg.pose.theta - M_PI_2 +
    //  index / (float)(sensor_msg.num_readings - 1) * M_PI;
    angle = sensor_msg.pose.theta - sensor_msg.config.fov / 2 +
      index / (float)(sensor_msg.num_readings - 1) * sensor_msg.config.fov;


    particle.pose.x = sensor_msg.pose.x + sensor_msg.range[index] *
      cos(angle);
    particle.pose.y = sensor_msg.pose.y + sensor_msg.range[index] *
      sin(angle);
    if(sensor_msg.mask[index])
      carmen_map_graphics_draw_circle(the_map_view,
				      &carmen_green, TRUE,
				      &particle, dot_size);
    else
      carmen_map_graphics_draw_circle(the_map_view,
				      &carmen_yellow, TRUE,
				      &particle, dot_size);
  }

#ifdef blah
  /* rear laser */
  for(index = 0; index < sensor_msg.rear_laser_scan.num_readings;
      index++) {
    colour = sensor_msg.rear_laser_scan.scan[index].mean_prob *
      (GRADIENT_COLORS - 1);

    particle.pose.x = sensor_msg.rear_laser_scan.scan[index].mean_x;
    particle.pose.y = sensor_msg.rear_laser_scan.scan[index].mean_y;
    carmen_map_graphics_draw_circle(the_map_view,
				    &RedBlueGradient[colour], TRUE,
				    &particle, dot_size);
  }
#endif
}

static double x_coord(double x, double y, carmen_world_point_t *offset)
{
  return x*cos(offset->pose.theta)-y*sin(offset->pose.theta)+offset->pose.x;
}

static double y_coord(double x, double y, carmen_world_point_t *offset)
{
  return x*sin(offset->pose.theta)+y*cos(offset->pose.theta)+offset->pose.y;
}

static void draw_robot_shape(GtkMapViewer *the_map_view,
			     carmen_world_point_t *location, int filled,
			     GdkColor *colour, double pixel_size)
{
  double robot_radius;
  carmen_world_point_t wp[5];
  double width2, length2;

  if (!robot_config->rectangular) {
    robot_radius = robot_config->width/2.0;
    if (robot_radius/pixel_size < 5)
      robot_radius = pixel_size*5;

    carmen_map_graphics_draw_circle(the_map_view, colour, filled,
				    location, robot_radius);
    return;
  }

  width2 = robot_config->width/2;
  length2 = robot_config->length/2;

  if (width2/pixel_size < 5)
    width2 = pixel_size*5;
  if (length2/pixel_size < 5)
    length2 = pixel_size*5;

  wp[0].pose.x = x_coord(length2, width2, location);
  wp[0].pose.y = y_coord(length2, width2, location);
  wp[1].pose.x = x_coord(length2, -width2, location);
  wp[1].pose.y = y_coord(length2, -width2, location);
  wp[2].pose.x = x_coord(-length2, -width2, location);
  wp[2].pose.y = y_coord(-length2, -width2, location);
  wp[3].pose.x = x_coord(-length2, width2, location);
  wp[3].pose.y = y_coord(-length2, width2, location);
  wp[4].pose.x = wp[0].pose.x;
  wp[4].pose.y = wp[0].pose.y;

  wp[0].map = wp[1].map = wp[2].map = wp[3].map = wp[4].map = location->map;

  carmen_map_graphics_draw_polygon(the_map_view, colour, wp, 5, filled);
}

static void draw_simulated_robot(GtkMapViewer *the_map_view, double pixel_size)
{
  double robot_size;
  carmen_world_point_t radius;

  if (!nav_panel_config->show_true_pos || simulator_trueposition.map == NULL)
    return;

  robot_size = robot_config->width/2.0;
  if (robot_size/pixel_size < 5)
    robot_size = pixel_size*5;

  draw_robot_shape(the_map_view, &simulator_trueposition, TRUE, &carmen_blue,
		   pixel_size);
  draw_robot_shape(the_map_view, &simulator_trueposition, FALSE, &carmen_black,
		   pixel_size);

  radius = simulator_trueposition;
  radius.pose.x = radius.pose.x +
    cos(radius.pose.theta)*0.2;
  radius.pose.y = radius.pose.y +
    sin(radius.pose.theta)*0.2;

  carmen_map_graphics_draw_line(the_map_view, &carmen_black,
				&simulator_trueposition, &radius);
}


static void draw_robot(GtkMapViewer *the_map_view, double pixel_size)
{
  carmen_world_point_t robot_radius;


  if (!nav_panel_config->show_particles && !nav_panel_config->show_gaussians)
    draw_robot_shape(the_map_view, &robot, TRUE, &robot_colour, pixel_size);

  if (!nav_panel_config->show_gaussians)
    draw_robot_shape(the_map_view, &robot, FALSE, &carmen_black, pixel_size);

  robot_radius = robot;
  robot_radius.pose.x = robot_radius.pose.x +
    cos(robot_radius.pose.theta)*0.2;
  robot_radius.pose.y = robot_radius.pose.y +
    sin(robot_radius.pose.theta)*0.2;

  carmen_map_graphics_draw_line(the_map_view, &carmen_black, &robot,
				&robot_radius);



}


void draw_robot_objects(GtkMapViewer *the_map_view)
{
  int index;
  //  int colour;
  carmen_world_point_t path_x_1, path_x_2;
  carmen_world_point_t particle;
  double goal_size, pixel_size, robot_size, circle_size;
  GdkColor *colour = &carmen_black;
  carmen_world_point_t *draw_point = NULL;
  carmen_traj_point_t *simulator_object;

  if (the_map_view->internal_map == NULL)
    return;

  pixel_size = 1/map_view->rescale_size*
    map_view->internal_map->config.resolution;

  // carmen_fmax
  //    (map_view->internal_map->config.x_size/(double)map_view->port_size_x,
  //     map_view->internal_map->config.y_size/(double)map_view->port_size_y);

  //  pixel_size *= map_view->internal_map->config.resolution *
  //    (map_view->zoom/100.0);

  /*
   * Draw robot features
   */

  if (received_robot_pose()) {
    draw_particles(the_map_view, pixel_size);
    draw_gaussians(the_map_view);
    draw_lasers(the_map_view, pixel_size);
    draw_robot(the_map_view, pixel_size);
  }

  /*
   * Draw path
   */

  for (index = 1; index < num_path_points; index++) {
    if (path->map == NULL)
      break;

    carmen_map_graphics_draw_line(the_map_view, &path_colour, path+index-1,
				  path+index);

    if (nav_panel_config->draw_waypoints) {
      path_x_1 = *(path+index);
      path_x_2 = *(path+index);

      path_x_1.pose.x -= path->map->config.resolution;
      path_x_1.pose.y -= path->map->config.resolution;
      path_x_2.pose.x += path->map->config.resolution;
      path_x_2.pose.y += path->map->config.resolution;
      carmen_map_graphics_draw_line(the_map_view, &path_colour, &path_x_1,
				    &path_x_2);

      path_x_1.pose.y += path->map->config.resolution*2;
      path_x_2.pose.y -= path->map->config.resolution*2;
      carmen_map_graphics_draw_line(the_map_view, &path_colour, &path_x_1,
				    &path_x_2);
    }
  }

  /*
   * Draw goal
   */

  if (goal.pose.x > 0 && goal.pose.y > 0 && goal.map != NULL) {
    goal_size = nav_config->goal_size/2;
    if (goal_size/pixel_size < 5)
      goal_size = pixel_size*5;

    carmen_map_graphics_draw_circle(the_map_view, &goal_colour, TRUE, &goal,
				    goal_size);
    carmen_map_graphics_draw_circle(the_map_view, &carmen_black, FALSE,
				    &goal, goal_size);
  }

  /*
   * Draw simulator features
   */

  draw_simulated_robot(the_map_view, pixel_size);

  if (nav_panel_config->show_simulator_objects) {
    circle_size = robot_config->width/2.0;
    if (circle_size < pixel_size*5)
      circle_size = pixel_size*5;

    particle.map = the_map_view->internal_map;
    if (simulator_objects) {
      for (index = 0; index < simulator_objects->length; index++) {
	simulator_object = (carmen_traj_point_t *)
	  carmen_list_get(simulator_objects, index);
	particle.pose.x = simulator_object->x;
	particle.pose.y = simulator_object->y;
	if (black_and_white)
	  carmen_map_graphics_draw_circle(the_map_view, &carmen_grey, TRUE,
					  &particle, circle_size);
	else
	  carmen_map_graphics_draw_circle(the_map_view, &carmen_orange, TRUE,
					  &particle, circle_size);
	carmen_map_graphics_draw_circle(the_map_view, &carmen_black, FALSE,
					&particle, circle_size);
      }
    }
  }

  if (placement_status != ORIENTING_ROBOT &&
      placement_status != ORIENTING_PERSON &&
      placement_status != ORIENTING_SIMULATOR)
    return;

  /* Everything from here down is only used if we are orienting something.
     We have to draw the object itself (the robot, person, whatever) since
     in some cases the display hasn't actually published the fact that the
     feature has changed.
   */

  if (placement_status == ORIENTING_ROBOT) {
    if (carmen_get_time() - last_navigator_update > 30 &&
	carmen_get_time() - last_simulator_update < 30)
      draw_point = &simulator_trueposition;
    else
      draw_point = &robot;
    colour = &carmen_red;
  } else if (placement_status == ORIENTING_PERSON) {
    draw_point = &new_person;
    colour = &carmen_orange;
  } else if (placement_status == ORIENTING_SIMULATOR) {
    draw_point = &new_simulator;
    colour = &carmen_blue;
  }

  robot_size = robot_config->width/2.0;
  if (robot_size < pixel_size*5)
    robot_size = pixel_size*5;

  if (black_and_white) {
    carmen_map_graphics_draw_circle(the_map_view, &carmen_grey, TRUE,
				    draw_point, robot_size);
    carmen_map_graphics_draw_circle(the_map_view, &carmen_black, FALSE,
				    draw_point, robot_size);

    carmen_map_graphics_draw_line(the_map_view, &carmen_black, draw_point,
				  &cursor_pos);
  } else {
    carmen_map_graphics_draw_circle(the_map_view, colour, TRUE,
				    draw_point, robot_size);
    carmen_map_graphics_draw_circle(the_map_view, &carmen_black, FALSE,
				    draw_point, robot_size);

    carmen_map_graphics_draw_line(the_map_view, colour, draw_point,
				  &cursor_pos);
  }
}

static gint
motion_handler (GtkMapViewer *the_map_view, carmen_world_point_p world_point,
		GdkEventMotion *event __attribute__ ((unused)))
{
  char buffer[255];
  carmen_map_point_t point;
  carmen_map_p the_map;

  the_map = the_map_view->internal_map;

  carmen_world_to_map(world_point, &point);
  sprintf(buffer, "Grid Cell: %d, %d\n(%.1f m, %.1f m)", point.x, point.y,
	  point.x * the_map->config.resolution, point.y *
	  the_map->config.resolution);
  gtk_label_set_text(GTK_LABEL(cursor_status_label), buffer);
  if (the_map != NULL) {
    sprintf(buffer, "Value: %.2f", the_map->map[point.x][point.y]);
    gtk_label_set_text(GTK_LABEL(value_label), buffer);
  }

  if (placement_status == ORIENTING_ROBOT ||
      placement_status == ORIENTING_SIMULATOR ||
      placement_status == ORIENTING_SIMULATOR) {
    cursor_pos = *world_point;
    display_needs_updating = 1;
    do_redraw();
  }

  return TRUE;
}

void
resend_coords(GtkWidget *widget __attribute__ ((unused)),
	      gpointer data __attribute__ ((unused)))
{
  carmen_verbose("Robot: %.0f %.0f Goal: %.0f %.0f\n", last_robot.pose.x,
		 last_robot.pose.y, goal.pose.x, goal.pose.y);
  if (goal.pose.x > 0)
    navigator_set_goal(goal.pose.x, goal.pose.y);
  if (last_robot.pose.x > 0)
    navigator_update_robot(&last_robot);
}

static int
button_press_handler(GtkMapViewer *the_map_view __attribute__ ((unused)),
		     carmen_world_point_p world_point __attribute__ ((unused)),
		     GdkEventButton *event __attribute__ ((unused)))
{

  if (the_map_view->internal_map == NULL)
    return TRUE;

  return TRUE;
}

static int
button_release_handler(GtkMapViewer *the_map_view,
		       carmen_world_point_p world_point,
		       GdkEventButton *event __attribute__ ((unused)))
{
  double angle, speed;
  GdkCursor *cursor;

  if (the_map_view->internal_map == NULL)
    return TRUE;

  if (placement_status == PLACING_ROBOT ||
      (placement_status == NO_PLACEMENT &&
       ((event->button == 1 && (event->state & GDK_CONTROL_MASK)) ||
	(event->button == 3)))) {
    if (GTK_TOGGLE_BUTTON (autonomous_button)->active) {
      placement_status = NO_PLACEMENT;
      return TRUE;
    }

    world_point->pose.theta = robot.pose.theta;
    robot = *world_point;
    last_robot = *world_point;
    navigator_update_robot (world_point);
    if (placement_status == PLACING_ROBOT) {
      placement_status = ORIENTING_ROBOT;
      cursor = gdk_cursor_new(GDK_EXCHANGE);
      gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
    } else {
      cursor = gdk_cursor_new(GDK_LEFT_PTR);
      gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
    }
    return TRUE;
  }

  if (placement_status == PLACING_GOAL ||
      (placement_status == NO_PLACEMENT && event->button == 1)) {
    placement_status = NO_PLACEMENT;

    if (GTK_TOGGLE_BUTTON (autonomous_button)->active)
      return TRUE;

    navigator_set_goal(world_point->pose.x, world_point->pose.y);
    cursor = gdk_cursor_new(GDK_LEFT_PTR);
    gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
    return TRUE;
  }

  if (placement_status == PLACING_PERSON) {
    new_person = *world_point;
    cursor = gdk_cursor_new(GDK_EXCHANGE);
    gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
    placement_status = ORIENTING_PERSON;
    return TRUE;
  }

  if (placement_status == PLACING_SIMULATOR) {
    new_simulator = *world_point;
    cursor = gdk_cursor_new(GDK_EXCHANGE);
    gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
    placement_status = ORIENTING_SIMULATOR;
    return TRUE;
  }

  if (placement_status == ORIENTING_ROBOT ||
      (placement_status == NO_PLACEMENT &&
       ((event->button == 2 && (event->state & GDK_CONTROL_MASK)) ||
	(event->button == 3 && (event->state & GDK_CONTROL_MASK))))) {
    placement_status = NO_PLACEMENT;

    if (GTK_TOGGLE_BUTTON (autonomous_button)->active)
      return TRUE;

    if (carmen_get_time() - last_navigator_update > 30 &&
	carmen_get_time() - last_simulator_update < 30) {
      angle = atan2(world_point->pose.y - simulator_trueposition.pose.y,
		    world_point->pose.x - simulator_trueposition.pose.x);
      simulator_trueposition.pose.theta = angle;
      navigator_update_robot(&simulator_trueposition);
    } else {
      angle = atan2(world_point->pose.y - robot.pose.y,
		    world_point->pose.x - robot.pose.x);
      robot.pose.theta = angle;
      last_robot = robot;
      navigator_update_robot(&robot);
    }
    cursor = gdk_cursor_new(GDK_LEFT_PTR);
    gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
  }

  if (placement_status == ORIENTING_PERSON) {
    placement_status = NO_PLACEMENT;

    angle = atan2(world_point->pose.y - new_person.pose.y,
		  world_point->pose.x - new_person.pose.x);
    speed = hypot(world_point->pose.y - new_person.pose.y,
		  world_point->pose.x - new_person.pose.x);
    speed /= 10;
    new_person.pose.theta = angle;
    carmen_simulator_set_object(&(new_person.pose), speed,
				CARMEN_SIMULATOR_RANDOM_OBJECT);
    cursor = gdk_cursor_new(GDK_LEFT_PTR);
    gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
    return TRUE;
  }

  if (placement_status == ORIENTING_SIMULATOR) {
    placement_status = NO_PLACEMENT;
    angle = atan2(world_point->pose.y - new_person.pose.y,
		  world_point->pose.x - new_person.pose.x);
    new_simulator.pose.theta = angle;
    carmen_simulator_set_truepose(&(new_simulator.pose));
    cursor = gdk_cursor_new(GDK_LEFT_PTR);
    gdk_window_set_cursor (the_map_view->image_widget->window, cursor);
    return TRUE;
  }

  return TRUE;
}

static void
initialize_position(carmen_world_point_p point)
{
  point->pose.x = -1;
  point->pose.y = -1;
  point->pose.theta = 0.0;
  point->map = map_view->internal_map;
}

static void set_goal(GtkAction *action, gpointer user_data)
{
  char *name;
  int place_index;

  name = (char *)gtk_action_get_name(action);

  for (place_index = 0; place_index < placelist->num_places; place_index++) {
    if (strcmp(user_data, placelist->places[place_index].name) == 0) {
      navigator_set_goal_by_place(placelist->places+place_index);
      return;
    }
  }
}

static void set_location(GtkAction *action, gpointer user_data)
{
  carmen_world_point_t point;
  char *name;
  int place_index;

  name = (char *)gtk_action_get_name(action);

  if (strcmp(name, "GlobalLocalization") == 0) {
    carmen_verbose("Global localization\n");
    navigator_update_robot(NULL);
    return;
  }

  for (place_index = 0; place_index < placelist->num_places; place_index++) {
    if (strcmp(user_data, placelist->places[place_index].name) == 0) {
      if (placelist->places[place_index].type == CARMEN_NAMED_POSITION_TYPE)  {
	point.pose.x = placelist->places[place_index].x;
	point.pose.y = placelist->places[place_index].y;
	point.pose.theta = robot.pose.theta;
      } else {
	point.pose.x = placelist->places[place_index].x;
	point.pose.y = placelist->places[place_index].y;
	point.pose.theta = placelist->places[place_index].theta;
      }
      navigator_update_robot(&point);
    }
  }
}

static gint
save_image(gpointer data __attribute__ ((unused)),
	   guint action __attribute__ ((unused)),
	   GtkWidget *widget  __attribute__ ((unused)))
{
  int x_size, y_size;
  int x_start, y_start;
  static int counter = 0;
  char filename[255];

  x_start = map_view->x_scroll_adj->value;
  y_start = map_view->y_scroll_adj->value;
  x_size = carmen_fmin(gdk_pixbuf_get_width(map_view->current_pixbuf),
		       map_view->port_size_x);
  y_size = carmen_fmin(gdk_pixbuf_get_height(map_view->current_pixbuf),
		       map_view->port_size_y);

  sprintf(filename, "%s%02d.png",
	  carmen_extract_filename(map_view->internal_map->config.map_name),
	  counter++);

  if (display == CARMEN_NAVIGATOR_ENTROPY_v)
    carmen_graphics_write_pixmap_as_png(map_view->drawing_pixmap, filename,
					x_start, y_start, x_size, y_size);
  else if (display == CARMEN_NAVIGATOR_UTILITY_v)
    carmen_graphics_write_pixmap_as_png(map_view->drawing_pixmap, filename,
					x_start, y_start, x_size, y_size);
  else {
    carmen_graphics_write_pixmap_as_png(map_view->drawing_pixmap, filename,
					0, 0, x_size, y_size);
  }

  return 1;
}

int
navigator_graphics_init(int argc, char *argv[],
			carmen_localize_globalpos_message *msg,
			carmen_robot_config_t *robot_conf_param,
			carmen_navigator_config_t *nav_conf_param,
			carmen_navigator_panel_config_t *nav_panel_conf_param)
{
  /* GtkWidget is the storage type for widgets */
  GtkWidget *main_box;
  GtkWidget *panel_box;
  GtkWidget *status_box;
  GtkWidget *button_box;
  GtkWidget *label_box;
  GtkWidget *menubar;
  GtkWidget *vseparator, *hseparator;
  /*   GtkWidget *resend_button; */
  GtkWidget *menu_item;
  int index;
  int sync_mode_var = 0;

  gtk_init (&argc, &argv);

  carmen_graphics_setup_colors();
  robot_colour = DEFAULT_ROBOT_COLOUR;
  goal_colour = DEFAULT_GOAL_COLOUR;
  path_colour = DEFAULT_PATH_COLOUR;
  people_colour = DEFAULT_PEOPLE_COLOUR;

  nav_panel_config = nav_panel_conf_param;
  if (nav_panel_config->initial_map_zoom < 1.0 ||
      nav_panel_config->initial_map_zoom > 100.0)
    nav_panel_config->initial_map_zoom = 100.0;

  /* Create a new window */
  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

  g_signal_connect (GTK_OBJECT (window), "destroy",
		    G_CALLBACK(gtk_main_quit), "WM destroy");

  gtk_window_set_title (GTK_WINDOW (window), "CARMEN Planner");
  g_signal_connect (GTK_OBJECT (window), "delete_event",
		    G_CALLBACK(delete_event), NULL);
  gtk_container_set_border_width (GTK_CONTAINER (window), 0);

  main_box = gtk_vbox_new(FALSE, 0);
  gtk_container_border_width (GTK_CONTAINER (main_box), 0);
  gtk_container_add (GTK_CONTAINER (window), main_box);

  menubar = get_main_menu ();
  gtk_box_pack_start (GTK_BOX (main_box), menubar, FALSE, FALSE, 0);

  panel_box = gtk_hbox_new(FALSE, 0);
  gtk_container_border_width (GTK_CONTAINER (panel_box), 5);
  gtk_container_add (GTK_CONTAINER (main_box), panel_box);

  map_view = carmen_map_graphics_new_viewer(400, 400, nav_panel_config->initial_map_zoom);
  gtk_box_pack_start(GTK_BOX (panel_box), map_view->map_box, TRUE, TRUE, 0);

  carmen_map_graphics_add_motion_event
    (map_view, (carmen_graphics_mapview_callback_t)motion_handler);
  carmen_map_graphics_add_button_release_event
    (map_view, (carmen_graphics_mapview_callback_t)button_release_handler);
  carmen_map_graphics_add_button_press_event
    (map_view, (carmen_graphics_mapview_callback_t)button_press_handler);
  carmen_map_graphics_add_drawing_func
    (map_view, (carmen_graphics_mapview_drawing_func_t)draw_robot_objects);

  vseparator = gtk_vseparator_new();
  gtk_widget_set_usize(vseparator, 5,
		       map_view->image_widget->allocation.height);
  gtk_box_pack_start(GTK_BOX(panel_box), vseparator, FALSE, FALSE, 0);

  label_box = gtk_vbox_new(FALSE, 0);
  gtk_widget_set_usize(label_box, BUTTON_WIDTH,
		       map_view->image_widget->allocation.height);
  gtk_container_border_width (GTK_CONTAINER (label_box), 0);

  gtk_box_pack_start(GTK_BOX (panel_box), label_box, FALSE, FALSE, 0);

  status_box = construct_status_frame(label_box);

  robot_status_label = new_label("Robot position: 0 0", status_box);
  robot_speed_label = new_label("Velocity: 0 m/s 0 rad/s", status_box);
  goal_status_label = new_label("Goal position: 0 0", status_box);
  cursor_status_label = new_label("Grid Cell:", status_box);
  value_label = new_label("Value: 0.0", status_box);

  button_box = gtk_hbutton_box_new();
  gtk_container_border_width (GTK_CONTAINER (button_box), 5);
  gtk_box_pack_start (GTK_BOX (label_box), button_box, FALSE, FALSE, 0);
  gtk_button_box_set_layout (GTK_BUTTON_BOX (button_box), GTK_BUTTONBOX_SPREAD);
  gtk_button_box_set_spacing (GTK_BUTTON_BOX (button_box), 10);
  gtk_button_box_set_child_size (GTK_BUTTON_BOX (button_box), BUTTON_WIDTH*.4,
				 BUTTON_HEIGHT);

  place_robot_button = gtk_button_new_with_label ("Place Robot");
  g_signal_connect (GTK_OBJECT (place_robot_button), "clicked",
		    G_CALLBACK(place_robot), NULL);
  gtk_box_pack_start(GTK_BOX(button_box), place_robot_button, FALSE, FALSE, 0);
  place_goal_button = gtk_button_new_with_label ("Place Goal");
  g_signal_connect (GTK_OBJECT (place_goal_button), "clicked",
		    G_CALLBACK(place_goal), NULL);
  gtk_box_pack_end(GTK_BOX(button_box), place_goal_button, FALSE, FALSE, 0);

  button_box = gtk_vbutton_box_new();
  gtk_container_border_width (GTK_CONTAINER (button_box), 5);
  gtk_box_pack_start (GTK_BOX (label_box), button_box, FALSE, FALSE, 0);
  gtk_button_box_set_layout (GTK_BUTTON_BOX (button_box), GTK_BUTTONBOX_START);
  gtk_button_box_set_spacing (GTK_BUTTON_BOX (button_box), 10);
  gtk_button_box_set_child_size (GTK_BUTTON_BOX (button_box), BUTTON_WIDTH,
				 BUTTON_HEIGHT);

  autonomous_button = gtk_toggle_button_new_with_label ("Go");
  g_signal_connect (GTK_OBJECT (autonomous_button), "clicked",
		    G_CALLBACK(go_autonomous), (gpointer)"Autonomous");
  gtk_box_pack_start(GTK_BOX(button_box), autonomous_button, FALSE, FALSE, 0);

  hseparator = gtk_hseparator_new();
  gtk_box_pack_start (GTK_BOX (label_box), hseparator, FALSE, FALSE, 0);

  //  gtk_container_add(GTK_CONTAINER(label_box), hseparator);
  gtk_widget_set_usize(hseparator, BUTTON_WIDTH, 5);

  simulator_box = gtk_vbutton_box_new();
  gtk_container_border_width (GTK_CONTAINER (simulator_box), 5);
  //  gtk_container_add (GTK_CONTAINER (label_box), simulator_box);
  gtk_box_pack_start (GTK_BOX (label_box), simulator_box, FALSE, FALSE, 0);

  gtk_button_box_set_layout (GTK_BUTTON_BOX (simulator_box),
			     GTK_BUTTONBOX_START);
  gtk_button_box_set_spacing (GTK_BUTTON_BOX (simulator_box), 10);
  gtk_button_box_set_child_size (GTK_BUTTON_BOX (simulator_box), BUTTON_WIDTH,
				 BUTTON_HEIGHT);

  place_simulator_button = gtk_button_new_with_label ("Place Simulator");
  g_signal_connect (GTK_OBJECT (place_simulator_button), "clicked",
		    G_CALLBACK(place_simulator), NULL);
  gtk_box_pack_start(GTK_BOX(simulator_box), place_simulator_button,
		     FALSE, FALSE, 0);
  next_tick_button = gtk_button_new_with_label ("Next Tick");
  g_signal_connect (GTK_OBJECT (next_tick_button), "clicked",
		    G_CALLBACK(next_tick), NULL);
  gtk_box_pack_start(GTK_BOX(simulator_box), next_tick_button,
		     FALSE, FALSE, 0);
  sync_mode_button = gtk_toggle_button_new_with_label ("Sync Mode");
  carmen_param_set_module(NULL);
  carmen_param_get_onoff("simulator_sync_mode", &sync_mode_var, NULL);
  carmen_param_subscribe_onoff("simulator", "sync_mode", NULL,
			       (carmen_param_change_handler_t)
			       sync_mode_change_handler);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON (sync_mode_button),
			       sync_mode_var);
  g_signal_connect (GTK_OBJECT (sync_mode_button), "clicked",
		    G_CALLBACK(sync_mode), (gpointer)"Autonomous");
  gtk_box_pack_start(GTK_BOX(simulator_box), sync_mode_button, FALSE,
		     FALSE, 0);

  filler_box = gtk_vbox_new(FALSE, 0);
  gtk_widget_set_usize(filler_box, BUTTON_WIDTH, 100);
  gtk_container_border_width (GTK_CONTAINER (filler_box), 0);
  //  gtk_container_add (GTK_CONTAINER (label_box), filler_box);
  gtk_box_pack_start (GTK_BOX (label_box), filler_box, FALSE, FALSE, 0);

  gtk_widget_show (filler_box);

  for(index = 0; index < GRADIENT_COLORS; index++)
    RedBlueGradient[index] =
      carmen_graphics_add_color_rgb(255 - 255 * index / (float)GRADIENT_COLORS,
				    0, 255 * index / (float)GRADIENT_COLORS);

  gtk_widget_show_all (window);
  gtk_widget_hide(simulator_box);
  simulator_hidden = 1;

  gtk_widget_grab_focus(window);

  menu_item =
    gtk_ui_manager_get_widget(ui_manager, "/ui/MainMenu/FileMenu/StopFilming");
  gtk_widget_hide(menu_item);

  globalpos = msg;

  robot_config = robot_conf_param;
  nav_config = nav_conf_param;

  cursor_pos.map = NULL;

  is_graphics_up = 1;

  return(0);
}

void
navigator_graphics_add_ipc_handler(GdkInputFunction handle_ipc)
{
  carmen_graphics_update_ipc_callbacks(handle_ipc);
}

void
navigator_graphics_change_map(carmen_map_p new_map)
{
  char buffer[1024];

  if (!is_graphics_up)
    return;

  carmen_map_graphics_add_map(map_view, new_map, 0);

  initialize_position(&robot);
  initialize_position(&goal);
  if (people)
    people->length = 0;
  initialize_position(&last_robot);

  sprintf(buffer, "Map: %s",
	  carmen_extract_filename(new_map->config.map_name));
  gtk_label_set_text(GTK_LABEL(map_status_label), buffer);
}

void
navigator_graphics_display_map(float *data, carmen_navigator_map_t type)
{
  char name[100];
  int flags = 0;

  if (!is_graphics_up)
    return;

  display = type;
  switch (type) {
  case CARMEN_NAVIGATOR_MAP_v:
    strcpy(name, "Map");
    break;
  case CARMEN_NAVIGATOR_ENTROPY_v:
    strcpy(name, "Entropy");
    flags = CARMEN_GRAPHICS_RESCALE;
    break;
  case CARMEN_NAVIGATOR_COST_v:
    strcpy(name, "Costs");
    flags = CARMEN_GRAPHICS_RESCALE;
    break;
  case CARMEN_NAVIGATOR_UTILITY_v:
    strcpy(name, "Utility");
    flags = CARMEN_GRAPHICS_RESCALE | CARMEN_GRAPHICS_INVERT;
    break;
  case CARMEN_LOCALIZE_LMAP_v:
    strcpy(name, "Likelihood Map");
    flags = CARMEN_GRAPHICS_RESCALE;
    break;
  case CARMEN_LOCALIZE_GMAP_v:
    strcpy(name, "Global Likelihood Map");
    flags = CARMEN_GRAPHICS_RESCALE;
    break;
  default:
    return;
  }

  if (black_and_white)
    flags |= CARMEN_GRAPHICS_BLACK_AND_WHITE;

  carmen_map_graphics_modify_map(map_view, data, flags);
}

void
navigator_graphics_add_placelist(carmen_map_placelist_p new_placelist)
{
  char name[1024], label[1024], menu_path[1024];
  int index;
  int *merge_uid, new_merge_uid;
  char *underscore;
  GtkAction *action;

  carmen_verbose("Received %d places\n", new_placelist->num_places);

  if (place_action_uids != NULL) {
    for (index = 0; index < place_action_uids->length; index++) {
      merge_uid = (int *)carmen_list_get(place_action_uids, index);
      gtk_ui_manager_remove_ui(ui_manager, *merge_uid);

      action = carmen_list_get(goal_actions, index);
      gtk_action_group_remove_action(goal_action_group, action);
      g_object_unref(action);

      action = carmen_list_get(start_actions, index);
      gtk_action_group_remove_action(start_action_group, action);
      g_object_unref(action);
    }
    free(placelist->places);
  } else {
    place_action_uids = carmen_list_create
      (sizeof(int), new_placelist->num_places);
    goal_actions = carmen_list_create
      (sizeof(GtkAction *), new_placelist->num_places);
    start_actions = carmen_list_create
      (sizeof(GtkAction *), new_placelist->num_places);
    placelist = (carmen_map_placelist_p)
      calloc(1, sizeof(carmen_map_placelist_t));
    carmen_test_alloc(placelist);
    goal_action_group = gtk_action_group_new("Goals");
    start_action_group = gtk_action_group_new("StartLocations");
    gtk_ui_manager_insert_action_group (ui_manager, goal_action_group, 1);
    gtk_ui_manager_insert_action_group (ui_manager, start_action_group, 2);
  }

  placelist->num_places = new_placelist->num_places;
  place_action_uids->length = 0;
  if (placelist->num_places == 0)
    return;

  placelist->places = (carmen_place_p)calloc(placelist->num_places,
					     sizeof(carmen_place_t));
  carmen_test_alloc(placelist->places);
  memcpy(placelist->places, new_placelist->places,
	 sizeof(carmen_place_t)*placelist->num_places);

  for (index = 0; index < placelist->num_places; index++) {
    strcpy(label, placelist->places[index].name);
    label[0] = toupper(label[0]);
    do {
      underscore = strchr(label, '_');
      if (underscore)
	*underscore = ' ';
    } while (underscore);

    sprintf(name, "Goal%s", placelist->places[index].name);
    action = gtk_action_new(name, label, NULL, NULL);
    gtk_action_group_add_action(goal_action_group, action);
    carmen_list_add(goal_actions, &action);
    g_signal_connect(action, "activate", G_CALLBACK(set_goal),
		     placelist->places[index].name);

    sprintf(menu_path, "/ui/MainMenu/GoalMenu/");
    new_merge_uid = gtk_ui_manager_new_merge_id(ui_manager);
    gtk_ui_manager_add_ui(ui_manager, new_merge_uid, menu_path,
			  name, name, GTK_UI_MANAGER_MENUITEM, FALSE);
    carmen_list_add(place_action_uids, &new_merge_uid);

    sprintf(name, "Start%s", placelist->places[index].name);
    action = gtk_action_new(name, label, NULL, NULL);
    gtk_action_group_add_action(start_action_group, action);
    carmen_list_add(start_actions, &action);
    g_signal_connect(action, "activate", G_CALLBACK(set_location),
		     placelist->places[index].name);

    sprintf(menu_path, "/ui/MainMenu/StartLocationMenu/");
    new_merge_uid = gtk_ui_manager_new_merge_id(ui_manager);
    gtk_ui_manager_add_ui(ui_manager, new_merge_uid, menu_path,
			  name, name, GTK_UI_MANAGER_MENUITEM, FALSE);
    carmen_list_add(place_action_uids, &new_merge_uid);

  }
}

void navigator_graphics_update_dynamics(void)
{
  display_needs_updating = 1;
  do_redraw();
}

void navigator_graphics_initialize_dynamics(carmen_list_t *new_people)
{
  people = new_people;
}

void
navigator_graphics_update_display(carmen_traj_point_p new_robot,
				  carmen_world_point_p new_goal,
				  int autonomous)
{
  char buffer[255];
  double robot_distance = 0.0, goal_distance = 0.0;
  carmen_world_point_t new_robot_w;
  static int previous_width = 0, previous_height = 0;
  double delta_angle;
  int autonomous_change = 0;
  double adjust_distance;

  if (!is_graphics_up)
    return;

  if (!autonomous &&
      gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON (autonomous_button))) {
    ignore_click = 1;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON (autonomous_button), 0);
    label_autonomy_button("Go");
    autonomous_change = 1;
  }

  if (autonomous &&
      !gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON (autonomous_button))) {
    ignore_click = 1;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON (autonomous_button), 1);
    label_autonomy_button("Stop");
    autonomous_change = 1;
  }

  if (!map_view->internal_map)
    return;

  if (time_of_simulator_update - carmen_get_time() > 30)
    {
      gtk_widget_hide(simulator_box);
      gtk_widget_show(filler_box);
      simulator_hidden = 1;
    }

  adjust_distance = carmen_fmax
    (map_view->internal_map->config.x_size/(double)map_view->port_size_x,
     map_view->internal_map->config.y_size/(double)map_view->port_size_y);

  adjust_distance *= map_view->internal_map->config.resolution *
    (map_view->zoom/100.0);
  adjust_distance *= 10;


  new_robot_w.pose.x = new_robot->x;
  new_robot_w.pose.y = new_robot->y;
  new_robot_w.pose.theta = new_robot->theta;
  new_robot_w.map = map_view->internal_map;
  robot_distance = carmen_distance_world(&new_robot_w, &(map_view->centre));

  if (nav_panel_config->track_robot &&
      (robot_distance > adjust_distance ||
       previous_width != map_view->image_widget->allocation.width ||
       previous_height != map_view->image_widget->allocation.height))
    carmen_map_graphics_adjust_scrollbars(map_view, &robot);

  robot_distance = carmen_distance_world(&new_robot_w, &robot);
  delta_angle =
    carmen_normalize_theta(new_robot_w.pose.theta-robot.pose.theta);

  robot = new_robot_w;
  robot_traj = *new_robot;

  if (new_goal) {
    goal_distance = carmen_distance_world(new_goal, &goal);
    goal = *new_goal;
  } else
    goal_distance = 0.0;

  previous_width = map_view->image_widget->allocation.width;
  previous_height = map_view->image_widget->allocation.height;

  if (autonomous_change || robot_distance > 1.0 || goal_distance > 1.0 ||
      fabs(delta_angle) > carmen_degrees_to_radians(0.01) )
    display_needs_updating = 1;

  sprintf(buffer, "Robot: %5.1f m, %5.1f m, %6.2f", robot.pose.x,
	  robot.pose.y, carmen_radians_to_degrees(robot.pose.theta));
  gtk_label_set_text(GTK_LABEL(robot_status_label), buffer);
  sprintf(buffer, "Velocity: %5.1f m/s, %5.1f deg/s", robot_traj.t_vel,
	  carmen_radians_to_degrees(robot_traj.r_vel));
  gtk_label_set_text(GTK_LABEL(robot_speed_label), buffer);
  sprintf(buffer, "Goal: %.1f m, %.1f m", goal.pose.x, goal.pose.y);
  gtk_label_set_text(GTK_LABEL(goal_status_label), buffer);

  last_navigator_update = carmen_get_time();

  do_redraw();
}

void
navigator_graphics_update_plan(carmen_traj_point_p new_plan, int plan_length)
{
  int index;

  if (map_view->internal_map == NULL)
    return;

  if (path != NULL)
    {
      free(path);
      path = NULL;
    }

  num_path_points = plan_length;

  if (plan_length > 0)
    {
      path = (carmen_world_point_t *)
	calloc(plan_length, sizeof(carmen_world_point_t));
      carmen_test_alloc(path);
      carmen_verbose("Got path of length %d\n", plan_length);
      for (index = 0; index < num_path_points; index++)
	{
	  path[index].pose.x = new_plan[index].x;
	  path[index].pose.y = new_plan[index].y;
	  path[index].pose.theta = new_plan[index].theta;
	  path[index].map = map_view->internal_map;
	  carmen_verbose("%.1f %.1f\n", path[index].pose.x,
			 path[index].pose.y);
	}
    }
  else
    num_path_points = 0;

  display_needs_updating = 1;
  do_redraw();
}

carmen_world_point_p
navigator_graphics_get_current_path(void)
{
  return path;
}

void
navigator_graphics_update_simulator_truepos(carmen_point_t truepose)
{
  time_of_simulator_update = carmen_get_time();
  if (simulator_hidden)
    {
      gtk_widget_show_all(simulator_box);
      gtk_widget_hide(filler_box);
      simulator_hidden = 0;
      if (!GTK_TOGGLE_BUTTON(sync_mode_button)->active)
	gtk_widget_hide(next_tick_button);
    }

  simulator_trueposition.pose = truepose;
  simulator_trueposition.map = map_view->internal_map;
  last_simulator_update = carmen_get_time();
  display_needs_updating = 1;
  do_redraw();
}

void navigator_graphics_update_simulator_objects(int num_objects,
						 carmen_traj_point_t
						 *objects_list)
{
  int i;

  if (simulator_objects == NULL) {
    if (num_objects == 0)
      return;
    simulator_objects = carmen_list_create
      (sizeof(carmen_traj_point_t), num_objects);
  }

  simulator_objects->length = 0;

  for (i = 0; i < num_objects; i++)
    carmen_list_add(simulator_objects, objects_list+i);

  display_needs_updating = 1;
  do_redraw();
}

static void
sync_mode_change_handler(char *module __attribute__ ((unused)),
			 char *variable __attribute__ ((unused)),
			 char *value)
{
  int new_value;

  if (strlen(value) >=2 && strncmp(value, "on", 2) == 0)
    new_value = 1;
  else if (strlen(value) >= 3 && strncmp(value, "off", 3) == 0)
    new_value = 0;
  else
    return;

  GTK_TOGGLE_BUTTON(sync_mode_button)->active = new_value;

  if (simulator_hidden)
    return;

  if (GTK_TOGGLE_BUTTON(sync_mode_button)->active)
    gtk_widget_show(next_tick_button);
  else
    gtk_widget_hide(next_tick_button);
}

void
navigator_graphics_start(void)
{
  gtk_main ();
}
