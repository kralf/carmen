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

#include "map_graphics.h"
#include "map_io.h"

static const int DEFAULT_WINDOW_HEIGHT = 500;
static const int DEFAULT_WINDOW_WIDTH = 500;
static GtkWidget *window;
static GtkWidget *notebook;
static carmen_map_t maps[50];
static char* map_filenames[50];
static int nmaps = 0;
static char *map_filename;
static GtkWidget *filew;
static carmen_world_point_t p0,p1;
static int is_insert_door, if_pressed, in_map, is_save;

static carmen_hmap_t hmap;

/*
  store doors in hmap.links (with num_links)
   - hmap.links[i].points[0] will be map1's p0 for the ith door
   - hmap.links[i].points[1] will be map1's p1 for the ith door
   - hmap.links[i].points[2] will be map2's p0 for the ith door
   - hmap.links[i].points[3] will be map2's p1 for the ith door
  store map names in zone_names (with num_zones)
  
*/
static void create_link(carmen_world_point_t p0, carmen_world_point_t p1, int curr_page)
{
  int num = hmap.num_links;
  if(!in_map)
  {
    num = hmap.num_links++;
    hmap.links = (carmen_hmap_link_p) realloc(hmap.links, hmap.num_links * sizeof(carmen_hmap_link_t));
    carmen_test_alloc(hmap.links);
    hmap.links[num].keys = (int*) malloc(2 * sizeof(int));
    carmen_test_alloc(hmap.links[num].keys);
    hmap.links[num].points = (carmen_point_p) malloc(4 * sizeof(carmen_point_t));  
    carmen_test_alloc(hmap.links[num].points);
    hmap.links[num].type = CARMEN_HMAP_LINK_DOOR;
    hmap.links[num].degree = 2;
  }
  else
    num--;
  hmap.links[num].keys[in_map] = curr_page;   
  hmap.links[num].points[2*in_map] = p0.pose;
  hmap.links[num].points[2*in_map +1] = p1.pose;
  in_map = !in_map;
}

static void draw_graph(GtkMapViewer *the_map_view  __attribute__ ((unused)))
{
  int i;
  int curr_page = gtk_notebook_get_current_page (GTK_NOTEBOOK(notebook));
  carmen_world_point_t temp1, temp2;

  for (i = 0; i < hmap.num_links; i++) {
    if (hmap.links[i].type == CARMEN_HMAP_LINK_DOOR) {
      if(hmap.links[i].keys[0] == curr_page)
      {
        temp1.pose = hmap.links[i].points[0];
        temp1.map = &maps[curr_page];
        temp2.pose = hmap.links[i].points[1];
        temp2.map = &maps[curr_page];
        carmen_map_graphics_draw_line(the_map_view, &carmen_red, &temp1,
          &temp2);
      }
      else if(hmap.links[i].keys[1] == curr_page)
      {
        temp1.pose = hmap.links[i].points[2];
	temp1.map = &maps[curr_page];
        temp2.pose = hmap.links[i].points[3];
        temp2.map = &maps[curr_page];
        carmen_map_graphics_draw_line(the_map_view, &carmen_red, &temp1, &temp2);
      }
    }
  }

  if(if_pressed)
    carmen_map_graphics_draw_line(the_map_view, &carmen_red, &p0, &p1);
}

static void release_handler(GtkMapViewer *the_map_view, 
			   carmen_world_point_p p, 
			    GdkEventButton *event __attribute__ ((unused)))
{
  if(is_insert_door == 1)
  {
    p1 = *p;
    create_link(p0, p1, gtk_notebook_get_current_page (GTK_NOTEBOOK
      (notebook)));
    carmen_map_graphics_redraw(the_map_view);
    is_insert_door = 0;
    if_pressed = 0;
  }
}  

static void motion_handler(GtkMapViewer *the_map_view, 
			   carmen_world_point_p p, 
			    GdkEventButton *event __attribute__ ((unused)))
{
  if(is_insert_door == 1 && if_pressed == 1)
  { 
    p1 = *p;
    carmen_map_graphics_redraw(the_map_view);
  }
}
 

static void press_handler(GtkMapViewer *the_map_view  __attribute__ ((unused)), 
			   carmen_world_point_p p, 
			   GdkEventButton *event __attribute__ ((unused)))
{
  if(is_insert_door == 1)
  {
    p0 = *p;
    if_pressed = 1;
  }
}

static void loadmap()
{
  
  GtkMapViewer *map_view;
  GtkWidget *tab; 
  if (carmen_map_file(map_filename) != 0) {
    if (carmen_map_read_gridmap_chunk(map_filename, &maps[nmaps]) >= 0)
    {
      map_view = carmen_map_graphics_new_viewer((&maps[nmaps])->config.x_size*2/3.0,
				 (&maps[nmaps])->config.y_size*2/3.0, 100.0);

      hmap.zone_names = (char **) realloc(hmap.zone_names, sizeof(char *) * (hmap.num_zones+1));
      carmen_test_alloc(hmap.zone_names);
      map_filenames[nmaps] = carmen_new_string(map_filename);
      hmap.zone_names[hmap.num_zones] = carmen_new_string(strrchr(map_filename,'/')+1);
      // *strstr(hmap.zone_names[hmap.num_zones], ".map") = '\0';
      hmap.num_zones++;

      tab = gtk_label_new(hmap.zone_names[hmap.num_zones-1]);
      gtk_notebook_append_page(GTK_NOTEBOOK(notebook), map_view->map_box, tab);

      carmen_map_graphics_add_drawing_func(map_view,
        (carmen_graphics_mapview_drawing_func_t)draw_graph);
      carmen_map_graphics_add_button_press_event(map_view, 
        (carmen_graphics_mapview_callback_t) press_handler);
      carmen_map_graphics_add_motion_event(map_view, 
        (carmen_graphics_mapview_callback_t) motion_handler);
      carmen_map_graphics_add_button_release_event(map_view, 
        (carmen_graphics_mapview_callback_t) release_handler);
      gtk_widget_draw(notebook, NULL);
      gtk_widget_show(map_view->image_widget);
      while(gtk_events_pending()) {
	gtk_main_iteration_do(TRUE);
	usleep(10000);
      }
      
     carmen_map_graphics_add_map(map_view, &maps[nmaps], 0);
     nmaps++;
    }
    else carmen_warn("Invalid gridmap chunk\n");
  }
  else carmen_warn("Invalid map file\n");
}

static int save_hmap()
{
  carmen_FILE *out_fp;
  int i;

  out_fp = carmen_fopen(map_filename, "w");
  if (out_fp == NULL)
    carmen_die("Error: Can't open file %s for writing\n", map_filename);

  if(carmen_map_write_comment_chunk(out_fp, 0, 0, 0, NULL, NULL) < 0)
    return -1;
  if (carmen_map_write_id(out_fp) < 0)
    return -1;

  if (carmen_map_write_hmap_chunk(out_fp, &hmap) < 0)
    carmen_die("Error: couldn't write hmap chunk\n");

  carmen_fclose(out_fp);

  for (i = 0; i < hmap.num_zones; i++) {
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_GRIDMAP_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], map_filename, CARMEN_MAP_GRIDMAP_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name gridmap chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_OFFLIMITS_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], map_filename, CARMEN_MAP_OFFLIMITS_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name offlimits chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_PLACES_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], map_filename, CARMEN_MAP_PLACES_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name places chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_EXPECTED_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], map_filename, CARMEN_MAP_EXPECTED_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name expected chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_CREATOR_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], map_filename, CARMEN_MAP_CREATOR_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name creator chunk\n");
  }
  return 1;
}
void store_filename(GtkWidget *w __attribute__ ((unused)), GtkFileSelection* filew)
{
  /*-- Extract filename from the file selection dialog --*/
  int i;
  int insert_flag = 1;
  map_filename = carmen_new_string(gtk_file_selection_get_filename(GTK_FILE_SELECTION(filew)));
  gtk_widget_hide(GTK_WIDGET(filew));
  if(!is_save)
  {
    for(i = 0; i < nmaps; i++)
      if( strcmp(map_filename, maps[i].config.map_name) == 0)
	insert_flag = 0;
    printf("***loadmap**\n");
    if(insert_flag == 1)
      loadmap();
  }
  else
    save_hmap();
  is_save = 0;
}


static void loadmap_file()
{ 
  gtk_widget_show(filew);
}

static void savemap_file()
{
  is_save = 1;
  gtk_widget_show(filew);
}

  

static GtkWidget *notebook_init()
{
  notebook = gtk_notebook_new();
  gtk_notebook_set_scrollable(GTK_NOTEBOOK(notebook), TRUE);
  gtk_notebook_set_tab_pos(GTK_NOTEBOOK(notebook), GTK_POS_LEFT);

  /*tab = gtk_label_new(map_name);
  gtk_notebook_append_page(GTK_NOTEBOOK(notebook), scrolled_window, tab);
  gtk_notebook_append_page(GTK_NOTEBOOK(notebook), scrolled_window, tab); */
  return notebook;
}
  
static void insert_door()
{
  is_insert_door = 1;
}


static GtkWidget *menubar_init(GtkWidget *window)
{
  GtkItemFactory *item_factory;
  GtkAccelGroup *accel_group;
  GtkWidget *menubar;
  gint nmenu_items;
  GtkItemFactoryEntry menu_items[] = {
    {"/_File", NULL, NULL, 0, "<Branch>"},
    {"/File/_Load maps", "<control>L", loadmap_file, 0, NULL},
    {"/File/", NULL, NULL, 0, "<Separator>"},
    {"/File/_Save"," <control>S", savemap_file, 0, NULL},
    {"/File/", NULL, NULL, 0, "<Separator>"},
    {"/File/_Quit", "<control>Q", gtk_main_quit, 0, NULL},
    {"/Tools", NULL, NULL, 0, "<Branch>"},
    {"/Tools/Insert_ door","<control>D", insert_door, 0, NULL}, 
  };
  nmenu_items = sizeof(menu_items)/sizeof(menu_items[0]);
  accel_group = gtk_accel_group_new();
  item_factory = gtk_item_factory_new(GTK_TYPE_MENU_BAR, "<main>", 
				      accel_group);
  gtk_item_factory_create_items(item_factory, nmenu_items, menu_items, NULL);
  gtk_window_add_accel_group(GTK_WINDOW(window), accel_group);

  menubar = gtk_item_factory_get_widget(item_factory, "<main>");
  return menubar;
}
static void window_destroy(GtkWidget *w __attribute__ ((unused)),
			   gpointer p __attribute__ ((unused))) {
  gtk_main_quit();
}
static void gui_init() 
{

  GtkWidget *vbox;
  char title[255];
  is_insert_door = 0;
  if_pressed = 0;
  in_map = 0;
  is_save = 0;
  hmap.links = NULL;
  hmap.zone_names = NULL;
  hmap.num_links = 0;
  hmap.num_zones = 0;
  filew = gtk_file_selection_new ("File selection");
  gtk_signal_connect (GTK_OBJECT (GTK_FILE_SELECTION (filew)->ok_button),
                        "clicked", GTK_SIGNAL_FUNC (store_filename), filew );
  
  gtk_signal_connect_object (GTK_OBJECT (GTK_FILE_SELECTION (filew)->cancel_button),
                               "clicked", (GtkSignalFunc) gtk_widget_hide,
                               GTK_OBJECT (filew));
  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_default_size(GTK_WINDOW(window), DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
  sprintf(title, "hmap_editor");
  gtk_window_set_title(GTK_WINDOW(window), title);
  gtk_signal_connect(GTK_OBJECT(window), "destroy",
               GTK_SIGNAL_FUNC(window_destroy), NULL); 

  vbox = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), menubar_init(window), FALSE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), notebook_init(), TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(window), vbox);
  gtk_widget_show_all(window);
}

int main(int argc, char *argv[])
{
  int i;

  gtk_init(&argc, &argv);
  carmen_graphics_setup_colors();
  gui_init();

  if (argc > 1) {
    for (i = 1; i < argc; i++) {
      map_filename = argv[i];
      loadmap();
    }
  }

  gtk_main();
  return 0; 
}


