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

#ifndef gui_h
#define gui_h

#define HELP_FILE_NAME "vasco_help.txt"

extern GdkGC *drawing_gc;

extern GdkColor gdkcolor_black;
extern GdkColor gdkcolor_red;
extern GdkColor gdkcolor_blue;
extern GdkColor gdkcolor_green;

extern GtkWidget *canvas;
extern GtkWidget *scrolled_window;
extern GdkPixmap *pixmap;

#define DEFAULT_DRAWING_AREA_WIDTH 382
#define DEFAULT_DRAWING_AREA_HEIGHT 382

extern int canvas_width;
extern int canvas_height;

extern double map_width;
extern double map_height;

extern GtkWidget *scan_range_min_scale;
extern GtkWidget *scan_range_mid_scale;
extern GtkWidget *scan_range_max_scale;
extern GtkObject *scan_range_min_adj;
extern GtkObject *scan_range_mid_adj;
extern GtkObject *scan_range_max_adj;

extern GtkWidget *status_bar, *progress_bar;

extern int button1;

void graphics_init();
guint status_print(const char *message, const char *context);

#endif
