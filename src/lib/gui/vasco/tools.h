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

#ifndef tools_h
#define tools_h

#include <carmen/carmen_graphics.h>

#define TOOL_NONE      0
#define TOOL_SHIFT     1
#define TOOL_ROTATE    2
#define TOOL_STRETCH   3
#define TOOL_BEND      4
#define TOOL_DELETE    5
#define TOOL_ZOOM_IN   6
#define TOOL_ZOOM_OUT  7

extern long tool;
extern double button1_x, button1_y;

extern GtkWidget *shift_button;
extern GtkWidget *rotate_button;
extern GtkWidget *stretch_button;
extern GtkWidget *bend_button;
extern GtkWidget *zoom_in_button;
extern GtkWidget *zoom_out_button;

void set_tool(GtkWidget *widget, gpointer data);
void shift_scans(double x, double y);
void shift_scans_by_motion(int x, int y);
void rotate_scans(double radians);
void rotate_scans_by_motion(int x, int y);
void stretch_scans(double x, double y);
void stretch_scans_by_motion(int x, int y);
void bend_scans(double radians);
void bend_scans_by_motion(int x, int y);
void zoom_in(int x, int y);
void zoom_out(int x, int y);

#endif
