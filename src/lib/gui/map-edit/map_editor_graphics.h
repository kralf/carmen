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

/***********************************
 * map_editor_graphics deals with  *
 * the pixmaps and the main window *
 * which display the map.          *
 ***********************************/

#ifndef CARMEN_MAP_EDITOR_GRAPHICS_H
#define CARMEN_MAP_EDITOR_GRAPHICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "global_graphics.h"

/* conversions from coordinates on the map to coordinates on the pixmap */
/* converts an x position on the drawing area to an x grid
 on the map */
extern double pix_x_to_map(double pix_x);
/* converts a y position on the drawing area to a y grid
 on the map */
extern double pix_y_to_map(double pix_y);
/* converts an x grid to an x position on the drawing area */
extern double map_x_to_pix(int map_x);
/* converts a y grid to a y position on the drawing area */
extern double map_y_to_pix(int map_y);
/* converts an x grid to a position on the map_pixmap */
extern double map_x_to_map_pix(int map_x);
/* converts an x grid to a position on the map_pixmap */
extern double map_y_to_map_pix(int map_y);

void draw_offlimits(int i);
/* creates tmp_pixmap from the map */
void map_to_tmp(void);
/* redraw the pixmap */
void redraw(void);
/* start the main window */
void start_drawing_window(int *argc, char **argv[]);
/* library initialization functions */

void set_up_map_widgets(void);

#ifdef __cplusplus
}
#endif

#endif
