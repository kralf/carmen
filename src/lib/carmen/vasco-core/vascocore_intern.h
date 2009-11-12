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

#include <stdio.h>

#ifndef VASCOCORE_H
#define VASCOCORE_H

#define EPSILON                          0.00000001
#define MAX_NUM_LASER_VALUES           401

#define UPDT_NOT                         0
#define UPDT_X                           1
#define UPDT_Y                           2

typedef struct carmen_vascocore_quad_tree_t {
  
  struct carmen_vascocore_quad_tree_t  * elem[4];
  carmen_svec2_t                         center;
  unsigned char                          level;
  unsigned char                          inuse;

} carmen_vascocore_quad_tree_t;

typedef struct {
  
  carmen_vascocore_quad_tree_t           qtree;
  carmen_point_t                         offset;
  double                                 resolution;
  unsigned char                       ** updated;
  float                               ** maphit;
  short                               ** mapsum;
  float                               ** mapprob;
  float                               ** calc;
  carmen_ivec2_t                         mapsize;
  carmen_vec2_t                          center;

} carmen_vascocore_map_t, *carmen_vascocore_map_p;

typedef struct {

  double                                 time;
  carmen_point_t                         estpos;
  double                                 fov;
  int                                    numvalues;
  double                               * val;
  double                               * angle;
  carmen_vec2_t                        * coord;
  carmen_bbox_t                          bbox;

} carmen_vascocore_extd_laser_t;

typedef struct {
  
  int                                    started;
  int                                    length;
  int                                    ptr;
  carmen_vascocore_extd_laser_t        * data;
  
} carmen_vascocore_history_t;

extern carmen_vascocore_param_t      carmen_vascocore_settings;
extern carmen_vascocore_map_t        carmen_vascocore_map;
extern carmen_vascocore_history_t    carmen_vascocore_history;

void   initialize_map( carmen_vascocore_map_t * map,
		       int sx, int sy, int center_x, int center_y,
		       double resolution, carmen_point_t start );

void   clear_local_treemap( carmen_vascocore_quad_tree_t *tree,
			    carmen_vascocore_map_t *map, int hk );

void   create_local_treemap( carmen_vascocore_map_t * map,
			     carmen_vascocore_extd_laser_t data,
			     carmen_move_t movement );

void   convolve_treemap( carmen_vascocore_map_t *map );

int    intersect_bboxes( carmen_bbox_t box1, carmen_bbox_t box2 );

carmen_move_t
fit_data_in_local_map( carmen_vascocore_map_t map,
		       carmen_vascocore_extd_laser_t data,
		       carmen_move_t movement );

void   vascocore_compute_bbox( carmen_vascocore_extd_laser_t *data );

carmen_vec2_t
vascocore_compute_laser2d_coord( carmen_vascocore_extd_laser_t data, int i );

int     compute_map_pos_from_vec2( carmen_vec2_t pos,
				   carmen_vascocore_map_t map,
				   carmen_ivec2_t *v );

void           vascocore_graphics_map( void );

void           vascocore_graphics_update( void );

void           vascocore_graphics_init( int argc, char *argv[] );

void           vascocore_graphics_scan( carmen_vascocore_extd_laser_t scan );

#endif
