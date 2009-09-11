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

#ifndef CARMEN_LIKELIHOOD_MAP_H
#define CARMEN_LIKELIHOOD_MAP_H

#include "localize_core.h"

#include "map.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  carmen_map_config_t config;
  carmen_map_t carmen_map;
  short int *complete_x_offset, *complete_y_offset;
  float *complete_distance, *complete_prob, *complete_gprob;
  short int **x_offset, **y_offset;
  float **distance, **prob, **gprob;
} carmen_localize_map_t, *carmen_localize_map_p;

void carmen_to_localize_map(carmen_map_p cmap, carmen_localize_map_p lmap,
			    carmen_localize_param_p param);

void carmen_localize_write_map_to_ppm(char *filename, 
				      carmen_localize_map_p map);

void carmen_localize_write_distance_map_to_ppm(char *filename, 
					       carmen_localize_map_p map);

void carmen_localize_write_likelihood_map_to_ppm(char *filename, 
						 carmen_localize_map_p map,
						 float **prob);

#ifdef __cplusplus
}
#endif

#endif
