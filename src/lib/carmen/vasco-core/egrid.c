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

#include <carmen/carmen.h>

#include "egrid.h"

#define LASER_RANGE_LIMIT 50.0

int carmen_mapper_initialize_evidence_grid(evidence_grid *grid, 
					 int size_x, int size_y, 
					 double resolution, double theta_offset,
					 double prior_occ, double occ_evidence,
					 double emp_evidence, double max_prob, 
					 double max_sure_range, double max_range,
					 double wall_thickness)
{
  int x, y;

  grid->size_x = size_x;
  grid->size_y = size_y;
  grid->resolution = resolution;
  grid->theta_offset = theta_offset;
  grid->prior_occ = prior_occ;
  grid->occ_evidence = occ_evidence;
  grid->emp_evidence = emp_evidence;
  grid->max_prob = max_prob;
  grid->max_sure_range = max_sure_range;
  grid->max_range = max_range;
  grid->wall_thickness = wall_thickness;

  grid->prob = (float **)calloc(grid->size_x, sizeof(float *));
  carmen_test_alloc(grid->prob);

  if(grid->prob == NULL) {
    fprintf(stderr, "Error: Could not allocate memory for evidence grid.\n");
    return -1;
  }
  for(x = 0; x < grid->size_x; x++) {
    grid->prob[x] = (float *)calloc(grid->size_y, sizeof(float));
    carmen_test_alloc(grid->prob[x]);

    if(grid->prob[x] == NULL) {
      fprintf(stderr, "Error: Could not allocate memory for evidence grid.\n");
      return -1;
    }
    for(y = 0; y < grid->size_y; y++)
      grid->prob[x][y] = -1;
  }

  grid->distance_table_size = (int)(grid->max_range / resolution);
  grid->distance_table = (double **)calloc(grid->distance_table_size, sizeof(double *));
  carmen_test_alloc(grid->distance_table);

  if(grid->distance_table == NULL) {
    fprintf(stderr, "Error: Could not allocate memory for evidence grid.\n");
    return -1;
  }
  for(x = 0; x < grid->distance_table_size; x++) {
    grid->distance_table[x] = (double *)calloc(grid->distance_table_size, sizeof(double));
    carmen_test_alloc(grid->distance_table[x]);

    if(grid->distance_table[x] == NULL) {
      fprintf(stderr, "Error: Could not allocate memory for evidence grid.\n");
      return -1;
    }
  }
  for(x = 0; x < grid->distance_table_size; x++)
    for(y = 0; y < grid->distance_table_size; y++)
      grid->distance_table[x][y] = sqrt(x * x + y * y) * resolution;
  grid->first = 1;
  return 0;
}

void carmen_mapper_free_evidence_grid(evidence_grid *grid)
{
  int x;
  
  for(x = 0; x < grid->distance_table_size; x++)
    free(grid->distance_table[x]);
  free(grid->distance_table);
  for(x = 0; x < grid->size_x; x++)
    free(grid->prob[x]);
  free(grid->prob);
}

void carmen_mapper_update_evidence_grid(evidence_grid *grid, 
					double laser_x, double laser_y,
					double laser_theta, int num_readings,
					float *laser_range,
					double angular_resolution,
					double first_beam_angle)
{
  int i;
  int x1int, y1int, x2int, y2int, current_x, current_y, x_diff, y_diff;
  double d, theta, delta_theta, p_filled, temp_laser_x, temp_laser_y;
  carmen_bresenham_param_t b_params;
  double new_prob;
  double correction_angle = 0;
  
  if(grid->first) {
    grid->start_x = laser_x / grid->resolution;
    grid->start_y = laser_y / grid->resolution;
    grid->start_theta = laser_theta;
    grid->first = 0;
  }

  correction_angle = grid->theta_offset - grid->start_theta;

  laser_theta = laser_theta + correction_angle;
  temp_laser_x = laser_x * cos(correction_angle) -
    laser_y * sin(correction_angle);
  temp_laser_y = laser_x * sin(correction_angle) +
    laser_y * cos(correction_angle);

  laser_x = grid->size_x / 2.0 + (temp_laser_x / grid->resolution - 
				  grid->start_x);
  laser_y = grid->size_y / 2.0 + (temp_laser_y / grid->resolution - 
				  grid->start_y);

  x1int = (int)floor(laser_x);
  y1int = (int)floor(laser_y);

  theta = laser_theta + first_beam_angle;
  delta_theta = angular_resolution;

  for(i = 0; i < num_readings; i++) {
    if(laser_range[i] < LASER_RANGE_LIMIT) {
      x2int = (int)(laser_x + (laser_range[i] + grid->wall_thickness) * 
		    cos(theta) / grid->resolution);
      y2int = (int)(laser_y + (laser_range[i] + grid->wall_thickness) * 
		    sin(theta) / grid->resolution);
      carmen_get_bresenham_parameters(x1int, y1int, x2int, y2int, &b_params);
      do {
	carmen_get_current_point(&b_params, &current_x, &current_y);
	if(current_x >= 0 && current_x < grid->size_x &&
	   current_y >= 0 && current_y < grid->size_y) {
	  /* Calculate distance from laser */
	  x_diff = abs(current_x - x1int);
	  y_diff = abs(current_y - y1int);
	  if(x_diff >= grid->distance_table_size || 
	     y_diff >= grid->distance_table_size)
	    d = 1e6;
	  else
	    d = grid->distance_table[x_diff][y_diff];
	  
	  if(d < laser_range[i]) {       /* Free observation */
	    if(d < grid->max_sure_range)
	      p_filled = grid->emp_evidence;
	    else if(d < grid->max_range)
	      p_filled = grid->emp_evidence +
		(d - grid->max_sure_range) / grid->max_range * 
		(grid->prior_occ - grid->emp_evidence);
	    else
	      break;
	  }
	  else {                               /* Filled observation */
	    if(d < grid->max_sure_range)
	      p_filled = grid->occ_evidence;
	    else if(d < grid->max_range)
	      p_filled = grid->occ_evidence + 
		(d - grid->max_sure_range) / grid->max_range * 
		(grid->prior_occ - grid->occ_evidence);
	    else
	      break;
	  }
	  
	  if(grid->prob[current_x][current_y] == -1)
	    grid->prob[current_x][current_y] = grid->prior_occ;

	  /* Adjust the map */
	  new_prob = 1 - 1 / (1 + (1 - grid->prior_occ) / grid->prior_occ * 
			      p_filled / (1 - p_filled) *
			      grid->prob[current_x][current_y] / 
			      (1 - grid->prob[current_x][current_y]));

	  grid->prob[current_x][current_y] = new_prob;
	}
      } while(carmen_get_next_point(&b_params));
    }
    theta += delta_theta;
  }
}

void carmen_mapper_update_evidence_grid_general(evidence_grid *grid, 
						double laser_x, double laser_y,
						double laser_theta, int num_readings,
						float *laser_range,
						float *laser_angle,
						double angular_resolution,
						double first_beam_angle)
{
  int i;
  int x1int, y1int, x2int, y2int, current_x, current_y, x_diff, y_diff;
  double d, theta, delta_theta, p_filled, temp_laser_x, temp_laser_y;
  carmen_bresenham_param_t b_params;
  double new_prob;
  double correction_angle = 0;
  
  if(grid->first) {
    grid->start_x = laser_x / grid->resolution;
    grid->start_y = laser_y / grid->resolution;
    grid->start_theta = laser_theta;
    grid->first = 0;
  }

  correction_angle = grid->theta_offset - grid->start_theta;

  laser_theta = laser_theta + correction_angle;
  temp_laser_x = laser_x * cos(correction_angle) -
    laser_y * sin(correction_angle);
  temp_laser_y = laser_x * sin(correction_angle) +
    laser_y * cos(correction_angle);

  laser_x = grid->size_x / 2.0 + (temp_laser_x / grid->resolution - 
				  grid->start_x);
  laser_y = grid->size_y / 2.0 + (temp_laser_y / grid->resolution - 
				  grid->start_y);

  x1int = (int)floor(laser_x);
  y1int = (int)floor(laser_y);

  theta = laser_theta + first_beam_angle;
  delta_theta = angular_resolution;

  for(i = 0; i < num_readings; i++) {
    if(laser_range[i] < LASER_RANGE_LIMIT) {
      theta = laser_theta + laser_angle[i];
      x2int = (int)(laser_x + (laser_range[i] + grid->wall_thickness) * 
		    cos(theta) / grid->resolution);
      y2int = (int)(laser_y + (laser_range[i] + grid->wall_thickness) * 
		    sin(theta) / grid->resolution);
      carmen_get_bresenham_parameters(x1int, y1int, x2int, y2int, &b_params);
      do {
	carmen_get_current_point(&b_params, &current_x, &current_y);
	if(current_x >= 0 && current_x < grid->size_x &&
	   current_y >= 0 && current_y < grid->size_y) {
	  /* Calculate distance from laser */
	  x_diff = abs(current_x - x1int);
	  y_diff = abs(current_y - y1int);
	  if(x_diff >= grid->distance_table_size || 
	     y_diff >= grid->distance_table_size)
	    d = 1e6;
	  else
	    d = grid->distance_table[x_diff][y_diff];
	  
	  if(d < laser_range[i]) {       /* Free observation */
	    if(d < grid->max_sure_range)
	      p_filled = grid->emp_evidence;
	    else if(d < grid->max_range)
	      p_filled = grid->emp_evidence +
		(d - grid->max_sure_range) / grid->max_range * 
		(grid->prior_occ - grid->emp_evidence);
	    else
	      break;
	  }
	  else {                               /* Filled observation */
	    if(d < grid->max_sure_range)
	      p_filled = grid->occ_evidence;
	    else if(d < grid->max_range)
	      p_filled = grid->occ_evidence + 
		(d - grid->max_sure_range) / grid->max_range * 
		(grid->prior_occ - grid->occ_evidence);
	    else
	      break;
	  }
	  
	  if(grid->prob[current_x][current_y] == -1)
	    grid->prob[current_x][current_y] = grid->prior_occ;

	  /* Adjust the map */
	  new_prob = 1 - 1 / (1 + (1 - grid->prior_occ) / grid->prior_occ * 
			      p_filled / (1 - p_filled) *
			      grid->prob[current_x][current_y] / 
			      (1 - grid->prob[current_x][current_y]));

	  grid->prob[current_x][current_y] = new_prob;
	}
      } while(carmen_get_next_point(&b_params));
    }
  }
}

void carmen_mapper_clear_evidence_grid(evidence_grid *grid)
{
  int x, y;
  
  for(x = 0; x < grid->size_x; x++)
    for(y = 0; y < grid->size_y; y++)
      grid->prob[x][y] = grid->prior_occ;
}

void carmen_mapper_finish_evidence_grid(evidence_grid *grid, int downsample, 
					int border)
{
  int min_x = 1e6, min_y = 1e6, max_x = 0, max_y = 0;
  int x, y, i, j, count, size_x, size_y;
  double sum;
  float **prob2;

  size_x = grid->size_x / downsample;
  size_y = grid->size_y / downsample;
  prob2 = (float **)calloc(size_x, sizeof(float *));
  carmen_test_alloc(prob2);

  for(x = 0; x < size_x; x++)
    {
      prob2[x] = (float *)calloc(size_y, sizeof(float));
      carmen_test_alloc(prob2[x]);
    }

  for(x = 0; x < size_x; x++)
    for(y = 0; y < size_y; y++) {
      count = 0;
      sum = 0;
      for(i = x * downsample; i < (x + 1) * downsample; i++)
	for(j = y * downsample; j < (y + 1) * downsample; j++)
	  if(i < grid->size_x && j < grid->size_y && grid->prob[i][j] != -1) {
	    count++;
	    sum += grid->prob[i][j];
	  }
      if(count == 0)
	prob2[x][y] = -1;
      else
	prob2[x][y] = sum / (double)count;
    }

  for(x = 0; x < grid->size_x; x++)
    free(grid->prob[x]);
  free(grid->prob);

  min_x = size_x - border - 1;
  min_y = size_y - border - 1;
  max_x = border;
  max_y = border;

  for(x = 0; x < size_x; x++)
    for(y = 0; y < size_y; y++)
      if(prob2[x][y] != -1) {
	if(x < min_x)
	  min_x = x;
	if(x > max_x)
	  max_x = x;
	if(y < min_y)
	  min_y = y;
	if(y > max_y)
	  max_y = y;
      }
  min_x -= border; if(min_x < 0) min_x = 0;
  min_y -= border; if(min_y < 0) min_y = 0;
  max_x += border; if(max_x >= size_x) max_x = size_x - 1;
  max_y += border; if(max_y >= size_y) max_y = size_y - 1;
  
  grid->size_x = max_x - min_x;
  grid->size_y = max_y - min_y;
  grid->resolution *= downsample;

  grid->prob = (float **)calloc(grid->size_x, sizeof(float *));
  carmen_test_alloc(grid->prob);

  for(x = 0; x < grid->size_x; x++)
    {
      grid->prob[x] = (float *)calloc(grid->size_y, sizeof(float));
      carmen_test_alloc(grid->prob[x]);
    }
  
  for(x = 0; x < grid->size_x; x++)
    for(y = 0; y < grid->size_y; y++)
      grid->prob[x][y] = prob2[x + min_x][y + min_y];

  for(x = 0; x < size_x; x++)
    free(prob2[x]);
  free(prob2);
}

