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

/** @addtogroup vasco libegrid **/
// @{

/** 
 * \file egrid.h 
 * \brief Library for the vasco grid structure.
 *
 * ...
 **/


#ifndef EGRID_H
#define EGRID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double resolution;
  int size_x, size_y;
  double theta_offset;
  double prior_occ;
  double occ_evidence, emp_evidence, max_prob;
  double max_sure_range, max_range;
  double wall_thickness;

  float **prob;
  int distance_table_size;
  double **distance_table;

  int first;
  double start_x, start_y, start_theta;
} evidence_grid;

int carmen_mapper_initialize_evidence_grid(evidence_grid *grid, 
					 int size_x, int size_y, 
					 double resolution, double theta_offset,
					 double prior_occ, double occ_evidence,
					 double emp_evidence, double max_prob, 
					 double max_sure_range, double max_range,
					 double wall_thickness);

void carmen_mapper_free_evidence_grid(evidence_grid *grid);

void carmen_mapper_update_evidence_grid(evidence_grid *grid, 
					double laser_x, double laser_y,
					double laser_theta, int num_readings,
					float *laser_range,
					double angular_resolution,
					double first_beam_angle);

void carmen_mapper_update_evidence_grid_general(evidence_grid *grid, 
						double laser_x, double laser_y,
						double laser_theta, int num_readings,
						float *laser_range,
						float *laser_angle,
						double angular_resolution,
						double first_beam_angle);

void carmen_mapper_clear_evidence_grid(evidence_grid *grid);

void carmen_mapper_finish_evidence_grid(evidence_grid *grid, int downsample, 
				      int border);
#ifdef __cplusplus
}
#endif

#endif

// @}
