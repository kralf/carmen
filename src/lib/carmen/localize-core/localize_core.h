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

/** @addtogroup localize liblocalizecore **/
// @{

/** 
 * \file localizecore.h 
 * \brief Library for Monto-Carlo localization.
 *
 * This library contains all functions to perform a MCL. 
 **/


#ifndef CARMEN_LOCALIZECORE_H
#define CARMEN_LOCALIZECORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "localize_motion.h"

#include "localize_messages.h"
#include "robot_messages.h"

#define      SMALL_PROB        0.01
#define      MAX_BEAMS_PER_SCAN   1024

/* #define      LOCALIZECORE_TRACKING_MINLIKELIHOOD        (0.5) */
/* #define      LOCALIZECORE_GLOBAL_MINLIKELIHOOD          (0.9) */

  /** localize parameter structure **/
typedef struct {
  double front_laser_offset, rear_laser_offset;
  int num_particles;
  double max_range, min_wall_prob, outlier_fraction;
  double update_distance;
  double integrate_angle;             /**< used to compute laser_skip **/
  int laser_skip;
  int use_rear_laser, do_scanmatching;
  int constrain_to_map;
#ifdef OLD_MOTION_MODEL
  double odom_a1, odom_a2, odom_a3, odom_a4;
#endif
  double occupied_prob;
  double lmap_std;
  double global_lmap_std, global_evidence_weight, global_distance_threshold;
  int global_test_samples;
  int use_sensor;

  double tracking_beam_minlikelihood;
  double global_beam_minlikelihood;

#ifndef OLD_MOTION_MODEL
  carmen_localize_motion_model_t *motion_model;
#endif  
} carmen_localize_param_t, *carmen_localize_param_p;

typedef struct {
  float x, y, theta, weight;
} carmen_localize_particle_t, *carmen_localize_particle_p;

typedef struct {
  int initialized, first_odometry, global_mode;
  carmen_localize_param_p param;
  carmen_localize_particle_p particles;
  carmen_point_t last_odometry_position;
  float **temp_weights;
  float distance_travelled;
  char laser_mask[MAX_BEAMS_PER_SCAN];
} carmen_localize_particle_filter_t, *carmen_localize_particle_filter_p;

typedef struct {
  float x, y, range;
  float prob;
  char mask;
} carmen_localize_laser_point_t, *carmen_localize_laser_point_p;

typedef struct {
  carmen_point_t mean, std;
  carmen_point_t odometry_pos;
  double xy_cov;
  int converged;
  int num_readings;
  carmen_localize_laser_point_t mean_scan[MAX_BEAMS_PER_SCAN];
} carmen_localize_summary_t, *carmen_localize_summary_p;

#include "likelihood_map.h"


/** Create (allocate memory for) a new particle filter **/
carmen_localize_particle_filter_p 
carmen_localize_particle_filter_new(carmen_localize_param_p param);

/** Creates a distribution of particles over the map based on the given observation 
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param laser Laser message used to generate the distribution.
 *  @param map Map which is used to compute p(z|m,x) for the initialized samples.
**/
void 
carmen_localize_initialize_particles_uniform(carmen_localize_particle_filter_p filter,
					     carmen_robot_laser_message *laser,
					     carmen_localize_map_p map);
  
/** Creates a multi Gaussian distribution of particles 
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param num_modes Number of modes if Gaussian to create the initial distrubution.
 *  @param mean Array of means (array size = num_modes)
 *  @param std Array of standard variances (array size = num_modes)
**/
void 
carmen_localize_initialize_particles_gaussians(carmen_localize_particle_filter_p filter,
					       int num_modes,
					       carmen_point_t *mean,
					       carmen_point_t *std);

/** Creates a Gaussian distribution of particles 
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param mean mean of the Gaussian
 *  @param std std var of the Gaussian
**/
void 
carmen_localize_initialize_particles_gaussian(carmen_localize_particle_filter_p filter,
					      carmen_point_t mean, 
					      carmen_point_t std);


/** Directly initialized the Samples
 *
 *  @param filter Particle filter structure the function is applied to.
**/
void 
carmen_localize_initialize_particles_manual(carmen_localize_particle_filter_p filter,
					    double *x, 
					    double *y, 
					    double *theta,
					    double *weight, 
					    int num_particles);

int 
carmen_localize_initialize_particles_placename(carmen_localize_particle_filter_p filter,
					       carmen_map_placelist_p placelist,
					       char *placename);

/** Draw the pose of the samples based on the proposal given by the motion model
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param odometry_position Odometry-based pose estimate.
**/
void 
carmen_localize_incorporate_odometry(carmen_localize_particle_filter_p filter,
				     carmen_point_t odometry_position);

/** Compute the particle weights according to the observation likelihood p(z|m,x)
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param map Map of the environment.
 *  @param num_readings Number of beams of that measurement.
 *  @param range The measured distances.
 *  @param forward_offset Offset of the laser in x direction.
 *  @param angular_resolution The angle between to beams.
 *  @param laser_maxrange The angle between to beams.
 *  @param first_beam_angle Angle of the first beam of a laser (often -0.5*PI)
 *  @param backwards Is it the rearlaser (=1) or the frontlaser(=0)
**/
void 
carmen_localize_incorporate_laser(carmen_localize_particle_filter_p filter,
				  carmen_localize_map_p map, 
				  int num_readings, 
				  float *range, 
				  double forward_offset, 
				  double angular_resolution,
				  double laser_maxrange,
				  double first_beam_angle,
				  int backwards);


/** Carries out the resampling step.
 *
 *  @param filter Particle filter structure the function is applied to.
 **/
void 
carmen_localize_resample(carmen_localize_particle_filter_p filter);

/** Carry out the three steps of the paricle filter which are:
 *  1) Draw from the motion model. 2) Compute the importance weights. 3) Resample.
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param map Map of the environment.
 *  @param laser A carmen_robot_laser_message which incorporates odoemtry, laser, and its configuration
 *  @param forward_offset Offset of the laser in x direction.
 *  @param backwards Is it the rearlaser (=1) or the frontlaser(=0)
 **/
void 
carmen_localize_run(carmen_localize_particle_filter_p filter, 
		    carmen_localize_map_p map,
		    carmen_robot_laser_message *laser, 
		    double forward_offset,
		    int backwards);

void 
carmen_localize_laser_scan_gd(int num_readings, 
			      float *range,
			      double angular_resolution,
			      double first_beam_angle,
			      carmen_point_p laser_pos, 
			      double forward_offset,
			      carmen_localize_map_p map, 
			      int laser_skip);

/** Carry a summary for sending it via ipc to other modules liek the gui **/
void 
carmen_localize_summarize(carmen_localize_particle_filter_p filter, 
			  carmen_localize_summary_p summary, 
			  carmen_localize_map_p map,
			  int num_readings, 
			  float *range, 
			  double forward_offset,
			  double angular_resolution,
			  double first_beam_angle,
			  int backwards);

#ifdef __cplusplus
}
#endif

#endif
// @}
