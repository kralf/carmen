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
#include "localize_core.h"
#include "likelihood_map.h"

/* gains for gradient descent */

#define K_T   0.0001
#define K_ROT 0.00001

/* priority queue for sorting global localization hypotheses */

typedef struct queue_node {
  carmen_point_t point;
  float prob;
  struct queue_node *next, *prev;
} queue_node_t, *queue_node_p;

typedef struct {
  int num_elements, max_elements;
  queue_node_p first, last;
} priority_queue_t, *priority_queue_p;

/* initialize a new priority queue */

static priority_queue_p priority_queue_init(int max_elements)
{
  priority_queue_p result;

  result = (priority_queue_p)calloc(1, sizeof(priority_queue_t));
  carmen_test_alloc(result);
  result->num_elements = 0;
  result->max_elements = max_elements;
  result->first = NULL;
  result->last = NULL;
  return result;
}

/* add a point to the priority queue */

static void priority_queue_add(priority_queue_p queue, carmen_point_t point,
			       float prob)
{
  queue_node_p mark, temp;

  if(queue->num_elements == 0) {
    temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
    carmen_test_alloc(temp);
    temp->point = point;
    temp->prob = prob;
    temp->prev = NULL;
    temp->next = NULL;
    queue->first = temp;
    queue->last = temp;
    queue->num_elements++;
  }
  else if(prob > queue->last->prob || 
	  queue->num_elements < queue->max_elements) {
    mark = queue->last;
    while(mark != NULL && prob > mark->prob)
      mark = mark->prev;
    if(mark == NULL) {
      temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
      carmen_test_alloc(temp);
      temp->point = point;
      temp->prob = prob;
      temp->prev = NULL;
      temp->next = queue->first;
      queue->first->prev = temp;
      queue->first = temp;
      queue->num_elements++;
    }
    else {
      temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
      carmen_test_alloc(temp);
      temp->point = point;
      temp->prob = prob;
      temp->prev = mark;
      temp->next = mark->next;
      if(mark->next != NULL)
	mark->next->prev = temp;
      else
	queue->last = temp;
      mark->next = temp;
      queue->num_elements++;
    }
    if(queue->num_elements > queue->max_elements) {
      queue->last = queue->last->prev;
      free(queue->last->next);
      queue->last->next = NULL;
      queue->num_elements--;
    }
  }
}

/* free the priority queue */

static void priority_queue_free(priority_queue_p queue)
{
  queue_node_p mark;

  while(queue->first != NULL) {
    mark = queue->first;
    queue->first = queue->first->next;
    free(mark);
  }
  free(queue);
}

/* initialize memory necessary for holding temporary sensor weights */

static void initialize_temp_weights(carmen_localize_particle_filter_p filter)
{
  int i;
  
  filter->temp_weights = (float **)calloc(filter->param->num_particles,
					  sizeof(float *));
  carmen_test_alloc(filter->temp_weights);
  for(i = 0; i < filter->param->num_particles; i++) {
    filter->temp_weights[i] = (float *)calloc(MAX_BEAMS_PER_SCAN, sizeof(float));
    carmen_test_alloc(filter->temp_weights[i]);
  }
}

/* resize memory for temporary sensor weights */

static void realloc_temp_weights(carmen_localize_particle_filter_p filter, 
				 int num_particles)
{
  int i;

  for(i = 0; i < filter->param->num_particles; i++)
    free(filter->temp_weights[i]);
  filter->temp_weights = (float **)realloc(filter->temp_weights, 
					   num_particles * sizeof(float *));
  carmen_test_alloc(filter->temp_weights);
  for(i = 0; i < num_particles; i++) {
    filter->temp_weights[i] = (float *)calloc(MAX_BEAMS_PER_SCAN, sizeof(float));
    carmen_test_alloc(filter->temp_weights[i]);
  }
}

/* allocate memory for a new particle filter */

carmen_localize_particle_filter_p 
carmen_localize_particle_filter_new(carmen_localize_param_p param)
{
  carmen_localize_particle_filter_p filter;

  /* allocate the particle filter */
  filter = 
    (carmen_localize_particle_filter_p)
    calloc(1, sizeof(carmen_localize_particle_filter_t));
  carmen_test_alloc(filter);

  /* set the parameters */
  filter->param = param;

  /* allocate the initial particle set */
  filter->particles = 
    (carmen_localize_particle_p)calloc(filter->param->num_particles,
				       sizeof(carmen_localize_particle_t));
  carmen_test_alloc(filter->particles);

  /* initialize the temporary weights */
  initialize_temp_weights(filter);
  
  /* filter has not been initialized */
  filter->initialized = 0;
  filter->first_odometry = 1;
  filter->global_mode = 0;
  filter->distance_travelled = 0;

  filter->param->laser_skip = 0; /* will be automatically initialized later on */

  return filter;
}

void carmen_localize_initialize_particles_uniform(carmen_localize_particle_filter_p filter,
						  carmen_robot_laser_message *laser,
						  carmen_localize_map_p map)
{
  priority_queue_p queue = priority_queue_init(filter->param->num_particles);
  float *laser_x, *laser_y;
  int i, j, x_l, y_l;
  float angle, prob, ctheta, stheta;
  carmen_point_t point;
  queue_node_p mark;
  int *beam_valid;


  /* compute the correct laser_skip */
  if (filter->param->laser_skip <= 0) {   
    filter->param->laser_skip = 
      floor(filter->param->integrate_angle / laser->config.angular_resolution);
  }
  
  fprintf(stderr, "\rDoing global localization... (%.1f%% complete)", 0.0);
  filter->initialized = 0;
  /* copy laser scan into temporary memory */
  laser_x = (float *)calloc(laser->num_readings, sizeof(float));
  carmen_test_alloc(laser_x);
  laser_y = (float *)calloc(laser->num_readings, sizeof(float));
  carmen_test_alloc(laser_y);
  beam_valid = (int *)calloc(laser->num_readings, sizeof(int));
  carmen_test_alloc(beam_valid);
  
  for(i = 0; i < laser->num_readings; i++) {
    if (laser->range[i] < laser->config.maximum_range &&
	laser->range[i] < filter->param->max_range) 
      beam_valid[i] = 1;
    else
      beam_valid[i] = 0;
  }
  
  /* do all calculations in map coordinates */
  for(i = 0; i < laser->num_readings; i++) {
    angle = laser->config.start_angle + 
      i * laser->config.angular_resolution;

    laser_x[i] = (filter->param->front_laser_offset + 
		  laser->range[i] * cos(angle)) / map->config.resolution;
    laser_y[i] = (laser->range[i] * sin(angle)) / map->config.resolution;
  }

  for(i = 0; i < filter->param->global_test_samples; i++) {
    if(i % 10000 == 0) {
      fprintf(stderr, "\rDoing global localization... (%.1f%% complete)", 
	      i / (float)filter->param->global_test_samples * 100.0);
      carmen_ipc_sleep(0.001);
    }
    do {
      point.x = carmen_uniform_random(0, map->config.x_size - 1);
      point.y = carmen_uniform_random(0, map->config.y_size - 1);
    } while(map->carmen_map.map[(int)point.x][(int)point.y] > 
	    filter->param->occupied_prob ||
	    map->carmen_map.map[(int)point.x][(int)point.y] == -1);
    point.theta = carmen_uniform_random(-M_PI, M_PI);
  
    prob = 0.0;
    ctheta = cos(point.theta);
    stheta = sin(point.theta);
    for(j = 0; j < laser->num_readings && 
	  (queue->last == NULL || prob > queue->last->prob);
	j += filter->param->laser_skip) {

	if (beam_valid[j]) {
	    x_l = point.x + laser_x[j] * ctheta - laser_y[j] * stheta;
	    y_l = point.y + laser_x[j] * stheta + laser_y[j] * ctheta;
	
	    if(x_l >= 0 && y_l >= 0 && x_l < map->config.x_size &&
	       y_l < map->config.y_size)
		prob += map->gprob[x_l][y_l];
	    else
		prob -= 100;
	}
    }
    priority_queue_add(queue, point, prob);
  }

  /* transfer samples from priority queue back into particles */
  mark = queue->first;
  for(i = 0; i < queue->num_elements; i++) {
    filter->particles[i].x = mark->point.x * map->config.resolution;
    filter->particles[i].y = mark->point.y * map->config.resolution;
    filter->particles[i].theta = mark->point.theta;
    mark = mark->next;
  }
  priority_queue_free(queue);
  free(laser_x);
  free(laser_y);
  free(beam_valid);


  if(filter->param->do_scanmatching) {
    for(i = 0; i < filter->param->num_particles; i++) {
      point.x = filter->particles[i].x;
      point.y = filter->particles[i].y;
      point.theta = filter->particles[i].theta;
      carmen_localize_laser_scan_gd(laser->num_readings, laser->range, 
				    laser->config.angular_resolution,
				    laser->config.start_angle,
				    &point, 
				    filter->param->front_laser_offset, 
				    map,
				    filter->param->laser_skip);
      filter->particles[i].x = point.x;
      filter->particles[i].y = point.y;
      filter->particles[i].theta = point.theta;
      filter->particles[i].weight = 0.0;
    }
  }
  filter->initialized = 1;
  filter->first_odometry = 1;
  filter->global_mode = 1;
  filter->distance_travelled = 0;
  fprintf(stderr, "\rDoing global localization... (%.1f%% complete)\n\n",
	  100.0);
}

/* initialize particles from a gaussian distribution */

void carmen_localize_initialize_particles_gaussians(carmen_localize_particle_filter_p filter,
						    int num_modes,
						    carmen_point_t *mean,
						    carmen_point_t *std)
{
  int i, j, each, start, end;
  float x, y, theta;

  each = (int)floor(filter->param->num_particles / (float)num_modes);
  for(i = 0; i < num_modes; i++) {
    start = i * each;
    if(i == num_modes - 1)
      end = filter->param->num_particles;
    else
      end = (i + 1) * each;

    for(j = start; j < end; j++) {
      x = carmen_gaussian_random(mean[i].x, std[i].x);
      y = carmen_gaussian_random(mean[i].y, std[i].y);
      theta = carmen_normalize_theta(carmen_gaussian_random(mean[i].theta, 
							    std[i].theta));
      filter->particles[j].x = x;
      filter->particles[j].y = y;
      filter->particles[j].theta = theta;
      filter->particles[j].weight = 0.0;
    }
  }
  filter->initialized = 1;
  filter->first_odometry = 1;
  if (num_modes < 2)
    filter->global_mode = 0;
  else
    filter->global_mode = 1;
  filter->distance_travelled = 0;
}

int carmen_localize_initialize_particles_placename(carmen_localize_particle_filter_p filter,
						   carmen_map_placelist_p placelist,
						   char *placename)
{
  carmen_point_t mean, std;

  int i;
  for(i = 0; i < placelist->num_places; i++)
    if(strcmp(placename, placelist->places[i].name) == 0)
      break;
/*   if(i == placelist->num_places ||  */
/*      placelist->places[i].type != CARMEN_LCALIZATION_INIT_TYPE) */
/*     return -1; */
  if(i == placelist->num_places/*  ||  */
/*      placelist->places[i].type != CARMEN_LCALIZATION_INIT_TYPE */)
    return -1;
  mean.x = placelist->places[i].x;
  mean.y = placelist->places[i].y;
  mean.theta = placelist->places[i].theta;
  std.x = placelist->places[i].x_std;
  std.y = placelist->places[i].y_std;
  std.theta = placelist->places[i].theta_std;
  carmen_localize_initialize_particles_gaussian(filter, mean, std);
  return 0;
}

/* initialize particles from a gaussian distribution */

void carmen_localize_initialize_particles_gaussian(carmen_localize_particle_filter_p filter,
					    carmen_point_t mean, 
					    carmen_point_t std)
{
  carmen_localize_initialize_particles_gaussians(filter, 1, &mean, &std);
}

/* initialize particle positions and weights from parameters */

void carmen_localize_initialize_particles_manual(carmen_localize_particle_filter_p filter,
					  double *x, double *y, double *theta,
					  double *weight, int num_particles)
{
  int i;
  
  if(num_particles != filter->param->num_particles) {
    filter->particles = 
      (carmen_localize_particle_p)realloc(filter->particles, num_particles * 
				   sizeof(carmen_localize_particle_t));
    carmen_test_alloc(filter->particles);
    realloc_temp_weights(filter, num_particles);
    filter->param->num_particles = num_particles;
  }
  for(i = 0; i < filter->param->num_particles; i++) {
    filter->particles[i].x = x[i];
    filter->particles[i].y = y[i];
    filter->particles[i].theta = theta[i];
    filter->particles[i].weight = weight[i];
  }
  filter->initialized = 1;
  filter->first_odometry = 1;
  filter->global_mode = 0;
  filter->distance_travelled = 0;
}

/* incorporate a single odometry reading into the particle filter */

void carmen_localize_incorporate_odometry(carmen_localize_particle_filter_p filter,
					  carmen_point_t odometry_position)
{
  int i, backwards;
  double delta_t, delta_theta;
  double dx, dy, odom_theta;
#ifndef OLD_MOTION_MODEL
  double downrange, crossrange, turn;
#else
  double dr1, dr2;
  double dhatr1, dhatt, dhatr2;
  double std_r1, std_r2, std_t;
#endif

  /* The dr1/dr2 code becomes unstable if dt is too small. */
  if(filter->first_odometry) {
    filter->last_odometry_position = odometry_position;
    filter->first_odometry = 0;
    return;
  }

  dx = odometry_position.x - filter->last_odometry_position.x;
  dy = odometry_position.y - filter->last_odometry_position.y;
  delta_t = sqrt(dx * dx + dy * dy);
  delta_theta = carmen_normalize_theta(odometry_position.theta - 
				       filter->last_odometry_position.theta);
  odom_theta = atan2(dy, dx);
  backwards = (dx * cos(odometry_position.theta) + 
	       dy * sin(odometry_position.theta) < 0);

  filter->distance_travelled += delta_t;

#ifndef OLD_MOTION_MODEL
  for(i = 0; i < filter->param->num_particles; i++) {
    downrange = 
      carmen_localize_sample_noisy_downrange(delta_t, delta_theta, 
					     filter->param->motion_model);
    crossrange = 
      carmen_localize_sample_noisy_crossrange(delta_t, delta_theta,
					      filter->param->motion_model);
    turn = carmen_localize_sample_noisy_turn(delta_t, delta_theta,
					     filter->param->motion_model);

    if(backwards) {
      filter->particles[i].x -= downrange * 
	cos(filter->particles[i].theta + turn/2.0) + 
	crossrange * cos(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
      filter->particles[i].y -= downrange * 
	sin(filter->particles[i].theta + turn/2.0) + 
	crossrange * sin(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
    } else {
      filter->particles[i].x += downrange * 
	cos(filter->particles[i].theta + turn/2.0) + 
      crossrange * cos(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
      filter->particles[i].y += downrange * 
	sin(filter->particles[i].theta + turn/2.0) + 
      crossrange * sin(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
    }
    filter->particles[i].theta = carmen_normalize_theta(filter->particles[i].theta+turn);
  }
#else
 /* The dr1/dr2 code becomes unstable if delta_t is too small. */
  if(delta_t < 0.05) {
    dr1 = carmen_normalize_theta(odometry_position.theta - 
				 filter->last_odometry_position.theta) / 2.0;
    dr2 = dr1;
  }
  else {
    if(backwards)
      dr1 = carmen_normalize_theta(atan2(filter->last_odometry_position.y -
					 odometry_position.y, 
					 filter->last_odometry_position.x -
					 odometry_position.x) -
                                   filter->last_odometry_position.theta);
    else
      dr1 = carmen_normalize_theta(atan2(odometry_position.y -
					 filter->last_odometry_position.y,
					 odometry_position.x - 
					 filter->last_odometry_position.x) -
                                   filter->last_odometry_position.theta);
    dr2 = carmen_normalize_theta(odometry_position.theta - 
				 filter->last_odometry_position.theta - dr1);
  }

  /* compute motion model parameters */
  std_r1 = filter->param->odom_a1 * fabs(dr1) + filter->param->odom_a2 * delta_t;
  std_t = filter->param->odom_a3 * delta_t + filter->param->odom_a4 * fabs(dr1 + dr2);
  std_r2 = filter->param->odom_a1 * fabs(dr2) + filter->param->odom_a2 * delta_t;

  /* update the positions of all of the particles */
  for(i = 0; i < filter->param->num_particles; i++) {
    dhatr1 = carmen_gaussian_random(dr1, std_r1);
    dhatt = carmen_gaussian_random(delta_t, std_t);
    dhatr2 = carmen_gaussian_random(dr2, std_r2);
    
    if(backwards) {
      filter->particles[i].x -=
        dhatt * cos(filter->particles[i].theta + dhatr1);
      filter->particles[i].y -=
        dhatt * sin(filter->particles[i].theta + dhatr1);
    }
    else {
      filter->particles[i].x +=
        dhatt * cos(filter->particles[i].theta + dhatr1);
      filter->particles[i].y +=
        dhatt * sin(filter->particles[i].theta + dhatr1);
    }
    filter->particles[i].theta =
      carmen_normalize_theta(filter->particles[i].theta + dhatr1 + dhatr2);
  }
#endif

  /* keep track of the last odometry */
  filter->last_odometry_position = odometry_position;
}

/* test to see if localize should switch to global mode */

static int global_mode_test(carmen_localize_particle_filter_p filter)
{
  int i;
  float mean_x = 0, mean_y = 0;

  for(i = 0; i < filter->param->num_particles; i++) {
    mean_x += filter->particles[i].x;
    mean_y += filter->particles[i].y;
  }
  mean_x /= filter->param->num_particles;
  mean_y /= filter->param->num_particles;
  
  for(i = 0; i < filter->param->num_particles; i++)
    if(fabs(filter->particles[i].x - mean_x) > 
       filter->param->global_distance_threshold ||
       fabs(filter->particles[i].y - mean_y) > 
       filter->param->global_distance_threshold)
      return 1;
  return 0;
}

/* incorporate a single laser scan into the paritcle filter */

void carmen_localize_incorporate_laser(carmen_localize_particle_filter_p filter,
				       carmen_localize_map_p map, int num_readings, 
				       float *range, double forward_offset, 
				       double angular_resolution,
				       double laser_maxrange,
				       double first_beam_angle,
				       int backwards)
{
  float angle, *laser_x, *laser_y, p_x, p_y, ctheta, stheta;

  float log_small_prob = log(filter->param->tracking_beam_minlikelihood);
  float global_log_small_prob = log(filter->param->global_beam_minlikelihood);
  float log_min_wall_prob = log(filter->param->tracking_beam_minlikelihood);

/*   float log_small_prob = log(SMALL_PROB); */
/*   float global_log_small_prob = log_small_prob *  filter->param->global_evidence_weight; */
/*   float log_min_wall_prob = log(filter->param->min_wall_prob); */

  int i, j, x, y, robot_x, robot_y;
  int count[num_readings]; 

  /* compute the correct laser_skip */
  if (filter->param->laser_skip <= 0) {   
    filter->param->laser_skip = 
      floor(filter->param->integrate_angle / angular_resolution);
  }

  /* reset the weights back to even */
  for(i = 0; i < filter->param->num_particles; i++)
    filter->particles[i].weight = 0.0;

  /* compute positions of laser points assuming robot pos is (0, 0, 0) */
  laser_x = (float *)calloc(num_readings, sizeof(float));
  carmen_test_alloc(laser_x);
  laser_y = (float *)calloc(num_readings, sizeof(float));
  carmen_test_alloc(laser_y);
  for(i = 0; i < num_readings; i++) {
    angle = first_beam_angle + i * angular_resolution;

    laser_x[i] = (forward_offset + range[i] * cos(angle)) / 
      map->config.resolution;
    laser_y[i] = (range[i] * sin(angle)) / map->config.resolution;
    if(backwards) {
      laser_x[i] = -laser_x[i];
      laser_y[i] = -laser_y[i];
    }
    if((i % filter->param->laser_skip) == 0 && 
       range[i] < filter->param->max_range && 
       range[i] < laser_maxrange)
      filter->laser_mask[i] = 1;
    else
      filter->laser_mask[i] = 0;
  }

  /* test for global mode */
  filter->global_mode = global_mode_test(filter);

  if(filter->global_mode)
    /* compute weight of each laser reading - using global map */
    for(i = 0; i < filter->param->num_particles; i++) {
      p_x = filter->particles[i].x / map->config.resolution;
      p_y = filter->particles[i].y / map->config.resolution;
      ctheta = cos(filter->particles[i].theta);
      stheta = sin(filter->particles[i].theta);
      for(j = 0; j < num_readings; j += filter->param->laser_skip)
	if(filter->laser_mask[j]) {
	  x = (p_x + laser_x[j] * ctheta - laser_y[j] * stheta);
	  y = (p_y + laser_x[j] * stheta + laser_y[j] * ctheta);
	  robot_x = p_x;
	  robot_y = p_y;
	  if(x < 0 || y < 0 || x >= map->config.x_size ||
	     y >= map->config.y_size || map->carmen_map.map[x][y] == -1)
	    filter->particles[i].weight += global_log_small_prob;
	  else if(filter->param->constrain_to_map &&
		  (robot_x < 0 || robot_y < 0 || 
		   robot_x >= map->config.x_size ||
		   robot_y >= map->config.y_size ||
		   map->carmen_map.map[robot_x][robot_y] >
		   filter->param->occupied_prob))
	    filter->particles[i].weight += global_log_small_prob;
	  else
	    filter->particles[i].weight += map->gprob[x][y];
/* 	  *  filter->param->global_evidence_weight; */
	}
    }
  else {
    /* compute weight of each laser reading */
    for(i = 0; i < filter->param->num_particles; i++) {
      p_x = filter->particles[i].x / map->config.resolution;
      p_y = filter->particles[i].y / map->config.resolution;
      ctheta = cos(filter->particles[i].theta);
      stheta = sin(filter->particles[i].theta);
      for(j = 0; j < num_readings; j += filter->param->laser_skip)
	if(filter->laser_mask[j]) {
	  x = (p_x + laser_x[j] * ctheta - laser_y[j] * stheta);
	  y = (p_y + laser_x[j] * stheta + laser_y[j] * ctheta);
	  robot_x = p_x;
	  robot_y = p_y;
	  if(x < 0 || y < 0 || x >= map->config.x_size ||
	     y >= map->config.y_size || map->carmen_map.map[x][y] == -1)
	    filter->temp_weights[i][j] = log_small_prob;
	  else if(filter->param->constrain_to_map &&
		  (robot_x < 0 || robot_y < 0 || 
		   robot_x >= map->config.x_size ||
		   robot_y >= map->config.y_size ||
		   map->carmen_map.map[robot_x][robot_y] >
		   filter->param->occupied_prob))
	    filter->temp_weights[i][j] = log_small_prob;
	  else
	    filter->temp_weights[i][j] = map->prob[x][y];
	}
    }

    /* ignore laser readings that are improbable in a large fraction
       of the particles */
    memset(count, 0, num_readings * sizeof(int));
    for(i = 0; i < filter->param->num_particles; i++)
      for(j = 0; j < num_readings; j += filter->param->laser_skip)
	if(filter->laser_mask[j] &&
	   filter->temp_weights[i][j] < log_min_wall_prob)
	  count[j]++;
    for(i = 0; i < num_readings; i++)
      if(filter->laser_mask[i] &&
	 count[i] / (float)filter->param->num_particles >
	 filter->param->outlier_fraction)
	filter->laser_mask[i] = 0;

    /* add log probabilities to particle weights */
    for(i = 0; i < filter->param->num_particles; i++)
      for(j = 0; j < num_readings; j += filter->param->laser_skip)
	if(filter->laser_mask[j])
	  filter->particles[i].weight += filter->temp_weights[i][j];
  }

  /* free laser points */
  free(laser_x);
  free(laser_y);
}

/* resample particle filter */

void carmen_localize_resample(carmen_localize_particle_filter_p filter)
{
  int i, which_particle;
  float weight_sum = 0.0, *cumulative_sum = NULL;
  float position, step_size, max_weight = filter->particles[0].weight;
  carmen_localize_particle_p temp_particles = NULL;

  /* change log weights back into probabilities */
  for(i = 0; i < filter->param->num_particles; i++)
    if(filter->particles[i].weight > max_weight)
      max_weight = filter->particles[i].weight;
  for(i = 0; i < filter->param->num_particles; i++)
    filter->particles[i].weight = 
      exp(filter->particles[i].weight - max_weight);

  /* Allocate memory necessary for resampling */
  cumulative_sum = (float *)calloc(filter->param->num_particles, sizeof(float));
  carmen_test_alloc(cumulative_sum);
  temp_particles = (carmen_localize_particle_p)
    calloc(filter->param->num_particles, sizeof(carmen_localize_particle_t));
  carmen_test_alloc(temp_particles);

  /* Sum the weights of all of the particles */
  for(i = 0; i < filter->param->num_particles; i++) {
    weight_sum += filter->particles[i].weight;
    cumulative_sum[i] = weight_sum;
  }

  /* choose random starting position for low-variance walk */
  position = carmen_uniform_random(0, weight_sum);
  step_size = weight_sum / (float)filter->param->num_particles;
  which_particle = 0;
  
  /* draw num_particles random samples */
  for(i = 0; i < filter->param->num_particles; i++) {
    position += step_size;
    if(position > weight_sum) {
      position -= weight_sum;
      which_particle = 0;
    }
    while(position > cumulative_sum[which_particle])
      which_particle++;
    memcpy(temp_particles + i, filter->particles + which_particle,
	   sizeof(carmen_localize_particle_t));
  }

  /* Copy new particles back into the filter. */
  free(filter->particles);
  filter->particles = temp_particles;
  free(cumulative_sum);

  /* set all log weights back to zero */
  for(i = 0; i < filter->param->num_particles; i++)
    filter->particles[i].weight = 0.0;
}

/* incorporate a robot laser reading */

void carmen_localize_run(carmen_localize_particle_filter_p filter, carmen_localize_map_p map,
		  carmen_robot_laser_message *laser, double forward_offset,
		  int backwards)
{
  carmen_point_t robot_position;

  if(!filter->initialized)
    return;

  /* incorporate the laser position stamp */
  robot_position.x = laser->robot_pose.x;
  robot_position.y = laser->robot_pose.y;
  robot_position.theta = laser->robot_pose.theta;
  carmen_localize_incorporate_odometry(filter, robot_position);

  if(filter->param->use_sensor) {
    /* incorporate the laser scan */
    carmen_localize_incorporate_laser(filter, map, laser->num_readings, 
				      laser->range, forward_offset, 
				      laser->config.angular_resolution,
				      laser->config.maximum_range,
				      laser->config.start_angle,
				      backwards);
    
    /* check if it is time to resample */
    if ( filter->distance_travelled > filter->param->update_distance) {
      carmen_localize_resample(filter);
      filter->distance_travelled = 0;
      filter->initialized = 1;
    }
  }
}

void carmen_localize_laser_scan_gd(int num_readings, float *range,
				   double angular_resolution,
				   double first_beam_angle,
				   carmen_point_p laser_pos, double forward_offset,
				   carmen_localize_map_p map, int laser_skip)
{
  float grad_x, grad_y, grad_theta, range_x, range_y, theta;
  int x_l, y_l, count = 0, i;
  
  double angular_res_in_degrees = carmen_radians_to_degrees(angular_resolution);


  do {
    grad_x = 0;
    grad_y = 0;
    grad_theta = 0;
    for(i = 0; i < num_readings; i += laser_skip) {

      theta = laser_pos->theta + first_beam_angle + i * angular_resolution;

      range_x = range[i] * cos(theta);
      range_y = range[i] * sin(theta);
      x_l = (int)((laser_pos->x + forward_offset * cos(laser_pos->theta) +
		   range_x) / map->config.resolution);
      y_l = (int)((laser_pos->y + forward_offset * sin(laser_pos->theta) +
		   range_y) / map->config.resolution);
      
      if(x_l >= 0 && y_l >= 0 && x_l < map->config.x_size &&
	 y_l < map->config.y_size) {
	grad_x += map->x_offset[x_l][y_l];
	grad_y += map->y_offset[x_l][y_l];
	grad_theta += range_x * map->y_offset[x_l][y_l] -
	  range_y * map->x_offset[x_l][y_l];
      }
    }

    /** what is the meaning of this? should this be adapted according to the fov ?*/
    /*     grad_x *= K_T * 180.0 / num_readings; */
    /*     grad_y *= K_T * 180.0 / num_readings; */
    /*     grad_theta *= K_ROT * 180.0 / num_readings; */

    grad_x *= K_T * angular_res_in_degrees;
    grad_y *= K_T * angular_res_in_degrees;
    grad_theta *= K_ROT * angular_res_in_degrees;

    laser_pos->x += grad_x;
    laser_pos->y += grad_y;
    laser_pos->theta += grad_theta;
    count++;
  } while(count < 20 && (grad_x > 0.05 || grad_y < 0.05 ||
			 grad_theta < 0.25 * 180.0 / M_PI));
}

void carmen_localize_summarize(carmen_localize_particle_filter_p filter, 
			       carmen_localize_summary_p summary, 
			       carmen_localize_map_p map,
			       int num_readings, float *range, 
			       double angular_resolution,
			       double first_beam_angle,
			       double forward_offset,
			       int backwards)
{
  float mean_x, mean_y, mean_theta_x, mean_theta_y, angle;
  float diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;
  float *weights, max_weight = filter->particles[0].weight;
  float total_weight = 0;
  int i, x, y;

  summary->converged = !filter->global_mode;

  weights = (float *)calloc(filter->param->num_particles, sizeof(float));
  carmen_test_alloc(weights);
  for(i = 0; i < filter->param->num_particles; i++)
    if(filter->particles[i].weight > max_weight)
      max_weight = filter->particles[i].weight;
  for(i = 0; i < filter->param->num_particles; i++) {
    weights[i] = exp(filter->particles[i].weight - max_weight);
    total_weight += weights[i];
  }

  /* compute mean particle pose */
  mean_x = 0;
  mean_y = 0;
  mean_theta_x = 0;
  mean_theta_y = 0;
  for(i = 0; i < filter->param->num_particles; i++) {
    mean_x += filter->particles[i].x * weights[i];
    mean_y += filter->particles[i].y * weights[i];
    mean_theta_x += cos(filter->particles[i].theta) * weights[i];
    mean_theta_y += sin(filter->particles[i].theta) * weights[i];
  }
  summary->mean.x = mean_x / total_weight;
  summary->mean.y = mean_y / total_weight;
  if(mean_theta_x == 0)
    summary->mean.theta = 0;
  else
    summary->mean.theta = atan2(mean_theta_y, mean_theta_x);
  summary->odometry_pos = filter->last_odometry_position;

  /* compute std particle pose */
  std_x = 0;
  std_y = 0;
  std_theta = 0;
  xy_cov = 0;
  for(i = 0; i < filter->param->num_particles; i++) {
    diff_x = (filter->particles[i].x - summary->mean.x);
    diff_y = (filter->particles[i].y - summary->mean.y);
    diff_theta = carmen_normalize_theta(filter->particles[i].theta -
					summary->mean.theta);
    std_x += carmen_square(diff_x);
    std_y += carmen_square(diff_y);
    std_theta += carmen_square(diff_theta);
    xy_cov += diff_x * diff_y;
  }
  summary->std.x = sqrt(std_x / filter->param->num_particles);
  summary->std.y = sqrt(std_y / filter->param->num_particles);
  summary->std.theta = sqrt(std_theta / filter->param->num_particles);
  summary->xy_cov = sqrt(xy_cov / filter->param->num_particles);

  if(filter->param->do_scanmatching)
    carmen_localize_laser_scan_gd(summary->num_readings, 
				  range, 
				  angular_resolution,
				  first_beam_angle,
				  &summary->mean, 
				  forward_offset, map, 1);

  /* compute mean scan */
  summary->num_readings = num_readings;
  for(i = 0; i < num_readings; i++) {
    summary->mean_scan[i].range = range[i];
    summary->mean_scan[i].mask = filter->laser_mask[i];
    if(backwards) {
      angle = summary->mean.theta + M_PI +
	first_beam_angle + i * angular_resolution;
      summary->mean_scan[i].x = summary->mean.x - forward_offset *
	cos(summary->mean.theta) + cos(angle) * range[i];
      summary->mean_scan[i].y = summary->mean.y - forward_offset *
	sin(summary->mean.theta) + sin(angle) * range[i];
    }
    else {
      angle = summary->mean.theta + 
	first_beam_angle + i * angular_resolution;
      summary->mean_scan[i].x = summary->mean.x + forward_offset *
	cos(summary->mean.theta) + cos(angle) * range[i];
      summary->mean_scan[i].y = summary->mean.y + forward_offset *
	sin(summary->mean.theta) + sin(angle) * range[i];
    }
    x = (summary->mean_scan[i].x / map->config.resolution);
    y = (summary->mean_scan[i].y / map->config.resolution);
    if(x < 0 || y < 0 || x >= map->config.x_size || y >= map->config.y_size ||
       map->carmen_map.map[x][y] == -1)
      summary->mean_scan[i].prob = filter->param->tracking_beam_minlikelihood; //SMALL_PROB;
    else
      summary->mean_scan[i].prob = exp(map->prob[x][y]);
  }

  free(weights);
}
