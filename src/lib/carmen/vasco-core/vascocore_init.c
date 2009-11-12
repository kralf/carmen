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

#include "vasco.h"
#include "vascocore_intern.h"

#include "param_interface.h"

carmen_vascocore_param_t      carmen_vascocore_settings;
carmen_vascocore_map_t        carmen_vascocore_map;
carmen_vascocore_history_t    carmen_vascocore_history;

void
vascocore_get_default_params( carmen_vascocore_param_p param, char *laser_type )
{
  if (!strcmp(laser_type, "samsung")) {
    param->verbose = 0;
    param->max_usable_laser_range = 5.5;
    param->local_map_max_range = 5.0;
    param->local_map_resolution = 0.025;
    param->local_map_kernel_len = 7;
    param->local_map_use_odometry = 1;
    param->local_map_num_convolve = 2;
    param->local_map_std_val = 0.00000000001;
    param->local_map_history_length = 3000;
    param->local_map_max_used_history = 90;
    param->local_map_min_bbox_distance = 0.025;
    param->local_map_use_last_scans = 2;
    param->local_map_object_prob = 0.99;
    param->bounding_box_max_range = 5.00;
    param->bounding_box_border = 1.00;
    param->motion_model_forward = 0.0075;
    param->motion_model_sideward = 0.0075;
    param->motion_model_rotation = 0.000872638;
    param->pos_corr_step_size_forward = 0.0015;
    param->pos_corr_step_size_sideward = 0.0015;
    param->pos_corr_step_size_rotation = 0.0872638;
    param->pos_corr_step_size_loop = 7;
  }
  else if (!strcmp(laser_type, "urg")) {
    param->verbose = 0;
    param->max_usable_laser_range = 5.5;
    param->local_map_max_range = 5.5;
    param->local_map_resolution = 0.025;
    param->local_map_kernel_len = 7;
    param->local_map_use_odometry = 0;
    param->local_map_num_convolve = 2;
    param->local_map_std_val = 0.00000000001;
    param->local_map_history_length = 3000;
    param->local_map_max_used_history = 3000;
    param->local_map_min_bbox_distance = 0.025;
    param->local_map_use_last_scans = 2;
    param->local_map_object_prob = 0.99;
    param->bounding_box_max_range = 6.00;
    param->bounding_box_border = 1.00;
    param->motion_model_forward = 0.0075;
    param->motion_model_sideward = 0.0075;
    param->motion_model_rotation = 0.000872638;
    param->pos_corr_step_size_forward = 0.0015;
    param->pos_corr_step_size_sideward = 0.0015;
    param->pos_corr_step_size_rotation = 0.0872638;
    param->pos_corr_step_size_loop = 7;
  }
  else if (!strcmp(laser_type, "s300")) {
    param->verbose = 0;
    param->max_usable_laser_range = 25;
    param->local_map_max_range = 20;
    param->local_map_resolution = 0.025;
    param->local_map_kernel_len = 7;
    param->local_map_use_odometry = 1;
    param->local_map_num_convolve = 2;
    param->local_map_std_val = 0.00000000001;
    param->local_map_history_length = 3000;
    param->local_map_max_used_history = 300;
    param->local_map_min_bbox_distance = 0.025;
    param->local_map_use_last_scans = 2;
    param->local_map_object_prob = 0.99;
    param->bounding_box_max_range = 20.00;
    param->bounding_box_border = 1.00;
    param->motion_model_forward = 0.0075;
    param->motion_model_sideward = 0.0075;
    param->motion_model_rotation = 0.000872638;
    param->pos_corr_step_size_forward = 0.0015;
    param->pos_corr_step_size_sideward = 0.0015;
    param->pos_corr_step_size_rotation = 0.0872638;
    param->pos_corr_step_size_loop = 7;
  }
  else {
    if (strcmp(laser_type, "sick"))
      carmen_warn("Parameter \"vasco_laser_type\" not found; using default (sick) params\n");

    param->verbose = 0;
    param->max_usable_laser_range = 81.90;
    param->local_map_max_range = 20.00;
    param->local_map_resolution = 0.08;
    param->local_map_kernel_len = 5;
    param->local_map_use_odometry = 1;
    param->local_map_num_convolve = 1;
    param->local_map_std_val = 0.01;
    param->local_map_history_length = 5000;
    param->local_map_max_used_history = 300;
    param->local_map_min_bbox_distance = 0.6;
    param->local_map_use_last_scans = 2;
    param->local_map_object_prob = 0.99;
    param->bounding_box_max_range = 20.00;
    param->bounding_box_border = 0.00;
    param->motion_model_forward = 0.013;
    param->motion_model_sideward = 0.013;
    param->motion_model_rotation = 0.125;
    param->pos_corr_step_size_forward = 0.075;
    param->pos_corr_step_size_sideward = 0.075;
    param->pos_corr_step_size_rotation = 0.125;
    param->pos_corr_step_size_loop = 7;
  }
}

void
vascocore_get_params( int argc, char **argv, carmen_vascocore_param_p param )
{
  char *laser_type;

  carmen_param_t param_list[] = {
    {"vasco", "laser_type", CARMEN_PARAM_STRING,
     &laser_type, 0, NULL}
  };

  //dbug: add simple history length params

  carmen_param_install_params(argc, argv, param_list,
			      sizeof(param_list) / sizeof(param_list[0]));

  vascocore_get_default_params(param, laser_type);
}

void
vascocore_initialize_maps( carmen_vascocore_map_t *local_map  )
{
  int                  size_x, size_y;
  carmen_point_t       npos = { 0.0, 0.0, 0.0 };
  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "***************************************\n" );
    fprintf( stderr, "*        MAPS\n" );
    fprintf( stderr, "***************************************\n" );
  }
  size_x = (int) ceil((carmen_vascocore_settings.local_map_max_range)/
		      carmen_vascocore_settings.local_map_resolution);
  size_y = (int) ceil((carmen_vascocore_settings.local_map_max_range)/
		      carmen_vascocore_settings.local_map_resolution);
  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "* INFO: create -local- map: %d x %d\n",
	     2*size_x, size_y );
  }
  initialize_map( local_map, 2*size_x, size_y, 60, size_y/2,
		  carmen_vascocore_settings.local_map_resolution, npos );
  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "***************************************\n" );
  }
}


void
vascocore_alloc_history( carmen_vascocore_history_t * history )
{
  int i, j, nh;

  nh = carmen_vascocore_settings.local_map_history_length;
  nh = (nh>0?nh:1);

  history->data =
    (carmen_vascocore_extd_laser_t *)
    malloc(nh * sizeof(carmen_vascocore_extd_laser_t));
  carmen_test_alloc(history->data);

  for (i=0; i<nh; i++) {
    history->data[i].estpos.x     = 0;
    history->data[i].estpos.y     = 0;
    history->data[i].estpos.theta = 0;
    history->data[i].time         = 0;
    history->data[i].bbox.min.x   = 0;
    history->data[i].bbox.min.y   = 0;
    history->data[i].bbox.max.x   = 0;
    history->data[i].bbox.max.y   = 0;
    history->data[i].numvalues    = 0;
    history->data[i].val        =
      (double *) malloc( MAX_NUM_LASER_VALUES * sizeof(double) );
    carmen_test_alloc(history->data[i].val);
    history->data[i].angle      =
      (double *) malloc( MAX_NUM_LASER_VALUES * sizeof(double) );
    carmen_test_alloc(history->data[i].angle);
    history->data[i].coord      =
      (carmen_vec2_t *) malloc( MAX_NUM_LASER_VALUES * sizeof(carmen_vec2_t) );
    carmen_test_alloc(history->data[i].coord);
    for (j=0; j<MAX_NUM_LASER_VALUES; j++) {
      history->data[i].coord[j].x  = 0;
      history->data[i].coord[j].y  = 0;
      history->data[i].angle[j]    = 0;
      history->data[i].val[j]      = 0;
    }
  }
  history->length = nh;
  history->ptr    = 0;
}

void
vascocore_init( int argc, char **argv )
{
  vascocore_get_params( argc, argv, &carmen_vascocore_settings );
  vascocore_initialize_maps( &carmen_vascocore_map );
  vascocore_alloc_history( &carmen_vascocore_history );
  carmen_vascocore_history.started = 0;
}

void
vascocore_init_no_ipc(carmen_vascocore_param_t *new_settings)
{
  carmen_vascocore_settings = *new_settings;
  vascocore_initialize_maps( &carmen_vascocore_map );
  vascocore_alloc_history( &carmen_vascocore_history );
}

void
vascocore_reset()
{
  carmen_vascocore_history.ptr = 0;
  carmen_vascocore_history.started = 0;
}
