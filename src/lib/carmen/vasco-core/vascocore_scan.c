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

void
clear_local_map( void )
{
  clear_local_treemap( &(carmen_vascocore_map.qtree), &carmen_vascocore_map,
		       (carmen_vascocore_settings.local_map_kernel_len-1)/2 );
}

void
create_local_map( carmen_vascocore_extd_laser_t data, carmen_move_t move )
{
  create_local_treemap( &carmen_vascocore_map, data, move );
}

void
convolve_map( void )
{
  convolve_treemap( &carmen_vascocore_map );
}

carmen_move_t
find_best_move( carmen_vascocore_extd_laser_t data, carmen_move_t move )
{
  return( fit_data_in_local_map( carmen_vascocore_map, data, move ) );
}

void
vascocore_copy_scan( carmen_laser_laser_message     scan,
		     carmen_vascocore_extd_laser_t *data )
{
  int       i;
  double    delta,start;

  data->time = scan.timestamp;

  data->fov =  scan.config.fov;
  delta = scan.config.angular_resolution;
  start = scan.config.start_angle;

  for (i=0;i<scan.num_readings; i++) {
    data->val[i] = (double) scan.range[i];
    data->angle[i] = start + i*delta;
  }
  data->numvalues = scan.num_readings;
}

int
hpos( int pos )
{
  return(pos % carmen_vascocore_settings.local_map_history_length);
}

carmen_point_t
vascocore_scan_match( carmen_laser_laser_message scan, carmen_point_t pos )
{
  static carmen_point_t  lastpos;
  carmen_point_t         estpos, centerpos, *histpos, nopos = {0.0, 0.0, 0.0};
  carmen_move_t          estmove, bestmove, move, nullmove = {0.0, 0.0, 0.0};
  int                    i, h, hp, hps, p, ctr=0;

  p = hpos( carmen_vascocore_history.ptr );

  vascocore_copy_scan( scan, &(carmen_vascocore_history.data[p]) );

  if (!carmen_vascocore_history.started) {

    /* THE FIRST SCAN WILL BE MAPPED TO 0/0 */
    estpos.x      = 0.0;
    estpos.y      = 0.0;
    estpos.theta  = 0.0;
    if (carmen_vascocore_settings.verbose) {
      fprintf( stderr, "***************************************\n" );
      fprintf( stderr, "* first scan ...\n" );
    }
    carmen_vascocore_history.started = TRUE;

  } else {

    estmove = carmen_move_between_points( lastpos, pos );
    clear_local_map();
    hps = hpos(carmen_vascocore_history.ptr-1);
    centerpos = carmen_vascocore_history.data[hps].estpos;
    /* CREATE LOCAL MAP FROM HISTORY */
    create_local_map( carmen_vascocore_history.data[hps], nullmove );
    histpos = &carmen_vascocore_history.data[hps].estpos;
    for (  h=carmen_vascocore_history.ptr-2;
	   h>=0 &&
	   h>(carmen_vascocore_history.ptr-
	      carmen_vascocore_settings.local_map_history_length) &&
	   ctr<carmen_vascocore_settings.local_map_max_used_history;
	   h--) {
      hp = hpos(h);
      if ( intersect_bboxes( carmen_vascocore_history.data[hps].bbox,
			     carmen_vascocore_history.data[hp].bbox ) &&
	   ( ( (carmen_vascocore_history.ptr-1-h) <
	       carmen_vascocore_settings.local_map_use_last_scans ) ||
	     (carmen_point_dist( *histpos,
				 carmen_vascocore_history.data[hp].estpos ) >
	      carmen_vascocore_settings.local_map_min_bbox_distance)) ) {
	move = carmen_move_between_points( carmen_vascocore_history.data[hp].estpos,
					   centerpos );
	create_local_map( carmen_vascocore_history.data[hp], move );
	histpos = &carmen_vascocore_history.data[hp].estpos;
	ctr++;
      }

    }
    /* COMPUTE AND CONVOLVE LOCAL MAP */
    convolve_map();
    if (carmen_vascocore_settings.verbose) {
      fprintf( stderr, "***************************************\n" );
      fprintf( stderr, "using %d scans\n", ctr );
      fprintf( stderr, "***************************************\n" );
      fprintf( stderr, "estimated movment    %.4f %.4f %.4f\n",
	      estmove.forward, estmove.sideward, estmove.rotation );
    }
    bestmove = find_best_move( carmen_vascocore_history.data[p], estmove );
    if (carmen_vascocore_settings.verbose) {
      fprintf( stderr, "best movment         %.4f %.4f %.4f\n",
	      bestmove.forward, bestmove.sideward, bestmove.rotation );
    }
    estpos = carmen_point_with_move( centerpos, bestmove );
  }

  /* SAVE THE COMPUTED POSITION AND COORDS */
  carmen_vascocore_history.data[p].estpos =
    carmen_point_with_move( nopos, bestmove );
  for (i=0; i<carmen_vascocore_history.data[p].numvalues; i++) {
    carmen_vascocore_history.data[p].coord[i] =
      vascocore_compute_laser2d_coord( carmen_vascocore_history.data[p], i );
  }
  carmen_vascocore_history.data[p].estpos = estpos;
  vascocore_compute_bbox( &carmen_vascocore_history.data[p] );

  lastpos = pos;
  carmen_vascocore_history.ptr++;

  return(estpos);

}

void
vascocore_copy_scan_general(int num_readings, float *range, float *angle,
			    double fov,
			    carmen_vascocore_extd_laser_t *data)
{
  int       i;

  data->time = 0;
  data->numvalues = num_readings;
  for(i = 0; i < num_readings; i++) {
    data->val[i] = range[i];
    data->angle[i] = angle[i];
  }
  data->fov = fov;
}

carmen_point_t
vascocore_scan_match_general(int num_readings, float *range, float *angle,
			     double fov,
			     carmen_point_t pos, int first)
{
  static carmen_point_t  lastpos;
  carmen_point_t         estpos, centerpos, *histpos, nopos = {0.0, 0.0, 0.0};
  carmen_move_t          estmove, bestmove, move, nullmove = {0.0, 0.0, 0.0};
  int                    i, h, hp, hps, p, ctr=0;

  p = hpos( carmen_vascocore_history.ptr );

  vascocore_copy_scan_general(num_readings, range,
			      angle,
			      fov,
			      &(carmen_vascocore_history.data[p]));

  if(first) {
    /* THE FIRST SCAN WILL BE MAPPED TO 0/0 */
    estpos.x      = 0.0;
    estpos.y      = 0.0;
    estpos.theta  = 0.0;
    if (carmen_vascocore_settings.verbose) {
      fprintf( stderr, "***************************************\n" );
      fprintf( stderr, "* first scan ...\n" );
    }
  }
  else {
    estmove = carmen_move_between_points( lastpos, pos );
    clear_local_map();
    hps = hpos(carmen_vascocore_history.ptr-1);
    centerpos = carmen_vascocore_history.data[hps].estpos;
    /* CREATE LOCAL MAP FROM HISTORY */
    create_local_map( carmen_vascocore_history.data[hps], nullmove );
    histpos = &carmen_vascocore_history.data[hps].estpos;
    for (  h=carmen_vascocore_history.ptr-2;
	   h>=0 &&
	   h>(carmen_vascocore_history.ptr-
	      carmen_vascocore_settings.local_map_history_length) &&
	   ctr<carmen_vascocore_settings.local_map_max_used_history;
	   h--) {
      hp = hpos(h);
      if ( intersect_bboxes( carmen_vascocore_history.data[hps].bbox,
			     carmen_vascocore_history.data[hp].bbox ) &&
	   carmen_point_dist( *histpos,
			      carmen_vascocore_history.data[hp].estpos ) >
	   carmen_vascocore_settings.local_map_min_bbox_distance ) {
	move = carmen_move_between_points( carmen_vascocore_history.data[hp].estpos,
					   centerpos );
	create_local_map( carmen_vascocore_history.data[hp], move );
	histpos = &carmen_vascocore_history.data[hp].estpos;
	ctr++;
      }

    }
    /* COMPUTE AND CONVOLVE LOCAL MAP */
    convolve_map();
    if (carmen_vascocore_settings.verbose) {
      fprintf( stderr, "***************************************\n" );
      fprintf( stderr, "using %d scans\n", ctr );
      fprintf( stderr, "***************************************\n" );
      fprintf( stderr, "estimated movment    %.4f %.4f %.4f\n",
	      estmove.forward, estmove.sideward, estmove.rotation );
    }
    bestmove = find_best_move( carmen_vascocore_history.data[p], estmove );
    if (carmen_vascocore_settings.verbose) {
      fprintf( stderr, "best movment         %.4f %.4f %.4f\n",
	      bestmove.forward, bestmove.sideward, bestmove.rotation );
    }
    estpos = carmen_point_with_move( centerpos, bestmove );
  }

  /* SAVE THE COMPUTED POSITION AND COORDS */
  carmen_vascocore_history.data[p].estpos =
    carmen_point_with_move( nopos, bestmove );
  for (i=0; i<carmen_vascocore_history.data[p].numvalues; i++) {
    carmen_vascocore_history.data[p].coord[i] =
      vascocore_compute_laser2d_coord( carmen_vascocore_history.data[p], i );
  }
  carmen_vascocore_history.data[p].estpos = estpos;
  vascocore_compute_bbox( &carmen_vascocore_history.data[p] );

  lastpos = pos;
  carmen_vascocore_history.ptr++;

  return(estpos);

}

