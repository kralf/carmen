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
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <float.h>

#include "vasco.h"
#include "vascocore_intern.h"

double
get_mapval( int pos_x, int pos_y, carmen_vascocore_map_t map )
{
  if ( pos_x>=0 && pos_x<map.mapsize.x &&
       pos_y>=0 && pos_y<map.mapsize.y ) {
      return( EPSILON + map.mapprob[pos_x][pos_y] );
  } else {
      return( EPSILON + carmen_vascocore_settings.local_map_std_val );
  }
  return(0);
}

int
get_mapsum( int pos_x, int pos_y, carmen_vascocore_map_t map )
{
  if ( pos_x>=0 && pos_x<map.mapsize.x &&
       pos_y>=0 && pos_y<map.mapsize.y ) {
      return( map.mapsum[pos_x][pos_y] );
  } else {
    return( 0 );
  }
  return(0);
}

float
get_maphit( int pos_x, int pos_y, carmen_vascocore_map_t map )
{
  if ( pos_x>=0 && pos_x<map.mapsize.x &&
       pos_y>=0 && pos_y<map.mapsize.y ) {
      return( map.maphit[pos_x][pos_y] );
  } else {
    return( 0 );
  }
  return(0);
}

int
max( int a, int b )
{
  return( b>a?b:a );
}

int
find_quadrant( carmen_svec2_t center, int x, int y )
{
  if (x>(center.x)) {
    if (y>(center.y)) {
      return(0);
    } else {
      return(3);
    }
  } else {
    if (y>(center.y)) {
      return(1);
    } else {
      return(2);
    }
  }
}

carmen_svec2_t
newcenter( carmen_svec2_t center, int i, short stepsize )
{
  carmen_svec2_t ncenter = center;
  switch(i) {
  case 0:
    ncenter.x += stepsize;
    ncenter.y += stepsize;
    break;
  case 1:
    ncenter.x -= stepsize;
    ncenter.y += stepsize;
    break;
  case 2:
    ncenter.x -= stepsize;
    ncenter.y -= stepsize;
    break;
  case 3:
    ncenter.x += stepsize;
    ncenter.y -= stepsize;
    break;
  }
  return(ncenter);
}

void
alloc_tree( carmen_vascocore_quad_tree_t * tree,
	    int level, carmen_svec2_t center, short stepsize )
{
  int i;
  short nstepsize = stepsize/2;
  tree->center = center;
  tree->level  = level;
  tree->inuse  = FALSE;
  if (level>0) {
    for( i=0; i<4; i++) {
      tree->elem[i] = (carmen_vascocore_quad_tree_t *)
	malloc( sizeof(carmen_vascocore_quad_tree_t) );
      carmen_test_alloc(tree->elem[i]);
      alloc_tree( tree->elem[i], level-1,
		  newcenter( center, i, nstepsize ), nstepsize );
    }
  }
}

void
initialize_qtree( carmen_vascocore_quad_tree_t * tree, int size_x, int size_y)
{
  int i,v,nlevel = max( (int) ceil(log10(size_x)/log10(2)),
			(int) ceil(log10(size_y)/log10(2)) );
  carmen_svec2_t center;
  if (carmen_vascocore_settings.verbose)
    fprintf( stderr, "* INFO: num levels       = %d\n", nlevel );
  v = 1;
  for (i=0;i<nlevel;i++) v=v*2;
  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "* INFO: size             = %d/%d\n", size_x, size_y );
    fprintf( stderr, "* INFO: poss. max size   = %d/%d\n", v, v );
  }
  center.x = v-1;
  center.y = v-1;
  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "* INFO: tree center:       %5.1f %5.1f\n",
	     center.x/2.0, center.y/2.0 );
    fprintf( stderr, "* INFO: tree step:         %5.1f %5.1f\n",
	     0.5*(v/2), 0.5*(v/2) );
    fprintf( stderr, "* INFO: allocate tree: ... " );
  }
  alloc_tree( tree, nlevel, center, v );
  if (carmen_vascocore_settings.verbose)
    fprintf( stderr, "done\n" );
}

void
initialize_map( carmen_vascocore_map_t * map,
		int sx, int sy, int center_x, int center_y,
		double resolution, carmen_point_t start )
{
  int x, y;

  map->mapsize.x  = sx;
  map->mapsize.y  = sy;
  map->resolution = resolution;
  map->offset     = start;
  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "* INFO: allocating memory ... " );
  }
  map->updated  = carmen_mdalloc( 2, sizeof(unsigned char),  sx, sy );
  carmen_test_alloc(map->updated);
  map->maphit   = carmen_mdalloc( 2, sizeof(float),  sx, sy );
  carmen_test_alloc(map->maphit);
  map->mapsum   = carmen_mdalloc( 2, sizeof(short),  sx, sy );
  carmen_test_alloc(map->mapsum);
  map->mapprob  = carmen_mdalloc( 2, sizeof(float), sx, sy );
  carmen_test_alloc(map->mapprob);
  map->calc     = carmen_mdalloc( 2, sizeof(float), sx, sy );
  carmen_test_alloc(map->calc);
  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "done\n" );
  }
  map->center.x = center_x;
  map->center.y = center_y;

  if (carmen_vascocore_settings.verbose) {
    fprintf( stderr, "* INFO: map:            %d %d\n",
	     map->mapsize.x, map->mapsize.y );
    fprintf( stderr, "* INFO: center:         %.1f %.1f\n",
	     map->center.x, map->center.y );
    fprintf( stderr, "* INFO: resolution:      %.2f\n",
	     map->resolution );
    fprintf( stderr, "* INFO: real-size:      [%.1f %.1f] [%.1f %.1f]\n",
	     -sx*map->resolution, sx*map->resolution,
	     -sy*map->resolution, sy*map->resolution );
    fprintf( stderr, "***************************************\n" );
  }

  for (x=0;x<sx;x++) {
    for (y=0;y<sy;y++) {
      map->mapprob[x][y] = carmen_vascocore_settings.local_map_std_val;
      map->calc[x][y]    = carmen_vascocore_settings.local_map_std_val;
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
      map->updated[x][y]  = UPDT_NOT;
    }
  }
  initialize_qtree( &(map->qtree), sx, sy );
}

void
compute_prob_point( carmen_vascocore_map_t *map, int x, int y )
{

  if ( x>=0 && x<map->mapsize.x &&
       y>=0 && y<map->mapsize.y ) {
    if (get_mapsum(x,y,*map)>0) {
      if (0) {
	map->mapprob[x][y]     = map->maphit[x][y] / map->mapsum[x][y];
      } else {
	map->mapprob[x][y]     = 1.0;
      }
    } else {
      map->mapprob[x][y]     = carmen_vascocore_settings.local_map_std_val;
    }
  }

}

void
convolve_calc_point( carmen_vascocore_map_t *map,
		     carmen_gauss_kernel_t kernel, int hk,
		     int x, int y, double std_val )
{
  int                     k;
  double                  ksum = 0.0;

  if (x-hk>=0 && x+hk<map->mapsize.x) {
    for (k=0;k<2*hk+1;k++) {
      if (get_mapsum(x+k-hk,y,*map)>0) {
	ksum +=  ( kernel.val[k] * map->mapprob[x+k-hk][y] );
      } else {
	ksum +=  ( kernel.val[k] * std_val );
	}
    }
    map->calc[x][y]     = ksum;
    map->updated[x][y]  = UPDT_X;
  }
}

void
convolve_prob_point( carmen_vascocore_map_t *map,
		     carmen_gauss_kernel_t kernel, int hk, int x, int y )
{
  int                     k;
  double                  ksum = 0.0;

  for (k=0;k<2*hk+1;k++) {
    if ( x>=0 && x<map->mapsize.x &&
	 y>=0 && y<map->mapsize.y)
      ksum +=  ( kernel.val[k] * map->calc[x][y+k-hk] );
  }
  map->mapprob[x][y]  = ksum;
  map->updated[x][y]  = UPDT_Y;

}

void
compute_prob_treemap( carmen_vascocore_quad_tree_t *tree,
		      carmen_vascocore_map_t *map )
{
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      compute_prob_treemap( tree->elem[0], map );
    if (tree->elem[1]->inuse)
      compute_prob_treemap( tree->elem[1], map );
    if (tree->elem[2]->inuse)
      compute_prob_treemap( tree->elem[2], map );
    if (tree->elem[3]->inuse)
      compute_prob_treemap( tree->elem[3], map );
  } else {
    compute_prob_point( map, (tree->center.x/2), (tree->center.y/2) );
  }
}

void
convolve_calc_treemap( carmen_vascocore_quad_tree_t *tree,
		       carmen_vascocore_map_t *map,
		       carmen_gauss_kernel_t kernel, int hk, double std )
{
  int i, j;
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      convolve_calc_treemap( tree->elem[0], map, kernel, hk, std );
    if (tree->elem[1]->inuse)
      convolve_calc_treemap( tree->elem[1], map, kernel, hk, std );
    if (tree->elem[2]->inuse)
      convolve_calc_treemap( tree->elem[2], map, kernel, hk, std );
    if (tree->elem[3]->inuse)
      convolve_calc_treemap( tree->elem[3], map, kernel, hk, std );
  } else {
    if ( (tree->center.x/2)>hk+1 && (tree->center.x/2)<map->mapsize.x-hk-1 &&
	 (tree->center.y/2)>hk+1 && (tree->center.y/2)<map->mapsize.y-hk-1 ) {
      for (i=(tree->center.x/2)-hk;i<(tree->center.x/2)+hk;i++) {
	for (j=(tree->center.y/2)-hk;j<(tree->center.y/2)+hk;j++) {
	  if (map->updated[i][j] != UPDT_X)
	    convolve_calc_point( map, kernel, hk, i, j, std );
	}
      }
    }

  }
}

void
convolve_prob_treemap( carmen_vascocore_quad_tree_t *tree,
		       carmen_vascocore_map_t *map,
		       carmen_gauss_kernel_t kernel, int hk )
{
  int i, j;
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      convolve_prob_treemap( tree->elem[0], map, kernel, hk );
    if (tree->elem[1]->inuse)
      convolve_prob_treemap( tree->elem[1], map, kernel, hk );
    if (tree->elem[2]->inuse)
      convolve_prob_treemap( tree->elem[2], map, kernel, hk );
    if (tree->elem[3]->inuse)
      convolve_prob_treemap( tree->elem[3], map, kernel, hk );
  } else {
    if ( (tree->center.x/2)>hk+1 && (tree->center.x/2)<map->mapsize.x-hk-1 &&
	 (tree->center.y/2)>hk+1 && (tree->center.y/2)<map->mapsize.y-hk-1 ) {
      for (i=(tree->center.x/2)-hk;i<(tree->center.x/2)+hk;i++) {
	for (j=(tree->center.y/2)-hk;j<(tree->center.y/2)+hk;j++) {
	  if (map->updated[i][j] != UPDT_Y)
	    convolve_prob_point( map, kernel, hk, i, j );
	}
      }
    }
  }
}

void
convolve_treemap( carmen_vascocore_map_t *map )
{
  int                              i;
  static int                       hk, first_time = TRUE;
  static carmen_gauss_kernel_t     kernel;

  if (first_time) {
    hk = (carmen_vascocore_settings.local_map_kernel_len-1)/2;
    kernel = carmen_gauss_kernel( carmen_vascocore_settings.local_map_kernel_len );
    first_time = FALSE;
  }

  compute_prob_treemap(  &(map->qtree), map );
  for (i=0;i<carmen_vascocore_settings.local_map_num_convolve;i++) {
    convolve_calc_treemap( &(map->qtree), map, kernel, hk,
			   carmen_vascocore_settings.local_map_std_val );
    convolve_prob_treemap( &(map->qtree), map, kernel, hk );
  }

}

int
compute_map_pos_from_pos( carmen_point_t pos,
			  carmen_vascocore_map_t map, carmen_ivec2_t *v )
{
  v->x = map.center.x + (int) (pos.x/(double)map.resolution);
  v->y = map.center.y + (int) (pos.y/(double)map.resolution);
  if (v->x<0 || v->x>map.mapsize.x-1) {
    return(FALSE);
  }
  if (v->y<0 || v->y>map.mapsize.y-1) {
    return(FALSE);
  }
  return(TRUE);
}

int
compute_map_pos_from_vec2( carmen_vec2_t vec,
			   carmen_vascocore_map_t map, carmen_ivec2_t *v )
{
  v->x = map.center.x + (int) (vec.x/(double)map.resolution);
  v->y = map.center.y + (int) (vec.y/(double)map.resolution);
  if (v->x<0) {
    return(FALSE);
  } else if (v->x>map.mapsize.x-1) {
    return(FALSE);
  }
  if (v->y<0) {
    return(FALSE);
  } else if (v->y>map.mapsize.y-1) {
    return(FALSE);
  }
  return(TRUE);
}

void
mark_maphitpoint( carmen_vascocore_quad_tree_t *tree,
		  carmen_vascocore_map_t *map, int x, int y, float value )
{
  tree->inuse=TRUE;
  if ((tree->level)>0) {
    mark_maphitpoint( tree->elem[find_quadrant( tree->center, x, y )],
		      map, x, y, value  );
  } else {
    map->maphit[tree->center.x/2][tree->center.y/2] += value;
  }
}

void
mark_mapsumpoint( carmen_vascocore_quad_tree_t *tree,
		  carmen_vascocore_map_t *map, int x, int y )
{
  tree->inuse=TRUE;
  if ((tree->level)>0) {
    mark_mapsumpoint( tree->elem[find_quadrant( tree->center, x, y )],
		      map, x, y );
  } else {
    map->mapsum[tree->center.x/2][tree->center.y/2]++;
  }
}

void
set_maphitpoint( carmen_vascocore_map_t *map, int x, int y, float value  )
{
  mark_maphitpoint( &(map->qtree), map, 2*x, 2*y, value );
}

void
set_mapsumpoint( carmen_vascocore_map_t *map, int x, int y )
{
  mark_mapsumpoint( &(map->qtree), map, 2*x, 2*y );
}

void
tree_list( carmen_vascocore_quad_tree_t *tree , int *ct )
{
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      tree_list( tree->elem[0], ct );
    if (tree->elem[1]->inuse)
      tree_list( tree->elem[1], ct );
    if (tree->elem[2]->inuse)
      tree_list( tree->elem[2], ct );
    if (tree->elem[3]->inuse)
      tree_list( tree->elem[3], ct );
  } else {
    (*ct)++;
  }
}

void
clear_local_treemap( carmen_vascocore_quad_tree_t *tree,
		     carmen_vascocore_map_t *map, int hk )
{
  int i,j;
  if ((tree->level)>0 ) {
    if (tree->elem[0]->inuse)
      clear_local_treemap( tree->elem[0], map, hk );
    if (tree->elem[1]->inuse)
      clear_local_treemap( tree->elem[1], map, hk );
    if (tree->elem[2]->inuse)
      clear_local_treemap( tree->elem[2], map, hk );
    if (tree->elem[3]->inuse)
      clear_local_treemap( tree->elem[3], map, hk );
  } else {
    if ( (tree->center.x/2)>hk-1 && (tree->center.x/2)<map->mapsize.x-hk &&
	 (tree->center.y/2)>hk-1 && (tree->center.y/2)<map->mapsize.y-hk ) {
      for (i=(tree->center.x/2)-hk;i<=(tree->center.x/2)+hk;i++) {
	for (j=(tree->center.y/2)-hk;j<=(tree->center.y/2)+hk;j++) {
	  map->maphit[i][j]  = 0;
	  map->mapsum[i][j]  = 0.0;
	  map->mapprob[i][j] = carmen_vascocore_settings.local_map_std_val;
	  map->calc[i][j]    = carmen_vascocore_settings.local_map_std_val;
	  map->updated[i][j] = UPDT_NOT;
	}
      }
    }
  }
  tree->inuse = FALSE;
}

void
create_local_treemap( carmen_vascocore_map_t         * map,
		      carmen_vascocore_extd_laser_t    data,
		      carmen_move_t                    movement )
{
  int                         i;
  carmen_point_t              rpos;
  carmen_ivec2_t              start, end;
  carmen_vec2_t               lpos;

  rpos  = carmen_point_backwards_from_move( movement );
  compute_map_pos_from_pos( rpos, *map, &start );
  for (i=0;i<data.numvalues;i++) {
    lpos = carmen_laser_point( rpos,  data.val[i], data.angle[i] );
    if (compute_map_pos_from_vec2( lpos, *map, &end )) {
      if ( data.val[i]<carmen_vascocore_settings.local_map_max_range &&
	   end.x>=0 && end.x<map->mapsize.x &&
	   end.y>=0 && end.y<map->mapsize.y ) {
	set_mapsumpoint( map, end.x, end.y );
      }
    }
  }
}

carmen_vec2_t
vascocore_compute_laser2d_coord( carmen_vascocore_extd_laser_t data, int i )
{
  return( carmen_laser_point( data.estpos, data.val[i], data.angle[i] ) );
}


int
minimal_rpos_diff( carmen_point_t pos1, carmen_point_t pos2,
		   double pos_diff_min_dist,
		   double pos_diff_min_rot )
{
  carmen_vec2_t v1, v2;
  v1.x = pos1.x;    v1.y = pos1.y;
  v2.x = pos2.x;    v2.y = pos2.y;
  if ( carmen_vec_distance(v1,v2) > pos_diff_min_dist )
    return(TRUE);
  if ( carmen_orientation_diff(pos1.theta,pos2.theta) > pos_diff_min_rot )
    return(TRUE);
  return(FALSE);
}

int
minimal_rmove_diff( carmen_move_t move,
		    double pos_diff_min_dist,
		    double pos_diff_min_rot )
{
  carmen_vec2_t v1;
  v1.x = move.forward;    v1.y = move.sideward;
  if ( carmen_vec_length(v1) > pos_diff_min_dist )
    return(TRUE);
  if ( move.rotation > pos_diff_min_rot )
    return(TRUE);
  return(FALSE);
}

void
vascocore_compute_bbox( carmen_vascocore_extd_laser_t *data )
{
  int i;
  carmen_vec2_t min,max;
  min.x = DBL_MAX;     min.y = DBL_MAX;
  max.x = -DBL_MAX;    max.y = -DBL_MAX;
  for (i=0;i<data->numvalues;i++) {
    if (data->val[i]<carmen_vascocore_settings.bounding_box_max_range) {
      if (data->coord[i].x<min.x)
	min.x = data->coord[i].x;
      if (data->coord[i].y<min.y)
	min.y = data->coord[i].y;
      if (data->coord[i].x>max.x)
	max.x = data->coord[i].x;
      if (data->coord[i].y>max.y)
	max.y = data->coord[i].y;
    }
  }
  min.x -= carmen_vascocore_settings.bounding_box_border;
  min.y -= carmen_vascocore_settings.bounding_box_border;
  max.x += carmen_vascocore_settings.bounding_box_border;
  max.y += carmen_vascocore_settings.bounding_box_border;
  data->bbox.min = min;
  data->bbox.max = max;
}

int
intersect_bboxes( carmen_bbox_t box1, carmen_bbox_t box2 )
{
  if (box1.min.x<=box2.min.x) {
    /* box1.min.x is smaller that box2 */
    if (box1.max.x>box2.min.x) {
      /* intersection in x */
      if (box1.min.y<=box2.min.y) {
	/* box1.min.y is smaller that box2 */
	if (box1.max.y>box2.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      } else {
	/* box2.min.y is smaller that box1 */
	if (box2.max.y>=box1.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      }
    } else {
      return(0);
    }
  } else {
    /* box2.min.x is smaller that box1 */
    if (box2.max.x>=box1.min.x) {
      /* intersection in x */
      if (box1.min.y<=box2.min.y) {
	/* box1.min.y is smaller that box2 */
	if (box1.max.y>box2.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      } else {
	/* box2.min.y is smaller that box1 */
	if (box2.max.y>=box1.min.y) {
	  /* intersection in y */
	  return(1);
	} else {
	  return(0);
	}
      }
    } else {
      return(0);
    }
  }
  return(0);
}

