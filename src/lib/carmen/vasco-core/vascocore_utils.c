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
#include <stdarg.h>
#include <stdlib.h>

#include "vasco.h"
#include "vascocore_intern.h"

/*********************************************************************/
/*   Written by Blair Haukedal 91/09 and placed in the public domain */
/*********************************************************************/

static void **md2(int n_units, int ndim, int *dims);
static void md3(char ***tip, int n_units, int ndim, int *dims);

static int w_units;

/* mdalloc: entry point for mdalloc function described above
 *      - reduces variable arg list to fixed list with last arg
 *      represented as pointer to int (array dimensions).
 *      Calls md2 to allocate storage.
 *      Calls md3 to initialize intermediate pointers.
 *      Returns pointer.
 */

void *
carmen_mdalloc(int ndim, int width, ...)
{
  va_list argp;
  int *dims, i;
  char ***tip;

  va_start(argp, width);

  /* allocate storage for variable args (dimensions) */

  dims = malloc(ndim*sizeof(int));
  /* check_alloc checked */
  if(dims == NULL)
    return NULL;

  /* initialize dimensions array for subsequent calls */

  for(i=0; i<ndim; i++)
    dims[i] = va_arg(argp,int);

  w_units = width;    /* global used by md2 and md3 */

  /* allocate required pointer and array element storage */

  tip = (char ***)md2(dims[0], ndim, &dims[1]);

  if(ndim>1 && tip)
    md3(tip, dims[0], ndim-1, &dims[1]); /* init pointers */

  free(dims);
  return tip;
}

/* mdfree:  companion function to mdalloc
 *          frees storage obtained by mdalloc
 */

void
carmen_mdfree(void *tip, int ndim)
{
  if(ndim == 1)
    free(tip);
  else
    {
      carmen_mdfree(((void **)tip)[0], ndim-1);
      free(tip);
    }
}

/* md2:  allocates storage for n-way indirect pointer arrays
 *       allocates storage for requested array elements
 */

static void **
md2(int n_units, int ndim, int *dims)
{
  char **tip;

  if(ndim == 1) {
    /* recursed to final dimension - allocate element storage */
    tip = malloc(n_units*w_units);
    carmen_test_alloc(tip);
  }
  else
    {
      /* allocate pointer array for dimension n */
      tip = malloc(n_units*sizeof(char *));
      carmen_test_alloc(tip);
      if(tip)
	{
	  /* recurse until final dimension */
	  tip[0] = (char *)md2(n_units*dims[0], ndim-1, &dims[1]);
	  if(tip[0] == NULL)
	    {
                        /* allocate error - fall back up freeing everything */
	      free(tip);
	      tip = NULL;
	    }
	}
    }
  return (void **)tip;
}

/* md3: initializes indirect pointer arrays */

static void
md3(char ***tip, int n_units, int ndim, int *dims)
{
  int i;

  for(i=1; i<n_units; i++)
    {
      if(ndim == 1)
	/* final dimension - must scale by element width */
	tip[i] = (char **)((char *)tip[0] + i*dims[0]*w_units);
      else
	/* intermediate dimension - scale by pointer size */
	tip[i] = tip[0] + i*dims[0];
    }
  if(ndim > 1)
            /* not at final dimension - continue to recurse */
    md3((char ***)tip[0], n_units*dims[0], ndim-1, &dims[1]);
}

/*********************************************************************/

double
carmen_vec_distance( carmen_vec2_t p1, carmen_vec2_t p2 ) {
  return sqrt( (p1.x-p2.x)*(p1.x-p2.x) +
	       (p1.y-p2.y)*(p1.y-p2.y) );
}

double
carmen_vec_length( carmen_vec2_t v1 )
{
  return( sqrt( (v1.x*v1.x) + (v1.y*v1.y) ) );
}

double
carmen_point_dist( carmen_point_t pos1, carmen_point_t pos2 )
{
  return(
	 sqrt( (pos1.x - pos2.x) * (pos1.x - pos2.x) +
	       (pos1.y - pos2.y) * (pos1.y - pos2.y) )
	 );
}


double
carmen_move_length( carmen_move_t move )
{
  return(
	 sqrt( (move.forward * move.forward) +
	       (move.sideward * move.sideward) )
	 );
}

carmen_vec2_t
carmen_laser_point( carmen_point_t rpos, double val, double angle )
{
  carmen_vec2_t abspt;
  abspt.x =
    rpos.x + cos( angle+rpos.theta ) * val;
  abspt.y =
    rpos.y + sin( angle+rpos.theta ) * val;
  return(abspt);
}

double
carmen_gauss( double x, double mu, double sigma )
{
  if (sigma < 1e-9) {
    if (fabs(x-mu) < 1e-9)
      return 1.0;
    else
      return 0.0;
  }
 return( (1/sqrt(2.0*M_PI*sigma*sigma)) *
	 exp(-(((x-mu)*(x-mu))/(2*sigma*sigma))) );
}

carmen_gauss_kernel_t
carmen_gauss_kernel(int length )
{
  carmen_gauss_kernel_t  kernel;
  int i, j, *store, sum = 0;
  store = (int *) malloc( length*sizeof(int) );
  carmen_test_alloc(store);
  store[0] = 1;
  for ( i=0; i<length-1; i++ ) {
    store[i+1] = 1;
    for ( j=i; j>0; j-- ) {
      store[j] = store[j] + store[j-1];
    }
  }
  for ( i=0; i<length; i++ ) {
    sum += store[i];
  }
  kernel.len = length;
  kernel.val = (double *) malloc( length * sizeof(double) );
  carmen_test_alloc(kernel.val);
  for (i=0;i<length;i++) {
    kernel.val[i] = store[i] / (double) sum;
  }
  free( store );
  return(kernel);
}

double
carmen_orientation_diff( double start, double end ) {
  double diff;
  diff =
    carmen_normalize_theta( start ) -
    carmen_normalize_theta( end );
  if (diff<-M_PI) {
    return(2*M_PI+diff);
  } else if (diff>M_PI) {
    return(-2*M_PI+diff);
  } else {
    return(diff);
  }
}

carmen_point_t
carmen_point_with_move( carmen_point_t start, carmen_move_t move )
{
  carmen_point_t end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) )
    return (start);
  end.x =
    start.x +
    cos(start.theta) * move.forward +
    sin(start.theta) * move.sideward;
  end.y =
    start.y +
    sin(start.theta) * move.forward -
    cos(start.theta) * move.sideward;
  end.theta =
    carmen_normalize_theta( start.theta + move.rotation );
  return(end);
}

carmen_point_t
carmen_point_from_move( carmen_move_t move )
{
  carmen_point_t end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) ) {
    end.x = 0.0; end.y = 0.0; end.theta = 0.0;
  } else {
    end.x     =   move.forward;
    end.y     = - move.sideward;
    end.theta =   carmen_normalize_theta( move.rotation );
  }
  return(end);
}

carmen_point_t
carmen_point_backwards_with_move( carmen_point_t start, carmen_move_t move )
{
  carmen_point_t end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) ) {
    return (start);
  } else {
    end.theta =
      carmen_normalize_theta( start.theta - move.rotation );
    end.x =
      start.x -
      cos(end.theta) * move.forward -
      sin(end.theta) * move.sideward;
    end.y =
      start.y -
      sin(end.theta) * move.forward +
      cos(end.theta) * move.sideward;
  }
  return(end);
}

carmen_point_t
carmen_point_backwards_from_move( carmen_move_t move )
{
  carmen_point_t end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) ) {
    end.x = 0.0; end.y = 0.0; end.theta = 0.0;
  } else {
    end.theta = carmen_normalize_theta( -move.rotation );
    end.x =
      - cos(end.theta) * move.forward
      - sin(end.theta) * move.sideward;
    end.y =
      - sin(end.theta) * move.forward
      + cos(end.theta) * move.sideward;
  }
  return(end);
}

carmen_move_t
carmen_move_between_points( carmen_point_t start, carmen_point_t end )
{
  carmen_move_t move;
  /* compute forward and sideward sensing_MOVEMENT */
  move.forward =
    + (end.y - start.y) * sin(start.theta)
    + (end.x - start.x) * cos(start.theta);

  move.sideward =
    - (end.y - start.y) * cos(start.theta)
    + (end.x - start.x) * sin(start.theta);

  move.rotation = carmen_orientation_diff( end.theta, start.theta );

  return( move );
}


