#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "global.h"
#include "map_io.h"

#include "log_graphics.h"

#include "log2pic.h"

log2pic_settings_t  settings = {
  /* enum FORMAT_TYPE     format; */
  GRAPHICS,
  /* int       display_arrow; */
  FALSE,
  /* RGB       bg; */
  { 0.9, 0.9, 0.9 },
  /*  char      bgcolor[MAX_STRING_LENGTH]; */
  "#e6e6e6",
  /*  double    darken; */
  1.0,
  /*  int       showpath; */
  FALSE,
  /*  char      pathcolor[MAX_STRING_LENGTH]; */
  "red",
  /*  double    pathwidth; */
  2.0,
  /*  double    rotation_angle; */
  DEFAULT_ROTATION_ANGLE,
  /*  logtools_vector2_t   rotation_center; */
  {0.0,0.0},
  /*  char      infilename[MAX_STRING_LENGTH]; */
  "in.rec",
  /*  char      outfilename[MAX_STRING_LENGTH]; */
  "out.png",
  /*  char      filetemplate[MAX_STRING_LENGTH]; */
  "dump",
  /* double     usable_range; */
  DEFAULT_MAX_USABLE_RANGE,
  /* double     max_range; */
  DEFAULT_MAX_RANGE,
  /* double     zoom; */
  DEFAULT_ZOOM,
  /* double     resolution_x; */
  DEFAULT_RESOLUTION,
  /* double     resolution_y; */
  DEFAULT_RESOLUTION,
  /* int        utm_correct; */
  FALSE,
  /* int        google_correct; */
  FALSE,
  /* int        google_zoom; */
  0,
  /* double     border; */
  DEFAULT_BORDER,
  /* double     unknown_val; */
  MAP_STD_VAL,
  /* int        use_odds_model; */
  FALSE,
  /* int        static_prob; */
  STATIC_PROB,
  /* int        dynamic_prob; */
  DYNAMIC_PROB,
  /* int        flip; */
  FALSE,
  /* int        animation; */
  FALSE,
  /* double     anim_step; */
  50.0,
  /* int        anim_skip; */
  0,
  /* int        laser_id; */
  DEFAULT_LASER_ID,
  /* int        endpoints; */
  FALSE,
  /* int        from; */
  -1,
  /* int        to; */
  -1,
  /* int        convolve; */
  FALSE,
  /* int        kernel_size; */
  5,
  /* int        integrate_scans; */
  TRUE,
  /* int        set_size; */
  FALSE,
  /* int        crop_size; */
  FALSE,
  /* double     min_x; */
  0.0,
  /* double     min_y; */
  0.0,
  /* double     max_x; */
  0.0,
  /* double     max_y; */
  0.0,
  /* int        set_pos; */
  FALSE,
  /* double     pos_x; */
  0.0,
  /* double     pos_y; */
  0.0,
  /* double     pos_o; */
  0.0,
  /* int        bgfile; */
  FALSE,
  /* IMAGE_TYPE background; */
  { 0, 0, { 0, 0 }, NULL },
  /*   int      gpspath; */
  FALSE,
  /* logtools_vector2_t  bgoffset; */
  {0.0, 0.0},
  /* logtools_rpos2_t              posstart; */
  {0.0, 0.0, 0.0}
};

void
fast_grid_line( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line )
{
  int dy = end.y - start.y;
  int dx = end.x - start.x;
  int stepx, stepy;
  int fraction, cnt = 0;

  if (dy < 0) {
    dy = -dy;
    stepy = -1;
  } else {
    stepy = 1;
  }
  if (dx < 0) {
    dx = -dx;
    stepx = -1;
  } else {
    stepx = 1;
  }

  dy <<= 1;
  dx <<= 1;

  line->grid[cnt++]=start;

  if (dx > dy) {
    fraction = dy - (dx >> 1);
    while (start.x != end.x) {
      if (fraction >= 0) {
	start.y += stepy;
	fraction -= dx;
      }
      start.x += stepx;
      fraction += dy;
      line->grid[cnt++]=start;
    }
  } else {
    fraction = dx - (dy >> 1);
    while (start.y != end.y) {
      if (fraction >= 0) {
	start.x += stepx;
	fraction -= dy;
      }
      start.y += stepy;
      fraction += dx;
      line->grid[cnt++]=start;
    }
  }
  line->numgrids = cnt;
}

void
grid_line_core( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line )
{
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  int cnt = 0;

  dx = abs(end.x-start.x); dy = abs(end.y-start.y);

  if (dy <= dx) {
    d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x; y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x; y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    line->grid[cnt].x=x;
    line->grid[cnt].y=y;
    cnt++;
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y++; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    } else {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y--; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    }
  } else {
    d = 2*dx - dy;
    incr1 = 2*dx; incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y; x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y; x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    line->grid[cnt].x=x;
    line->grid[cnt].y=y;
    cnt++;
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x++; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    } else {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x--; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    }
  }
  line->numgrids = cnt;
}

void
grid_line( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line ) {
  int i,j;
  int half;
  logtools_ivector2_t v;
  grid_line_core( start, end, line );
  if ( start.x!=line->grid[0].x ||
       start.y!=line->grid[0].y ) {
    half = line->numgrids/2;
    for (i=0,j=line->numgrids - 1;i<half; i++,j--) {
      v = line->grid[i];
      line->grid[i] = line->grid[j];
      line->grid[j] = v;
    }
  }
}

int
log2pic_map_pos_from_vec2( logtools_vector2_t pos,
			   logtools_grid_map2_t *map,
			   logtools_vector2_t *v )
{
  v->x = (map->center.x + (pos.x-map->offset.x)/settings.resolution_x);
  v->y = (map->center.y + (pos.y-map->offset.y)/settings.resolution_y);
  if (v->x<0) {
    return(FALSE);
  } else if (v->x>=map->mapsize.x) {
    return(FALSE);
  }
  if (v->y<0) {
    return(FALSE);
  } else if (v->y>=map->mapsize.y) {
    return(FALSE);
  }
  return(TRUE);
}

int
log2pic_map_pos_from_rpos( logtools_rpos2_t rpos, logtools_grid_map2_t *map,
			   logtools_vector2_t *v )
{
  logtools_vector2_t pos;
  pos.x = rpos.x;
  pos.y = rpos.y;
  return(log2pic_map_pos_from_vec2( pos, map, v));
}

int
log2pic_imap_pos_from_vec2( logtools_vector2_t pos,
			    logtools_grid_map2_t *map, logtools_ivector2_t *iv )
{
  logtools_vector2_t v;
  int ret = log2pic_map_pos_from_vec2( pos, map, &v );
  iv->x = (int)v.x;
  iv->y = (int)v.y;
  return(ret);
}

int
log2pic_imap_pos_from_rpos( logtools_rpos2_t rpos, logtools_grid_map2_t *map,
			    logtools_ivector2_t *iv )
{
  logtools_vector2_t v;
  int ret = log2pic_map_pos_from_rpos( rpos, map, &v );
  iv->x = (int)v.x;
  iv->y = (int)v.y;
  return(ret);
}

void
log2pic_simple_convolve_map( logtools_grid_map2_t *map, logtools_gauss_kernel_t kernel )
{
  int x, y, k, hk;
  double ksum;

  hk = ( kernel.len - 1 ) / 2;
  for (x=hk;x<map->mapsize.x-hk;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * map->mapprob[x+k-hk][y] );
      }
      map->calc[x][y] = ksum;
      if (map->calc[x][y]>1.0)
	map->calc[x][y]=1.0;
    }
  }
  for (x=0;x<map->mapsize.x;x++) {
    for (y=hk;y<map->mapsize.y-hk;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * map->calc[x][y+k-hk] );
      }
      map->mapprob[x][y] = ksum;
      if (map->mapprob[x][y]>0.0) {
	if (map->mapprob[x][y]>1.0)
	  map->mapprob[x][y]=1.0;
	map->mapsum[x][y]++;
      }
    }
  }
}

void
log2pic_map_integrate_scan( logtools_grid_map2_t * map, logtools_lasersens2_data_t data,
			    double max_range, double max_usable  )
{
  static int            first_time = TRUE;
  static logtools_grid_line_t      line;
  static int            max_num_linepoints = 0;
  int                   i, j, x, y;
  logtools_ivector2_t              start, end;
  logtools_vector2_t    abspt;
  logtools_rmove2_t                nomove = {0.0, 0.0, 0.0};

  if (first_time) {
    max_num_linepoints =
      3 * ( max_range / map->resolution );
    line.grid =
      (logtools_ivector2_t *) malloc( max_num_linepoints *
				      sizeof(logtools_ivector2_t) );
    first_time = FALSE;
  }

  if (settings.integrate_scans) {
    for (j=0;j<data.laser.numvalues;j++) {
      if (data.laser.val[j] <= max_usable ) {
	if (settings.endpoints) {
	  if (data.laser.val[j] <= max_range ) {
	    abspt = logtools_compute_laser_points( data.estpos,
						   data.laser.val[j]+
						   (map->resolution),
						   nomove,
						   data.laser.angle[j] );
	    log2pic_imap_pos_from_vec2( abspt, map, &end );
	    map->maphit[end.x][end.y]++;
	    map->mapsum[end.x][end.y]++;
	  }
	} else {
	  if (data.laser.val[j] > max_range ) {
	    abspt = logtools_compute_laser_points( data.estpos,
						   max_range,
						   nomove,
						   data.laser.angle[j] );
	    log2pic_imap_pos_from_vec2( abspt, map, &end );
	  } else {
	    abspt = logtools_compute_laser_points( data.estpos,
						   data.laser.val[j]+
						   (map->resolution),
						   nomove,
						   data.laser.angle[j] );
	    log2pic_imap_pos_from_vec2( abspt, map, &end );
	  }
	  log2pic_imap_pos_from_rpos( data.estpos, map, &start );
	  //grid_line( start, end, &line );
	  fast_grid_line( start, end, &line );
	  for (i=0;i<line.numgrids;i++) {
	    x = line.grid[i].x;
	    y = line.grid[i].y;
	    if ( x>=0 && x<map->mapsize.x &&
		 y>=0 && y<map->mapsize.y ) {
	      if (data.laser.val[j]<=max_range ) {
		if (i>=line.numgrids-2) {
		  map->maphit[x][y]++;
		}
		map->mapsum[x][y]++;
	      } else {
		if (i<line.numgrids-1) {
		  map->mapsum[x][y]++;
		}
	      }
	    }
	  }
	}
      }
    }
  }
}

void
clear_map( logtools_grid_map2_t * map, logtools_rpos2_t pos )
{
  int x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
      map->calc[x][y]    = 0.0;
    }
  }
  map->offset     = pos;
}

void
log2pic_map_initialize( logtools_grid_map2_t *map, int sx, int sy, int center_x,
  int center_y, double zoom, double resolution, logtools_rpos2_t start )
{
  int x, y;

  map->mapsize.x  = sx;
  map->mapsize.y  = sy;
  map->resolution = resolution;
  map->zoom       = zoom;
  map->offset     = start;

  fprintf( stderr, "# INFO: allocating memory ... " );
  map->maphit   = mdalloc( 2, sizeof(float),  sx, sy );
  map->mapsum   = mdalloc( 2, sizeof(short),  sx, sy );
  map->mapprob  = mdalloc( 2, sizeof(float), sx, sy );
  map->calc     = mdalloc( 2, sizeof(float), sx, sy );
  fprintf( stderr, "done\n" );
  map->center.x = center_x;
  map->center.y = center_y;

  fprintf( stderr, "# INFO: map:            %d %d\n",
	   map->mapsize.x, map->mapsize.y );
  fprintf( stderr, "# INFO: center:         %.1f %.1f\n",
	   map->center.x, map->center.y );
  fprintf( stderr, "# INFO: resolution:      %.2f cm\n",
	   map->resolution );
  fprintf( stderr, "# INFO: real-size:      [%.2f %.2f] [%.2f %.2f] m\n",
	   -sx*map->resolution / 100.0, sx*map->resolution / 100.0,
	   -sy*map->resolution / 100.0, sy*map->resolution / 100.0 );

  for (x=0;x<sx;x++) {
    for (y=0;y<sy;y++) {
      map->mapprob[x][y] = -1;
      map->calc[x][y]    = 0.0;
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
    }
  }
}

void
printUnknown( FILE *fp, int n)
{
   while (n-- > 0)
      fprintf( fp, "-1 ");
}

void
log2pic_write_plot2d_data( logtools_log_data_t *rec )
{
  int       i, j;
  FILE    * ofp;

  if ((ofp = fopen( settings.outfilename, "w")) == 0){
    fprintf(stderr, "# ERROR: can't write data file %s\n",
	    settings.outfilename );
    return;
  }

  fprintf(stderr, "# INFO: write 2d data file %s\n",
	  settings.outfilename );
  for (i=0; i<rec->numlaserscans; i++) {
    if (rec->lsens[i].id==settings.laser_id) {
      for (j=0;j<rec->lsens[i].laser.numvalues;j++) {
	if (rec->lsens[i].laser.val[j]<settings.max_range) {
	  fprintf(ofp, "%f %f\n",
		  rec->lsens[i].coord[j].abspt.x,
		  rec->lsens[i].coord[j].abspt.y );
	}
      }
    }
  }
}

void
log2pic_write_plot3d_data( logtools_grid_map2_t *map )
{
  FILE    * ofp;
  int       x, y;

  if ((ofp = fopen( settings.outfilename, "w")) == 0){
    fprintf(stderr, "# ERROR: can't write data file %s\n",
	    settings.outfilename );
    return;
  }

  for (x = 0; x < map->mapsize.x; x++){
    for (y = 0; y < map->mapsize.y; y++) {
      if (map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] == 0)
	fprintf(ofp, "%d %d 0.5\n", x, y );
      else
	fprintf(ofp, "%d %d %.3f\n", x, y,
		1.0 * map->maphit[map->mapsize.x-x-1][map->mapsize.y-y-1] /
		(double) map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] );
    }
    fprintf(ofp, "\n" );
  }

  fclose(ofp);
}

double
rpos2_length( logtools_rpos2_t pos1, logtools_rpos2_t pos2 )
{
  return( sqrt( ( (pos1.x-pos2.x) * (pos1.x-pos2.x) ) +
		( (pos1.y-pos2.y) * (pos1.y-pos2.y) ) ) );
}


void
log2pic_map_compute_probs( logtools_grid_map2_t * map, double unknown_val )
{
  int     x, y, occ_cells, free_cells;
  double  odds, logodds, s_prob = 0.0, d_prob = 0.0;
  if (settings.use_odds_model) {
    s_prob  = log( settings.static_prob /(1.0-settings.static_prob) );
    d_prob  = log( settings.dynamic_prob /(1.0-settings.dynamic_prob) );
  }
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      if (map->mapsum[x][y]>0) {
	if (settings.use_odds_model) {
	  occ_cells  = map->maphit[x][y];
	  free_cells = map->mapsum[x][y]-map->maphit[x][y];
	  logodds = ( occ_cells * s_prob +  free_cells * d_prob );
	  odds = exp(logodds);
	  map->mapprob[x][y] = (odds / (1+odds));
	} else {
	  map->mapprob[x][y] =
	    ( settings.darken * map->maphit[x][y] /
	      (double) ( map->mapsum[x][y] ) );
	  if (map->mapprob[x][y] > 0.95 )
	    map->mapprob[x][y] = 0.95;
	}
      } else {
	if (settings.format==GRAPHICS) {
	  map->mapprob[x][y] = unknown_val;
	} else {
	  if (map->mapprob[x][y] < 0) {
	    map->mapprob[x][y] = unknown_val;
	  } else {
	    map->mapsum[x][y] = 1;
	  }
	}
      }
    }
  }
}

void
log2pic_compute_map( logtools_log_data_t rec, logtools_grid_map2_t * map )
{
  logtools_gauss_kernel_t     kernel;
  int              i, idx;
  for (i=0; i<rec.numentries; i++) {
    idx = rec.entry[i].index;
    if (rec.entry[i].type==LASER_VALUES) {
      if (rec.lsens[idx].id==settings.laser_id) {
	log2pic_map_integrate_scan( map, rec.lsens[idx], settings.max_range,
				    settings.usable_range );
      }
    }
  }
  if (settings.endpoints) {
    log2pic_map_compute_probs( map, 0.0 );
  } else {
    log2pic_map_compute_probs( map, settings.unknown_val );
  }

  if (settings.convolve) {
    kernel = logtools_compute_gauss_kernel( settings.kernel_size );
    log2pic_simple_convolve_map( map, kernel );
  }
}

void
log2pic_read_carmen_map( char * filename, logtools_grid_map2_t * map, double zoom )
{
  carmen_map_t      carmen_map;
  int               x, y;
  logtools_rpos2_t  nullpos = {0.0, 0.0, 0.0};
  char              description[MAX_STRING_LENGTH];
  char              username[MAX_STRING_LENGTH];
  char              origin[MAX_STRING_LENGTH];
  time_t            creation_time;

  fprintf( stderr, "# read carmen map %s ... ", filename );
  carmen_map_read_gridmap_chunk( filename, &carmen_map );
  carmen_map_read_creator_chunk( filename, &creation_time, username,
				 origin, description );
  fprintf( stderr, "done\n" );
  fprintf( stderr, "#####################################################################\n" );

  log2pic_map_initialize( map, carmen_map.config.x_size,
		  carmen_map.config.y_size, 0, 0, zoom,
		  carmen_map.config.resolution*100.0, nullpos );

  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->mapprob[x][y] = carmen_map.map[x][y];
    }
  }
}

void
log2pic_write_bee_map( logtools_grid_map2_t *map, int clip )
{
  FILE    * ofp;

  int       globalSizeX = 0, globalSizeY = 0;
  int       extendedSizeX = 0, extendedSizeY = 0;
  int       top = 0, bottom = 0;
  int       left = 0, right = 0, x, y;

  if ((ofp = fopen( settings.outfilename, "w")) == 0){
    fprintf(stderr, "# ERROR: can't write data map file %s\n",
	    settings.outfilename );
    return;
  }

  if (!clip) {
    if (map->mapsize.x < MINIMUM_MAP_SIZE)
      extendedSizeX = MINIMUM_MAP_SIZE;
    else
      extendedSizeX = ((map->mapsize.x / MAP_SIZE_STEP) + 1) * MAP_SIZE_STEP;

    if (map->mapsize.y < MINIMUM_MAP_SIZE)
      extendedSizeY = MINIMUM_MAP_SIZE;
    else
      extendedSizeY = ((map->mapsize.y / MAP_SIZE_STEP) + 1) * MAP_SIZE_STEP;

    top         = (extendedSizeY - map->mapsize.y) / 2;
    bottom      = extendedSizeY - top - map->mapsize.y;
    left        = (extendedSizeX - map->mapsize.x) / 2;
    right       = extendedSizeX - left - map->mapsize.x;

    globalSizeX = extendedSizeX * map->resolution;
    globalSizeY = extendedSizeY * map->resolution;

    fprintf( ofp, "robot_specifications->global_mapsize_x  %d\n", globalSizeY);
    fprintf( ofp, "robot_specifications->global_mapsize_y  %d\n", globalSizeX);
    fprintf( ofp, "robot_specifications->resolution %d\n", (int) map->resolution);

    fprintf( ofp, "global_map[0]: %d %d\n", extendedSizeY, extendedSizeX);

    for (x = 0; x < left; x++){
      printUnknown(ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
  } else {

    fprintf( ofp, "robot_specifications->global_mapsize_x  %d\n",
	     map->mapsize.y );
    fprintf( ofp, "robot_specifications->global_mapsize_y  %d\n",
	     map->mapsize.x) ;
    fprintf( ofp, "robot_specifications->resolution %d\n", (int) map->resolution);
    fprintf( ofp, "global_map[0]: %d %d\n", map->mapsize.y, map->mapsize.x );
  }

  for (x = 0; x < map->mapsize.x; x++){
    if (!clip)
      printUnknown( ofp, top);
    for (y = 0; y < map->mapsize.y; y++)
      if (map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] == 0)
	fprintf(ofp, "-1.000 ");
      else
	fprintf(ofp, "%.3f ",
		1.0-( map->maphit[map->mapsize.x-x-1][map->mapsize.y-y-1] /
		      (double) map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] ) );
    if (!clip)
      printUnknown( ofp, bottom);
    fprintf(ofp, "\n");
  }

  if (!clip) {
    for (x = 0; x < right; x++){
      printUnknown( ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
  }

  fclose(ofp);
}

void
log2pic_write_carmen_map( logtools_grid_map2_t * map )
{
  char                creator[MAX_STRING_LENGTH];
  char                comment[MAX_STRING_LENGTH];
  carmen_FILE*   fp;

  fp = carmen_fopen( settings.outfilename, "w");
  if(fp == NULL) {
    fprintf( stderr, "# Error: Could not open file %s for writing.",
	     settings.outfilename);
    exit(1);
  }
  fprintf( stderr, "# INFO: write carmen map file %s\n",
	   settings.outfilename);
  snprintf( creator, MAX_STRING_LENGTH,
	    "CARMEN map file converted from %s",
	    settings.infilename );
  if (fabs(settings.rotation_angle)>MIN_ROTATION) {
    snprintf( comment, MAX_STRING_LENGTH,
	      "[offset={%.4f,%.4f};rotation={%.4f,%.4f,%.4f}]",
	      (map->offset.x - map->center.x * map->resolution)/100.0,
	      (map->offset.y - map->center.y * map->resolution)/100.0,
	      settings.rotation_angle,
	      settings.rotation_center.x/100.0,
	      settings.rotation_center.y/100.0 );
  } else {
    snprintf( comment, MAX_STRING_LENGTH,
	      "[offset={%.4f,%.4f}]",
	      (map->offset.x - map->center.x * map->resolution)/100.0,
	      (map->offset.y - map->center.y * map->resolution)/100.0 );
  }
  carmen_map_write_all(
//			carmen_FILE* fp
                        fp,
//			float **prob
			map->mapprob,
//			int size_x
			map->mapsize.x,
//			int size_x
			map->mapsize.y,
//			double resolution
			map->resolution/100.0,
//			char *comment_origin,
			"",
//                      char *comment_description,
			"",
//                      char *creator_origin
			creator,
//                      char *creator_description
			comment,
//                      carmen_place_p places
			NULL,
//                      int num_places
			0,
//                      carmen_offlimits_p offlimits_list
			NULL,
//                      int offlimits_num_items
			0,
//                      carmen_laser_scan_p scan_list,
			NULL,
//                      int num_scans
			0 );
  carmen_fclose(fp);
}

void
log2pic_filetemplate_from_filename( char * filetemplate, char * filename )
{
  char    template[MAX_STRING_LENGTH];
  char  * ptr;

  strncpy( template, filename, MAX_STRING_LENGTH );
  ptr = rindex( template, '.' );
  if (ptr!=NULL) {
    *ptr = 0;
  }
  strncpy( filetemplate, template, MAX_STRING_LENGTH );
}

char *
log2pic_dump_filename( void )
{
  static char filename[MAX_STRING_LENGTH];
  static int ctr = 0;
  snprintf( filename, MAX_STRING_LENGTH, "%s-%s%s%s%s%d.png",
	    settings.filetemplate,
	    ctr<10000?"0":"",
	    ctr<1000?"0":"",
	    ctr<100?"0":"",
	    ctr<10?"0":"",
	    ctr );
  ctr++;
  return(filename);
}
