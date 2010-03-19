#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "log2pic.h"

void
print_usage( void )
{
  fprintf(stderr,
	  "\nusage: log2pic [options] <LOG-FILE> <PIC-FILE>\n"
	  "  -anim-step <STEP-SIZE>:    min distance between scans (in m)\n"
	  "  -anim-skip <NUM>:          skip NUM animation dumps (0: no skip)\n"
	  "  -animation:                write sep. pics for animation\n"
	  "  -background <FILE>:        use background file\n"
	  "  -bee-map:                  save in bee-map format\n"
	  "  -bg-offset <X><Y>:         shift by <X>/<Y> pixels\n"
	  "  -bg-color <COLOR>:         background color\n"
	  "  -bg-map <FILE>:            read carmen-map as background image\n"
	  "  -border <BORDER>:          add a border (in m)\n"
 	  "  -carmen-map:               save in carmen-map format\n"
	  "  -convolve:                 convolve map with gaussian kerne;\n"
	  "  -crop <X><Y><X><Y>:        crop part of the map (min,max):\n"
	  "  -darken <FACTOR>:          darken the occ. cells\n"
	  "  -display-arrow:            display arrow in marking\n"
	  "  -endpoints:                use endpoints instead of beams\n"
	  "  -free-prob:                probability for free observation\n"
	  "  -from <NUM>:               start animation with scan NUM\n"
	  "  -gps-path:                 draw gps points\n"
	  "  -id <ID>:                  set laser number\n"
	  "  -kernel-size <NUM>:        size of the gaussian kernel (>0 and odd)\n"
	  "  -maxrange <MAX-RANGE>:     max range for building maps\n"
	  "  -no-scans:                 don't integrate the scans\n"
	  "  -odds-model:               use odds-model to compute probs\n"
	  "  -pathcolor <COLORNAME>:    color of the robot path\n"
	  "  -pathwidth <WIDTH>:        width of the robot path\n"
	  "  -start-pose <X><Y><O>:     start pose of the robot (in m, m, deg)\n"
	  "  -plot2d:                   save as 2d data file\n"
	  "  -plot3d:                   save map as 3d data file\n"
	  "  -pos-start <X><Y>:         pos of lower left bg-corner\n"
	  "  -rear-laser:               use rear laser instead of front laser\n"
	  "  -res   <RES> <RES>:        resolution of the map\n"
	  "  -res-x <RES>:              resolution in x direction\n"
	  "  -res-y <RES>:              resolution in y direction\n"
	  "  -rotate <ANGLE>:           rotate the map by ANGLE degree\n"
	  "  -showpath:                 show robot path\n"
	  "  -size:                     set the size of the output image\n"
	  "  -static-prob:              probability for static observation\n"
	  "  -to <NUM>:                 end animation with scan NUM\n"
	  "  -usablerange <MAX-RANGE>:  max range for detecting corrupted beams\n"
	  "  -utm-correct:              corrects gps positions for UTM tiles\n"
	  "  -zoom <ZOOM>:              scale factor for the map (must be >=1.0)\n" );
}

int
main( int argc, char** argv)
{
  logtools_grid_map2_t           map;
  char                           mapfilename[MAX_STRING_LENGTH];
  char                           bgfilename[MAX_STRING_LENGTH];
  logtools_log_data_t            rec;
  int                            i, j, idx;
  logtools_bounding_box_t        bbox;
  logtools_vector2_t             size;
  logtools_ivector2_t            isize = {0,0}, istart = {0,0};
  int                            readmap = FALSE;
  int                            readbg = FALSE;
  int                            numctr = 0;
  logtools_rpos2_t               npos = {0.0, 0.0, 0.0};
  logtools_rpos2_t               rpos = {0.0, 0.0, 0.0};
  logtools_rmove2_t              move;
  double                         res;

  if (argc<3) {
    print_usage();
    exit(0);
  }

  for (i=1; i<argc-2; i++) {
    if (!strcmp(argv[i],"-showpath")) {
      settings.showpath = TRUE;
    } else if (!strcmp(argv[i],"-carmen-map")) {
      settings.format = CARMEN_MAP;
    } else if (!strcmp(argv[i],"-bee-map")) {
      settings.format = BEE_MAP;
    } else if (!strcmp(argv[i],"-res") && (argc>i+2)) {
      settings.resolution_x = 100.0 * atof(argv[++i]);
      settings.resolution_y = settings.resolution_x;
    } else if (!strcmp(argv[i],"-res-x") && (argc>i+2)) {
      settings.resolution_x = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-res-y") && (argc>i+2)) {
      settings.resolution_y = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-zoom") && (argc>i+2)) {
      settings.zoom = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-pathcolor") && (argc>i+2)) {
      strncpy( settings.pathcolor, argv[++i], MAX_STRING_LENGTH );
    } else if (!strcmp(argv[i],"-pathwidth") && (argc>i+2)) {
      settings.pathwidth = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-border") && (argc>i+1)) {
      settings.border = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-bg-color") && (argc>i+1)) {
      strncpy( settings.bgcolor, argv[++i], MAX_STRING_LENGTH );
    } else if (!strcmp(argv[i],"-bg-map") && (argc>i+1)) {
      strncpy( mapfilename, argv[++i], MAX_STRING_LENGTH );
      readmap = TRUE;
    } else if (!strcmp(argv[i],"-id") && (argc>i+1)) {
      settings.laser_id = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-rear-laser")) {
      settings.flip = TRUE;
    } else if (!strcmp(argv[i],"-plot2d")) {
      settings.format = PLOT2D;
    } else if (!strcmp(argv[i],"-plot3d")) {
      settings.format = PLOT3D;
    } else if (!strcmp(argv[i],"-gps-path")) {
      settings.gpspath = TRUE;
    } else if (!strcmp(argv[i],"-utm-correct")) {
      settings.utm_correct = TRUE;
    } else if (!strcmp(argv[i],"-google-correct") && (argc>i+2)) {
      settings.google_correct = TRUE;
      settings.google_zoom = atoi( argv[++i] );
    } else if (!strcmp(argv[i],"-background") && (argc>i+1)) {
      strncpy( bgfilename, argv[++i], MAX_STRING_LENGTH );
      readbg = TRUE;
    } else if (!strcmp(argv[i],"-bg-offset") && (argc>i+2)) {
      settings.bgoffset.x = atof( argv[++i] );
      settings.bgoffset.y = atof( argv[++i] );
    } else if (!strcmp(argv[i],"-pos-start") && (argc>i+2)) {
      settings.posstart.x = 100.0 * atof( argv[++i] );
      settings.posstart.y = 100.0 * atof( argv[++i] );
    } else if (!strcmp(argv[i],"-maxrange") && (argc>i+1)) {
      settings.max_range = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-rotate") && (argc>i+1)) {
      settings.rotation_angle = deg2rad(atof(argv[++i]));
    } else if (!strcmp(argv[i],"-display-arrow")) {
      settings.display_arrow = TRUE;
    } else if (!strcmp(argv[i],"-animation")) {
      settings.animation = TRUE;
    } else if (!strcmp(argv[i],"-anim-step") && (argc>i+1)) {
      settings.anim_step = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-anim-skip") && (argc>i+1)) {
      settings.anim_skip = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-usablerange") && (argc>i+1)) {
      settings.usable_range = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-darken") && (argc>i+1)) {
      settings.darken = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-endpoints")) {
      settings.endpoints = TRUE;
    } else if (!strcmp(argv[i],"-convolve")) {
      settings.convolve = TRUE;
    } else if (!strcmp(argv[i],"-kernel-size") && (argc>i+1)) {
      settings.kernel_size = atoi(argv[++i]);
      if (settings.kernel_size<1 || settings.kernel_size%2 != 1) {
	print_usage();
	exit(1);
      }
    } else if (!strcmp(argv[i],"-odds-model")) {
      settings.use_odds_model = TRUE;
    } else if (!strcmp(argv[i],"-static-prob") && (argc>i+1)) {
      settings.static_prob = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-free-prob") && (argc>i+1)) {
      settings.dynamic_prob = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-from") && (argc>i+1)) {
      settings.from = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-to") && (argc>i+1)) {
      settings.to = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-no-scans")) {
      settings.integrate_scans = FALSE;
    } else if (!strcmp(argv[i],"-crop") && (argc>i+4)) {
      settings.min_x = atof(argv[++i]);
      settings.min_y = atof(argv[++i]);
      settings.max_x = atof(argv[++i]);
      settings.max_y = atof(argv[++i]);
      settings.crop_size = TRUE;
    } else if (!strcmp(argv[i],"-size") && (argc>i+2)) {
      isize.x = atoi(argv[++i]);
      isize.y = atoi(argv[++i]);
      settings.set_size = TRUE;
    } else if (!strcmp(argv[i],"-pos") && (argc>i+3)) {
      settings.pos_x = 100.0 * atof(argv[++i]);
      settings.pos_y = 100.0 * atof(argv[++i]);
      settings.pos_o = deg2rad(atof(argv[++i]));
    } else {
      print_usage();
      exit(1);
    }
  }

  if (settings.zoom<1.0) {
    print_usage();
    exit(1);
  }

  fprintf( stderr, "\n");

  if (settings.use_odds_model) {
    if (settings.dynamic_prob>0.5) {
      fprintf( stderr, "# WARNING: dynamic-prob should not be > 0.5\n" );
    }
    if (settings.static_prob<0.5) {
      fprintf( stderr, "# WARNING: static-prob should not be < 0.5\n" );
    }
  }

  /** Warnings to avoid m/cm convertion errors  **********************************************/

  if (settings.max_range < 100.0)
    carmen_warn("# WARNUNG: max_range is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.max_range);

  if (settings.usable_range < 100.0)
    carmen_warn("# WARNUNG: usable_range is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.usable_range);

  if (settings.resolution_x < 1.0)
    carmen_warn("# WARNUNG: resolution_x is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.resolution_x);

  if (settings.resolution_y < 1.0)
    carmen_warn("# WARNUNG: resolution_y is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.resolution_y);

  if (settings.max_range >= 100000.0)
    carmen_warn("# WARNUNG: max_range is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.max_range);

  if (settings.usable_range >= 100000.0)
    carmen_warn("# WARNUNG: usable_range is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.usable_range);

  if (settings.resolution_x >= 1000.0)
    carmen_warn("# WARNUNG: resolution_x is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.resolution_x);

  if (settings.resolution_y >= 1000.0)
    carmen_warn("# WARNUNG: resolution_y is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.resolution_y);

  /*******************************************************************************************/

  strncpy( settings.infilename, argv[argc-2], MAX_STRING_LENGTH );
  strncpy( settings.outfilename, argv[argc-1], MAX_STRING_LENGTH );

  if (settings.format==CARMEN_MAP || settings.format==BEE_MAP) {
    settings.unknown_val = CARMEN_MAP_STD_VAL;
  }

  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#              READ FILE\n" );
  fprintf( stderr, "#\n" );

  if (!logtools_read_logfile( &rec, settings.infilename ))
      exit(1);

  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );

  if (fabs(settings.rotation_angle)>MIN_ROTATION) {
    if (rec.info.system==CARMEN) {
      if (rec.numlaserscans>=1) {
	if (settings.set_pos) {
	  rpos.x = settings.pos_x;
	  rpos.y = settings.pos_y;
	  rpos.o = settings.pos_o;
	} else {
	  rpos = rec.lsens[0].estpos;
	}
	settings.rotation_center.x = rpos.x;
	settings.rotation_center.y = rpos.y;
        rec.lsens[0].estpos.o += settings.rotation_angle;
	for (i=1; i<rec.numlaserscans; i++) {
	  move = logtools_movement2_between_rpos2( rpos, rec.lsens[i].estpos );
	  rpos = rec.lsens[i].estpos;
	  rec.lsens[i].estpos =
	    logtools_rpos2_with_movement2( rec.lsens[i-1].estpos, move );
	}
      }
    } else {
       if (rec.numpositions>=1) {
	 if (settings.set_pos) {
	   rpos.x = settings.pos_x;
	   rpos.y = settings.pos_y;
	   rpos.o = settings.pos_o;
	 } else {
	   rpos = rec.psens[0].rpos;
	 }
	 settings.rotation_center.x = rpos.x;
	 settings.rotation_center.y = rpos.y;
         rec.psens[0].rpos.o += settings.rotation_angle;
         npos = rec.psens[0].rpos;
	 for (i=0; i<rec.numentries; i++) {
	   idx = rec.entry[i].index;
	   if (rec.entry[i].type==POSITION) {
	     if (idx>0) {
	       move = logtools_movement2_between_rpos2( rpos,
						       rec.psens[idx].rpos );
	       rpos = rec.psens[idx].rpos;
	       rec.psens[idx].rpos =
		 logtools_rpos2_with_movement2( rec.psens[idx-1].rpos, move );
	       npos = rec.psens[idx].rpos;
	     }
	   } else if (rec.entry[i].type==LASER_VALUES) {
	     rec.lsens[idx].estpos = npos;
	   }
	 }
      }
    }
  }

  if (settings.flip) {
    for (i=0; i<rec.numlaserscans; i++) {
      for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
	rec.lsens[i].laser.angle[j] += M_PI;
      }
    }
  }

  /* compute abs values */
  logtools_compute_coordpts( &rec );

  if (settings.crop_size) {
    bbox.min.x = settings.min_x;
    bbox.min.y = settings.min_y;
    bbox.max.x = settings.max_x;
    bbox.max.y = settings.max_y;
  } else {
    bbox.min.x = MAXFLOAT;    bbox.min.y = MAXFLOAT;
    bbox.max.x = -MAXFLOAT;   bbox.max.y = -MAXFLOAT;
    /* compute bounding box */
    if (settings.gpspath) {
      for (j=0;j<rec.numgps;j++) {
	if (fabs(rec.gps[j].latitude) > 0.1 &&
	    fabs(rec.gps[j].longitude) > 0.1) {
	  if (rec.gps[j].latitude < bbox.min.x) {
	    bbox.min.x = rec.gps[j].latitude;
	  }
	  if (rec.gps[j].longitude < bbox.min.y) {
	    bbox.min.y = rec.gps[j].longitude;
	  }
	  if (rec.gps[j].latitude > bbox.max.x) {
	    bbox.max.x = rec.gps[j].latitude;
	  }
	  if (rec.gps[j].longitude > bbox.max.y) {
	    bbox.max.y = rec.gps[j].longitude;
	  }
	}
      }
      fprintf( stderr, "#              GPS DATA \n" );
      fprintf( stderr, "# INFO: min x / max x    = %.8f deg / %.8f deg\n",
	       bbox.min.x, bbox.max.x );
      fprintf( stderr, "# INFO: min y / max y    = %.8f deg / %.8f deg\n",
	       bbox.min.y, bbox.max.y );
      fprintf( stderr, "#####################################################################\n" );
    } else {
      for (i=0; i<rec.numlaserscans; i++) {
	if (rec.lsens[i].id==settings.laser_id) {
	  numctr++;
	  for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
	    if (rec.lsens[i].laser.val[j]<settings.max_range) {
	      if (rec.lsens[i].coord[j].abspt.x < bbox.min.x) {
		bbox.min.x = rec.lsens[i].coord[j].abspt.x;
	      }
	      if (rec.lsens[i].coord[j].abspt.x > bbox.max.x) {
		bbox.max.x = rec.lsens[i].coord[j].abspt.x;
	      }
	      if (rec.lsens[i].coord[j].abspt.y < bbox.min.y) {
		bbox.min.y = rec.lsens[i].coord[j].abspt.y;
	      }
	      if (rec.lsens[i].coord[j].abspt.y > bbox.max.y) {
		bbox.max.y = rec.lsens[i].coord[j].abspt.y;
	      }
	    }
	  }
	}
      }
    }
  }

  if (readmap) {

    log2pic_read_carmen_map( mapfilename, &map, settings.zoom );
    if ( map.center.x < bbox.min.x ) {
      bbox.min.x = map.center.x;
    }
    if ( map.center.x + ( map.mapsize.x * map.resolution ) > bbox.max.x ) {
      bbox.max.x = map.center.x + ( map.mapsize.x * map.resolution );
    }
    if ( map.center.y < bbox.min.y ) {
      bbox.min.y = map.center.y;
    }
    if ( map.center.y + ( map.mapsize.y * map.resolution ) > bbox.max.y ) {
      bbox.max.y = map.center.y + ( map.mapsize.y * map.resolution );
    }

  } else if (readbg) {

    settings.bgfile = TRUE;
    fprintf( stderr, "#\n" );
    fprintf( stderr, "# INFO: read background image %s ... \n", bgfilename );
    log2pic_read_image_file( bgfilename, &(settings.background) );
    fprintf( stderr, "#\n" );
    fprintf( stderr, "# INFO: size of image %d x %d\n",
	     settings.background.width,
	     settings.background.height );
    fprintf( stderr, "# INFO: resolution %f x %f\n",
	     settings.resolution_x,
	     settings.resolution_y );
    bbox.min.x = settings.background.start.x =
      -settings.bgoffset.x * settings.resolution_x;
    bbox.max.x = settings.background.start.x +
      ( settings.background.width * settings.resolution_x );
    bbox.min.y = settings.background.start.y =
      -settings.bgoffset.y * settings.resolution_y;
    bbox.max.y = settings.background.start.y +
      ( settings.background.height * settings.resolution_y );

  } else if (settings.set_size) {

    bbox.min.x = settings.background.start.x =
      -settings.bgoffset.x * settings.resolution_x;
    bbox.max.x = settings.background.start.x +
      ( settings.background.width * settings.resolution_x );
    bbox.min.y = settings.background.start.y =
      -settings.bgoffset.y * settings.resolution_y;
    bbox.max.y = settings.background.start.y +
      ( settings.background.height * settings.resolution_y );

  } else {

    if (numctr==0 && !(settings.crop_size)) {
      /* no laser and no map read */
      if (!settings.showpath || settings.format!=GRAPHICS) {
	fprintf( stderr, "# ERROR: found %d laser scans!\n\n", numctr );
	exit(1);
      } else {
	if (rec.info.system==CARMEN) {
	  for (i=0; i<rec.numlaserscans; i++) {
	    if (rec.lsens[i].estpos.x < bbox.min.x) {
	      bbox.min.x = rec.lsens[i].estpos.x;
	    }
	    if (rec.lsens[i].estpos.y < bbox.min.y) {
	      bbox.min.y = rec.lsens[i].estpos.y;
	    }
	    if (rec.lsens[i].estpos.x > bbox.max.x) {
	      bbox.max.x = rec.lsens[i].estpos.x;
	    }
	    if (rec.lsens[i].estpos.y > bbox.max.y) {
	      bbox.max.y = rec.lsens[i].estpos.y;
	    }
	  }
	} else if (rec.info.system==REC) {
	  for (i=0; i<rec.numpositions; i++) {
	    if (rec.psens[i].rpos.x < bbox.min.x) {
	      bbox.min.x = rec.psens[i].rpos.x;
	    }
	    if (rec.psens[i].rpos.y < bbox.min.y) {
	      bbox.min.y = rec.psens[i].rpos.y;
	    }
	    if (rec.psens[i].rpos.x > bbox.max.x) {
	      bbox.max.x = rec.psens[i].rpos.x;
	    }
	    if (rec.psens[i].rpos.y > bbox.max.y) {
	      bbox.max.y = rec.psens[i].rpos.y;
	    }
	  }
	}
      }
    } else {
      fprintf( stderr, "# INFO: using %d laser scans\n", numctr );
    }

  }

  if (!settings.set_size) {
    size.x = bbox.max.x-bbox.min.x+2*settings.border;
    size.y = bbox.max.y-bbox.min.y+2*settings.border;
    isize.x = (int) (size.x/settings.resolution_x);
    isize.y = (int) (size.y/settings.resolution_y);
  } else {
    size.x = isize.x * settings.resolution_x;
    size.y = isize.y * settings.resolution_y;
    settings.background.width = isize.x;
    settings.background.height = isize.y;
  }



  istart.x = (int) ((bbox.min.x-settings.border)/settings.resolution_x);
  istart.y = (int) ((bbox.min.y-settings.border)/settings.resolution_y);

  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#              DATA FILE\n" );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "# INFO: min x / max x    = %.2f m / %.2f m\n",
	   bbox.min.x / 100.0, bbox.max.x / 100.0 );
  fprintf( stderr, "# INFO: min y / max y    = %.2f m / %.2f m\n",
	   bbox.min.y / 100.0, bbox.max.y / 100.0 );
  fprintf( stderr, "# INFO: size x / y       = %.2f m x %.2f m\n",
	   size.x / 100.0, size.y / 100.0 );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#              MAP SIZE\n" );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "# INFO: resolution-x       = %.1f cm per pixel\n",
	   settings.resolution_x );
  fprintf( stderr, "# INFO: resolution-y       = %.1f cm per pixel\n",
	   settings.resolution_y );
  fprintf( stderr, "# INFO: size x / y       = %d pixel x %d pixel\n",
	   (int) (settings.zoom * isize.x), (int) (settings.zoom * isize.y) );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#\n" );

  res = (settings.resolution_x+settings.resolution_y)/2.0;
  map_initialize( &map, isize.x, isize.y, -istart.x, -istart.y, settings.zoom,
		  res, settings.posstart );

  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );

  switch (settings.format) {
  case PLOT2D:
    log2pic_write_plot2d_data( &rec );
    break;
  case PLOT3D:
    log2pic_compute_map( rec, &map );
    log2pic_write_plot3d_data( &map );
    break;
  case GRAPHICS:
    log2pic_write_image_magick_map( &map, &rec );
    break;
  case CARMEN_MAP:
    log2pic_compute_map( rec, &map );
    log2pic_write_carmen_map( &map );
    break;
  case BEE_MAP:
    log2pic_compute_map( rec, &map );
    log2pic_write_bee_map( &map, 1 );
    break;
  }

  fprintf( stderr, "#####################################################################\n" );
  exit(0);

}

