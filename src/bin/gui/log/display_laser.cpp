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

#include <qapplication.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "global.h"

#ifdef __cplusplus
}
#endif

#include "laserdisplay.h"

void
print_usage( void )
{
  fprintf( stderr, "usage: display-laserdata [-num LASER_ID] <FILE>\n");
}

void
check_mode( QLaserDisplay *w )
{
  switch(w->mode) {
  case MODE_START:
    w->recpos = 0;
    w->mode   = MODE_STOP;
    w->repaint();
    break;
  case MODE_PLAY:
    if (w->recpos<w->rec.numlaserscans-1)
      (w->recpos)++;
    else
      w->mode = MODE_STOP;
    w->repaint();
    break;
  case MODE_REVPLAY:
    if (w->recpos>0)
      (w->recpos)--;
    else
      w->mode = MODE_STOP;
    w->repaint();
    break;
  case MODE_END:
    w->recpos = w->rec.numlaserscans-1;
    w->mode   = MODE_STOP;
    w->repaint();
    break;
  }
}

int
main( int argc, char *argv[] )
{
  QApplication         app( argc, argv );
  QLaserDisplay        window;
  int                  i, cnt=1;

  for (i=1; i<argc-1; i++) {
    if (!strcmp(argv[i],"-num") && (argc>i+1)) {
      window.LASER_ID = atoi( argv[++i] );
    } else {
      print_usage();
      exit(1);
    }
  }

  if (!logtools_read_logfile( &window.rec, argv[argc-1] ))
    exit(0);

  while (window.rec.lsens[window.recpos].id!=window.LASER_ID) {
    window.recpos++;
    if (window.recpos>=window.rec.numlaserscans) {
      fprintf( stderr, "# ERROR: no laser scans with id %d !!!\n",
	       window.LASER_ID );
      exit(0);
    }
  }

  if (window.rec.numlaserscans>1)
    window.slider->setMaxValue(window.rec.numlaserscans-2);
  else
    window.slider->setMaxValue(1);

  window.show();

  while(1) {
    if (cnt%window.wait==0) {
      check_mode( &window );
      cnt=1;
    }
    cnt++;
    app.processEvents();
    usleep(1);
  }

  return(0);
}

