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

#include "readlog.h"
#include "writelog.h"

void
print_usage( void )
{
  fprintf( stderr, "usage: vasco-tiny [options] <carmen-log-file>\n" );
   fprintf( stderr, "  Options:  -f, --flaser      : use only the old FLASER messages\n" );
   fprintf( stderr, "  Options:  -r, --robotlaser  : use only the new  ROBOTLASER1 messages\n" );
}

double
rad2deg(double x)
{
  return x * 57.29577951308232087679;
}


int
main( int argc, char *argv[] )
{
  carmen_point_t                  pos, corrpos, odo;
  double                          time;
  int                             i;
  int                             scancnt;
  carmen_move_t                   pos_move = {0.0, 0.0, 0.0};
  carmen_move_t                   corr_move = {0.0, 0.0, 0.0};
  carmen_point_t                  old_pos = {0.0, 0.0, 0.0};
  carmen_point_t                  old_corrpos = {0.0, 0.0, 0.0};
  int which_laser = 0;

  carmen_FILE stdout_carmen;
  stdout_carmen.compressed = 0;
  stdout_carmen.fp = stdout;

  if (argc < 2) {
    print_usage();
    exit(1);
  }

  scancnt=0;
  for (i=1; i < argc-1; i++) {
    if (!strcmp(argv[i], "-f")  || !strcmp(argv[i], "--flaser"))
      which_laser = 1;
    if (!strcmp(argv[i], "-r")  || !strcmp(argv[i], "--robotlaser"))
      which_laser = 2;
  }

  carmen_ipc_initialize(argc, argv);
  vascocore_init(argc, argv);

  /*   file handler */
  carmen_FILE *logfile = NULL;

  /*   log file index structure */
  carmen_logfile_index_p logfile_index = NULL;

  /*  open logfile for reading */
  logfile = carmen_fopen(argv[argc-1], "r");
  if(logfile == NULL)
    carmen_die("Error: could not open file %s for reading.\n", argv[1]);

  /* create index structure */
  logfile_index = carmen_logfile_index_messages(logfile);

  /*   buffer for reading the lines */
  const int max_line_length=9999;
  char line[max_line_length+1];

  /*   used to read the messages. erase stucture before! */
  carmen_robot_laser_message l;
  carmen_erase_structure(&l, sizeof(carmen_robot_laser_message) );

  /*   iterate over all entries in the logfile */
  while (!carmen_logfile_eof(logfile_index)) {

    int correct_msg = 1;
    /*     read the line from the file */
    carmen_logfile_read_next_line(logfile_index,
				  logfile,
				  max_line_length,
				  line);

    /*     what kind of message it this? */
    if (which_laser != 1 && strncmp(line, "ROBOTLASER1 ", 12) == 0) {
      /*  convert the string using the corresponding read function */
      char* next = carmen_string_to_robot_laser_message(line, &l);
      time = atof( strtok( next, " ") );

    }
    else if (which_laser != 2 && strncmp(line, "FLASER ", 7) == 0) {
      /*  convert the string using the corresponding read function */
      char* next = carmen_string_to_robot_laser_message_orig(line, &l);
      time = atof( strtok( next, " ") );
    }
    else {
      correct_msg = 0;
      fputs(line, stdout);
    }

    if (correct_msg) {
      odo = l.robot_pose;
      pos = l.laser_pose;

      carmen_laser_laser_message scan;
      carmen_erase_structure(&scan,  sizeof(carmen_laser_laser_message) );
      scan.timestamp      = l.timestamp;
      scan.config         = l.config;
      scan.num_readings   = l.num_readings;
      scan.num_remissions = l.num_remissions;
      scan.range          = l.range;
      scan.remission      = l.remission;
      scan.host           = l.host;
      corrpos = vascocore_scan_match( scan, pos );
      scancnt++;
      corr_move = carmen_move_between_points( old_corrpos, corrpos );
      pos_move = carmen_move_between_points( old_pos, pos );

      fprintf( stderr, "expected movement  %.6f m %.6f m %.6f degree\n",
	       corr_move.forward, corr_move.sideward,
	       rad2deg(corr_move.rotation) );
      fprintf( stderr, "corrected movement %.6f m %.6f m %.6f degree\n\n",
	       pos_move.forward, pos_move.sideward,
	       rad2deg(pos_move.rotation) );

      l.laser_pose = corrpos;

      carmen_logwrite_write_robot_laser(&l,1,&stdout_carmen, time);
      fflush(stdout);

      old_pos     = pos;
      old_corrpos = corrpos;
    }

  }

  /* close the file */
  carmen_fclose(logfile);

  fprintf(stderr, "\ndone, corrected %d messages!\n", scancnt);

  /* free index structure */
  carmen_logfile_free_index(&logfile_index);

  return(0);

}
