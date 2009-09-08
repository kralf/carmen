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
#include "readlog.h"

off_t carmen_logfile_uncompressed_length(carmen_FILE *infile)
{
  unsigned char buffer[10000];
  long int log_bytes = 0;
  int nread;
  struct stat stat_buf;

  if(!infile->compressed) {
    /* compute total length of logfile */
    carmen_fseek(infile, 0L, SEEK_SET);
    log_bytes = 0;
    do {
      nread = carmen_fread(buffer, 1, 10000, infile);
      log_bytes += nread;
    } while(nread > 0);
    carmen_fseek(infile, 0L, SEEK_SET);
    return log_bytes;
  } 
  else {
    /* report compressed size for compressed files */
    fstat(fileno(infile->fp), &stat_buf);
    return stat_buf.st_size;
  }
}

/** 
 * Builds the index structure used for parsing a carmen log file. 
 **/
carmen_logfile_index_p carmen_logfile_index_messages(carmen_FILE *infile)
{
  carmen_logfile_index_p index;
  int i, found_linebreak = 1, nread, max_messages;
  off_t file_length = 0, file_position = 0, total_bytes, read_count = 0;

  unsigned char buffer[10000];

  /* allocate and initialize an index */
  index = (carmen_logfile_index_p)calloc(1, sizeof(carmen_logfile_index_t));
  carmen_test_alloc(index);

  /* compute the total length of the uncompressed logfile. */
  fprintf(stderr, "\n\rIndexing messages (0%%)    ");
  file_length = carmen_logfile_uncompressed_length(infile);

  /* mark the start of all messages */
  index->num_messages = 0;
  max_messages = 10000;
  index->offset = (off_t*)calloc(max_messages, sizeof(off_t));
  carmen_test_alloc(index->offset);

  carmen_fseek(infile, 0L, SEEK_SET);

  total_bytes = 0;
  do {
    nread = carmen_fread(buffer, 1, 10000, infile);
    read_count++;
    if(read_count % 1000 == 0) {
      if(!infile->compressed)
	file_position = total_bytes + nread;
      else
	file_position = lseek(fileno(infile->fp), 0, SEEK_CUR);
      fprintf(stderr, "\rIndexing messages (%.0f%%)      ", 
	      ((float)file_position) / file_length * 100.0);
    }

    if(nread > 0) {
      for(i = 0; i < nread; i++) {
        if(found_linebreak && buffer[i] != '\r') {
          found_linebreak = 0;
	  if(index->num_messages == max_messages) {
	    max_messages += 10000;
	    index->offset = (off_t*)realloc(index->offset, max_messages *
						sizeof(off_t));
	    carmen_test_alloc(index->offset);
	  }
	  index->offset[index->num_messages] = total_bytes + i;
	  index->num_messages++;
        }
        if(buffer[i] == '\n')
          found_linebreak = 1;
      }
      total_bytes += nread;
    }
  } while(nread > 0);

  // set file size as last offset
  // offset array now contains one element more than messages
  // required by carmen_logfile_read_line to read the last line
  if(index->num_messages == max_messages) {
    max_messages += 1;
    index->offset = (off_t*)realloc(index->offset, max_messages * sizeof(off_t));
    carmen_test_alloc(index->offset);
  }
  index->offset[index->num_messages] = total_bytes;

  fprintf(stderr, "\rIndexing messages (100%%) - %d messages found.      \n",
	  index->num_messages);
  carmen_fseek(infile, 0L, SEEK_SET);
  index->current_position = 0;
  return index;
}

void carmen_logfile_free_index(carmen_logfile_index_p* pindex) {
  if (pindex == NULL) 
    return;

  if ( (*pindex) == NULL) 
    return;
    
  if ( (*pindex)->offset != NULL) {
    free( (*pindex)->offset);
  }
  free(*pindex);
  (*pindex) = NULL;
}

int carmen_logfile_eof(carmen_logfile_index_p index)
{
  if(index->current_position > index->num_messages - 1)
    return 1;
  else
    return 0;
}

float carmen_logfile_percent_read(carmen_logfile_index_p index)
{
  return index->current_position / (float)index->num_messages;
}

int carmen_logfile_read_line(carmen_logfile_index_p index, carmen_FILE *infile,
			     int message_num, int max_line_length, char *line)
{
  size_t nread;
  
  /* are we moving sequentially through the logfile?  If not, fseek */
  if(message_num != index->current_position) {
    index->current_position = message_num;
    carmen_fseek(infile, index->offset[index->current_position], SEEK_SET);
  }

  /* check maximum line length */
  if(index->offset[index->current_position + 1] - 
     index->offset[index->current_position] >= max_line_length)
    carmen_die("Error: exceed maximum line length.\n");

  /* read the line of the logfile */
  nread = carmen_fread(line, 1, index->offset[index->current_position + 1] - 
		       index->offset[index->current_position], infile);
  line[nread] = '\0';
  index->current_position++;
  return nread;
}

int carmen_logfile_read_next_line(carmen_logfile_index_p index, carmen_FILE *infile,
				  int max_line_length, char *line)
{
  return carmen_logfile_read_line(index, infile, 
				  index->current_position, 
				  max_line_length, line);
}


int first_wordlength(char *str)
{
  char* c_enter = strchr(str, '\n'); // check also for newline
  char* c = strchr(str, ' ');

  if (c_enter == NULL && c == NULL) // it is the last word in the string
    return strlen(str); 

  if (c_enter != NULL && c == NULL) // there is no space but a newline
    return c_enter - str;

  if (c_enter == NULL && c != NULL) // there is a space but no newline
    return c - str;

  if (c_enter < c )    // use whatever comes first
    return c_enter - str;
  else
    return c - str;
}

void copy_host_string(char **host, char **string)
{
  int l;
  while(*string[0] == ' ')
    *string += 1;                           /* advance past spaces */
  l = first_wordlength(*string);
  if(*host != NULL)
    free(*host);
  *host = (char *)calloc(1, l+1);     /* allocate one extra char for the \0 */
  carmen_test_alloc(*host);
  strncpy(*host, *string, l);
  (*host)[l] = '\0';
  *string += l;
}

#define CLF_READ_DOUBLE(str) strtod(*(str), (str))
#define CLF_READ_INT(str) (int)strtol(*(str), (str), 10)
#define CLF_READ_CHAR(str) (char) ( ( (*str)++)[0] )

char *carmen_string_to_base_odometry_message(char *string,
					     carmen_base_odometry_message
					     *odometry)
{
  char *current_pos = string;

  if (strncmp(current_pos, "ODOM ", 5) == 0)
    current_pos = carmen_next_word(current_pos); 
  
  odometry->x = CLF_READ_DOUBLE(&current_pos);
  odometry->y = CLF_READ_DOUBLE(&current_pos);
  odometry->theta = CLF_READ_DOUBLE(&current_pos);
  odometry->tv = CLF_READ_DOUBLE(&current_pos);
  odometry->rv = CLF_READ_DOUBLE(&current_pos);
  odometry->acceleration = CLF_READ_DOUBLE(&current_pos);
  odometry->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&odometry->host, &current_pos);
  return current_pos;
}

char *carmen_string_to_arm_state_message(char *string,
					 carmen_arm_state_message *arm)
{
  int i;
  char *current_pos = string;

  if (strncmp(current_pos, "ARM ", 4) == 0)
    current_pos = carmen_next_word(current_pos); 
  
  arm->flags = CLF_READ_INT(&current_pos);
  arm->num_joints = CLF_READ_INT(&current_pos);
  arm->joint_angles = (double *) realloc(arm->joint_angles, arm->num_joints*sizeof(double));
  carmen_test_alloc(arm->joint_angles);
  for (i = 0; i < arm->num_joints; i++)
    arm->joint_angles[i] = CLF_READ_DOUBLE(&current_pos);
  arm->num_currents = CLF_READ_INT(&current_pos);
  arm->joint_currents = (double *) realloc(arm->joint_currents, arm->num_currents*sizeof(double));
  carmen_test_alloc(arm->joint_currents);
  for (i = 0; i < arm->num_currents; i++)
    arm->joint_currents[i] = CLF_READ_DOUBLE(&current_pos);
  arm->num_vels = CLF_READ_INT(&current_pos);
  arm->joint_angular_vels = (double *) realloc(arm->joint_angular_vels, 
					       arm->num_vels*sizeof(double));
  carmen_test_alloc(arm->joint_angular_vels);
  for (i = 0; i < arm->num_vels; i++)
    arm->joint_angular_vels[i] = CLF_READ_DOUBLE(&current_pos);
  arm->gripper_closed = CLF_READ_INT(&current_pos);

  arm->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&arm->host, &current_pos);
  return current_pos;
}

char *carmen_string_to_simulator_truepos_message(char *string,
						 carmen_simulator_truepos_message *truepos)
{
  char *current_pos = string;

  if (strncmp(current_pos, "TRUEPOS ", 8) == 0)
    current_pos = carmen_next_word(current_pos); 

 
  truepos->truepose.x = CLF_READ_DOUBLE(&current_pos);
  truepos->truepose.y = CLF_READ_DOUBLE(&current_pos);
  truepos->truepose.theta = CLF_READ_DOUBLE(&current_pos);
  truepos->odometrypose.x = CLF_READ_DOUBLE(&current_pos);
  truepos->odometrypose.y = CLF_READ_DOUBLE(&current_pos);
  truepos->odometrypose.theta = CLF_READ_DOUBLE(&current_pos);
  truepos->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&truepos->host, &current_pos);
  return current_pos;
}

double carmen_laser_guess_fov(int num_beams)
{
  if (num_beams == 181)
    return M_PI;                  /* 180 degrees */
  else if (num_beams == 180)
    return M_PI / 180.0 * 179.0;  /* 179 degrees (last beam ignored)*/
  else if (num_beams == 361)
    return M_PI;                  /* 180 degrees */
  else if (num_beams == 360)
    return M_PI / 180.0 * 179.5 ; /* 179.5 degrees (last beam ignored)*/
  else if (num_beams == 401)
    return M_PI / 180.0 * 100.0 ; /* 100.0 degrees */
  else if (num_beams == 400)
    return M_PI / 100.0 * 99.75 ; /* 99.75 degrees (last beam ignored)*/
  else
    return M_PI;                  /* assume 180 degrees */
}

double carmen_laser_guess_angle_increment(int num_beams)
{
  if (num_beams == 181 || num_beams == 180)
    return M_PI / 180.0; /* 1 degree = M_PI/180 */
  else if (num_beams == 361 || num_beams == 360)
    return M_PI / 360.0;  /* 0.5 degrees = M_PI/360 */
  else if (num_beams == 401 || num_beams == 400)
    return M_PI / 720.0;  /* 0.25 degrees = M_PI/720 */
  else
    return carmen_laser_guess_fov(num_beams) /
      ((double) (num_beams-1));
}

char *carmen_string_to_laser_laser_message_orig(char *string,
						carmen_laser_laser_message
						*laser)
{
  char *current_pos = string;
  int i, num_readings;

  if (strncmp(current_pos, "LASER", 5) == 0)
    current_pos = carmen_next_word(current_pos); 
 

  num_readings = CLF_READ_INT(&current_pos);
  if(laser->num_readings != num_readings) {
    laser->num_readings = num_readings;
    laser->range = (float *)realloc(laser->range, laser->num_readings * 
				    sizeof(float));
    carmen_test_alloc(laser->range);
  }
  for(i = 0; i < laser->num_readings; i++)
    laser->range[i] = CLF_READ_DOUBLE(&current_pos);
  laser->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&laser->host, &current_pos);

  /* fill in remission with nothing */
  laser->num_remissions = 0;
  laser->remission = NULL;

  /* guess at fields */
  laser->config.laser_type = SICK_LMS;
  laser->config.fov = carmen_laser_guess_fov(laser->num_readings);
  laser->config.start_angle = -M_PI / 2.0;
  laser->config.angular_resolution = 
    carmen_laser_guess_angle_increment(laser->num_readings);
  laser->config.maximum_range = 80.0;
  laser->config.accuracy = 0.01;
  laser->config.remission_mode = 0;

  return current_pos;
}

char *carmen_string_to_robot_laser_message_orig(char *string,
						carmen_robot_laser_message
						*laser)
{
  char *current_pos = string;
  int i, num_readings;

  if (strncmp(current_pos, "FLASER", 6) == 0 || 
      strncmp(current_pos, "RLASER", 6) == 0)
    current_pos = carmen_next_word(current_pos); 

  num_readings = CLF_READ_INT(&current_pos);
  if(laser->num_readings != num_readings) {
    laser->num_readings = num_readings;
    laser->range = (float *)realloc(laser->range, laser->num_readings * 
				    sizeof(float));
    carmen_test_alloc(laser->range);
    laser->tooclose = (char *)realloc(laser->tooclose, laser->num_readings);
    carmen_test_alloc(laser->tooclose);
  }
  for(i = 0; i < laser->num_readings; i++) {
    laser->range[i] = CLF_READ_DOUBLE(&current_pos);
    laser->tooclose[i] = 0;
  }

  laser->laser_pose.x = CLF_READ_DOUBLE(&current_pos);
  laser->laser_pose.y = CLF_READ_DOUBLE(&current_pos);
  laser->laser_pose.theta = CLF_READ_DOUBLE(&current_pos);
  laser->robot_pose.x = CLF_READ_DOUBLE(&current_pos);
  laser->robot_pose.y = CLF_READ_DOUBLE(&current_pos);
  laser->robot_pose.theta = CLF_READ_DOUBLE(&current_pos);
  laser->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&laser->host, &current_pos);

  /* fill in remission with nothing */
  laser->num_remissions = 0;
  laser->remission = NULL;

  /* guess at fields */
  laser->config.laser_type = SICK_LMS;
  laser->config.fov = carmen_laser_guess_fov(laser->num_readings);
  laser->config.start_angle = -M_PI / 2.0;
  laser->config.angular_resolution = 
    carmen_laser_guess_angle_increment(laser->num_readings);
  laser->config.maximum_range = 80.0;
  laser->config.accuracy = 0.01;
  laser->config.remission_mode = 0;

  return current_pos;
}

char *carmen_string_to_laser_laser_message(char *string,
					   carmen_laser_laser_message *laser)
{
  char *current_pos = string;
  int i, num_readings, num_remissions;

  if (strncmp(current_pos, "RAWLASER", 8) == 0) {
    current_pos += 8;
    laser->id = CLF_READ_INT(&current_pos);
  } else {
    laser->id = -1;
  }

  laser->config.laser_type = CLF_READ_INT(&current_pos);
  laser->config.start_angle = CLF_READ_DOUBLE(&current_pos);
  laser->config.fov = CLF_READ_DOUBLE(&current_pos);
  laser->config.angular_resolution = CLF_READ_DOUBLE(&current_pos);
  laser->config.maximum_range = CLF_READ_DOUBLE(&current_pos);
  laser->config.accuracy = CLF_READ_DOUBLE(&current_pos);
  laser->config.remission_mode = CLF_READ_INT(&current_pos);

  num_readings = CLF_READ_INT(&current_pos);
  if(laser->num_readings != num_readings) {
    laser->num_readings = num_readings;
    laser->range = (float *)realloc(laser->range, laser->num_readings * 
				    sizeof(float));
    carmen_test_alloc(laser->range);
  }
  for(i = 0; i < laser->num_readings; i++)
    laser->range[i] = CLF_READ_DOUBLE(&current_pos);

  num_remissions = CLF_READ_INT(&current_pos);
  if(laser->num_remissions != num_remissions) {
    laser->num_remissions = num_remissions;
    laser->remission = (float *)realloc(laser->remission, 
					laser->num_remissions * sizeof(float));
    carmen_test_alloc(laser->remission);
  }
  for(i = 0; i < laser->num_remissions; i++)
    laser->remission[i] = CLF_READ_DOUBLE(&current_pos);

  laser->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&laser->host, &current_pos);

  return current_pos;
}

char *carmen_string_to_robot_laser_message(char *string,
					   carmen_robot_laser_message *laser)
{
  char *current_pos = string;
  int i, num_readings, num_remissions;

  if (strncmp(current_pos, "ROBOTLASER", 10) == 0) {
    current_pos += 10;
    laser->id = CLF_READ_INT(&current_pos);
  } else {
    laser->id = -1;
  }

  laser->config.laser_type = CLF_READ_INT(&current_pos);
  laser->config.start_angle = CLF_READ_DOUBLE(&current_pos);
  laser->config.fov = CLF_READ_DOUBLE(&current_pos);
  laser->config.angular_resolution = CLF_READ_DOUBLE(&current_pos);
  laser->config.maximum_range = CLF_READ_DOUBLE(&current_pos);
  laser->config.accuracy = CLF_READ_DOUBLE(&current_pos);
  laser->config.remission_mode = CLF_READ_INT(&current_pos);

  num_readings = CLF_READ_INT(&current_pos);
  if(laser->num_readings != num_readings) {
    laser->num_readings = num_readings;
    laser->range = (float *)realloc(laser->range, laser->num_readings * 
				    sizeof(float));
    carmen_test_alloc(laser->range);


    laser->tooclose = (char *)realloc(laser->tooclose, laser->num_readings * 
				       sizeof(char));
    carmen_test_alloc(laser->tooclose);
    

  }
  for(i = 0; i < laser->num_readings; i++) {
    laser->range[i] = CLF_READ_DOUBLE(&current_pos);
    laser->tooclose[i] = 0;
  }

  num_remissions = CLF_READ_INT(&current_pos);
  if(laser->num_remissions != num_remissions) {
    laser->num_remissions = num_remissions;
    laser->remission = (float *)realloc(laser->remission, 
					laser->num_remissions * sizeof(float));
    carmen_test_alloc(laser->remission);
  }
  for(i = 0; i < laser->num_remissions; i++)
    laser->remission[i] = CLF_READ_DOUBLE(&current_pos);

  laser->laser_pose.x = CLF_READ_DOUBLE(&current_pos);
  laser->laser_pose.y = CLF_READ_DOUBLE(&current_pos);
  laser->laser_pose.theta = CLF_READ_DOUBLE(&current_pos);
  laser->robot_pose.x = CLF_READ_DOUBLE(&current_pos);
  laser->robot_pose.y = CLF_READ_DOUBLE(&current_pos);
  laser->robot_pose.theta = CLF_READ_DOUBLE(&current_pos);

  laser->tv = CLF_READ_DOUBLE(&current_pos);
  laser->rv = CLF_READ_DOUBLE(&current_pos);
  laser->forward_safety_dist = CLF_READ_DOUBLE(&current_pos);
  laser->side_safety_dist = CLF_READ_DOUBLE(&current_pos);
  laser->turn_axis = CLF_READ_DOUBLE(&current_pos);

  laser->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&laser->host, &current_pos);

  return current_pos;
}



char *carmen_string_to_gps_gpgga_message(char *string,
				       carmen_gps_gpgga_message *gps_msg)
{
  char *current_pos = string;
  
  if (strncmp(current_pos, "NMEAGGA ", 8) == 0)
    current_pos = carmen_next_word(current_pos); 
  
  gps_msg->nr               = CLF_READ_INT(&current_pos);
  gps_msg->utc              = CLF_READ_DOUBLE(&current_pos);
  gps_msg->latitude_dm      = CLF_READ_DOUBLE(&current_pos);
  gps_msg->latitude         = carmen_global_convert_degmin_to_double(gps_msg->latitude_dm);
  current_pos = carmen_next_word(current_pos); 
  gps_msg->lat_orient       = CLF_READ_CHAR(&current_pos);
  gps_msg->longitude_dm     = CLF_READ_DOUBLE(&current_pos);
  gps_msg->longitude        = carmen_global_convert_degmin_to_double(gps_msg->longitude_dm);
  current_pos = carmen_next_word(current_pos); 
  gps_msg->long_orient      = CLF_READ_CHAR(&current_pos);
  gps_msg->gps_quality      = CLF_READ_INT(&current_pos);
  gps_msg->num_satellites   = CLF_READ_INT(&current_pos);
  gps_msg->hdop             = CLF_READ_DOUBLE(&current_pos);
  gps_msg->sea_level        = CLF_READ_DOUBLE(&current_pos);
  gps_msg->altitude         = CLF_READ_DOUBLE(&current_pos);
  gps_msg->geo_sea_level    = CLF_READ_DOUBLE(&current_pos);
  gps_msg->geo_sep          = CLF_READ_DOUBLE(&current_pos);
  gps_msg->data_age         = CLF_READ_INT(&current_pos);
  gps_msg->timestamp        = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&gps_msg->host, &current_pos);

  return current_pos;
}

char *carmen_string_to_gps_gprmc_message(char *string,
					 carmen_gps_gprmc_message *gps_msg)
{
  char *current_pos = string;
  
  if (strncmp(current_pos, "NMEARMC ", 8) == 0)
    current_pos = carmen_next_word(current_pos); 
  
  gps_msg->nr               = CLF_READ_INT(&current_pos);
  gps_msg->validity         = CLF_READ_INT(&current_pos);
  gps_msg->utc              = CLF_READ_DOUBLE(&current_pos);
  gps_msg->latitude_dm      = CLF_READ_DOUBLE(&current_pos);
  gps_msg->latitude         = carmen_global_convert_degmin_to_double(gps_msg->latitude_dm);
  current_pos = carmen_next_word(current_pos); 
  gps_msg->lat_orient       = CLF_READ_CHAR(&current_pos);
  gps_msg->longitude_dm     = CLF_READ_DOUBLE(&current_pos);
  gps_msg->longitude        = carmen_global_convert_degmin_to_double(gps_msg->longitude_dm);
  current_pos = carmen_next_word(current_pos); 
  gps_msg->long_orient      = CLF_READ_CHAR(&current_pos);
  gps_msg->speed            = CLF_READ_DOUBLE(&current_pos);
  gps_msg->true_course      = CLF_READ_DOUBLE(&current_pos);
  gps_msg->variation        = CLF_READ_DOUBLE(&current_pos);
  current_pos = carmen_next_word(current_pos); 
  gps_msg->var_dir          = CLF_READ_CHAR(&current_pos);
  gps_msg->date             = CLF_READ_INT(&current_pos);
  gps_msg->timestamp        = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&gps_msg->host, &current_pos);

  return current_pos;
}

char* carmen_string_to_base_sonar_message(char* string, carmen_base_sonar_message* sonar_msg)
{
  int i;
  char* current_pos = string;
  if (strncmp(current_pos, "SONAR ", 6) == 0)
    current_pos = carmen_next_word(current_pos);

  sonar_msg->cone_angle = CLF_READ_DOUBLE(&current_pos);
  sonar_msg->num_sonars = CLF_READ_INT(&current_pos);
  sonar_msg->sonar_offsets = (carmen_point_p) realloc(sonar_msg->sonar_offsets, sizeof(carmen_point_t) * sonar_msg->num_sonars);
  sonar_msg->range = (double*) realloc(sonar_msg->range, sizeof(double) * sonar_msg->num_sonars);
  for (i = 0; i < sonar_msg->num_sonars; ++i)
    sonar_msg->range[i] = CLF_READ_DOUBLE(&current_pos);
  for (i = 0; i < sonar_msg->num_sonars; ++i) {
    sonar_msg->sonar_offsets[i].x     = CLF_READ_DOUBLE(&current_pos);
    sonar_msg->sonar_offsets[i].y     = CLF_READ_DOUBLE(&current_pos);
    sonar_msg->sonar_offsets[i].theta = CLF_READ_DOUBLE(&current_pos);
  }
  sonar_msg->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&sonar_msg->host, &current_pos);

  return current_pos;
}

char* carmen_string_to_base_bumper_message(char* string, carmen_base_bumper_message* bumper_msg)
{
  int i;
  char* current_pos = string;
  if (strncmp(current_pos, "BUMPER ", 7) == 0)
    current_pos = carmen_next_word(current_pos);

  bumper_msg->num_bumpers = CLF_READ_INT(&current_pos);
  bumper_msg->state = (unsigned char*) realloc(bumper_msg->state, sizeof(unsigned char) * bumper_msg->num_bumpers);
  bumper_msg->bumper_offsets = (carmen_position_t*) realloc(bumper_msg->bumper_offsets, sizeof(carmen_position_t) * bumper_msg->num_bumpers);
  for (i = 0; i < bumper_msg->num_bumpers; ++i)
    bumper_msg->state[i] = CLF_READ_INT(&current_pos);
  for (i = 0; i < bumper_msg->num_bumpers; ++i) {
    bumper_msg->bumper_offsets[i].x = CLF_READ_DOUBLE(&current_pos);
    bumper_msg->bumper_offsets[i].y = CLF_READ_DOUBLE(&current_pos);
  }
  bumper_msg->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&bumper_msg->host, &current_pos);

  return current_pos;
}

char* carmen_string_to_pantilt_scanmark_message(char* string, carmen_pantilt_scanmark_message* scanmark) {

  char *current_pos = string;

  if (strncmp(current_pos, "SCANMARK ", 9) == 0) {
    current_pos = carmen_next_word(current_pos);
  }

  scanmark->type = CLF_READ_INT(&current_pos);
  scanmark->laserid = CLF_READ_INT(&current_pos);

  scanmark->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&scanmark->host, &current_pos);

  return current_pos;
}

char* carmen_string_to_pantilt_laserpos_message(char* string, carmen_pantilt_laserpos_message* laserpos) {

  char *current_pos = string;

  if (strncmp(current_pos, "POSITIONLASER ", 14) == 0) {
    current_pos = carmen_next_word(current_pos);
  } 
  
  laserpos->id = CLF_READ_INT(&current_pos);
  
  laserpos->x = CLF_READ_DOUBLE(&current_pos);
  laserpos->y = CLF_READ_DOUBLE(&current_pos);
  laserpos->z = CLF_READ_DOUBLE(&current_pos);

  laserpos->phi = CLF_READ_DOUBLE(&current_pos);
  laserpos->theta = CLF_READ_DOUBLE(&current_pos);
  laserpos->psi = CLF_READ_DOUBLE(&current_pos);
  
  laserpos->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&laserpos->host, &current_pos);
  
  return current_pos;
}


char* carmen_string_to_imu_message(char* string, carmen_imu_message* msg)
{
  char* current_pos = string;
  if (strncmp(current_pos, "IMU ", 4) == 0)
    current_pos = carmen_next_word(current_pos);

  msg->accX = CLF_READ_DOUBLE(&current_pos);
  msg->accY = CLF_READ_DOUBLE(&current_pos);
  msg->accZ = CLF_READ_DOUBLE(&current_pos);

  msg->q0 = CLF_READ_DOUBLE(&current_pos);
  msg->q1 = CLF_READ_DOUBLE(&current_pos);
  msg->q2 = CLF_READ_DOUBLE(&current_pos);
  msg->q3 = CLF_READ_DOUBLE(&current_pos);

  msg->magX = CLF_READ_DOUBLE(&current_pos);
  msg->magY = CLF_READ_DOUBLE(&current_pos);
  msg->magZ = CLF_READ_DOUBLE(&current_pos);

  msg->gyroX = CLF_READ_DOUBLE(&current_pos);
  msg->gyroY = CLF_READ_DOUBLE(&current_pos);
  msg->gyroZ = CLF_READ_DOUBLE(&current_pos);

  msg->timestamp = CLF_READ_DOUBLE(&current_pos);
  copy_host_string(&msg->host, &current_pos);

  return current_pos;
}

char *carmen_string_to_robot_velocity_message(char *string, carmen_robot_velocity_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "VELOCITY", 8) == 0)
   	current_pos = carmen_next_word(current_pos); 
  
  	msg->tv = CLF_READ_DOUBLE(&current_pos);
  	msg->rv = CLF_READ_DOUBLE(&current_pos);
  	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
   copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char *carmen_string_to_robot_vector_move_message(char *string, carmen_robot_vector_move_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "VECTORMOVE", 10) == 0)
   	current_pos = carmen_next_word(current_pos); 

  	msg->distance = CLF_READ_DOUBLE(&current_pos);
  	msg->theta = CLF_READ_DOUBLE(&current_pos);
  	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
   copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char *carmen_string_to_robot_follow_trajectory_message(char *string, carmen_robot_follow_trajectory_message *msg)
{
	char *current_pos = string;
	
	if (strncmp(current_pos, "ROBOTVELOCITY", 13) == 0)
   	current_pos = carmen_next_word(current_pos); 

	msg->robot_position.x = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.y = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.theta = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.t_vel = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.r_vel = CLF_READ_DOUBLE(&current_pos);

	int length = CLF_READ_INT(&current_pos);
	
   if(msg->trajectory_length != length) {
    msg->trajectory_length = length;
    msg->trajectory = (carmen_traj_point_t *)realloc(msg->trajectory, length * 
				   sizeof(carmen_traj_point_t));
    carmen_test_alloc(msg->trajectory);
  }



	int i;
	for (i=0; i<msg->trajectory_length; i++) 
	{
		msg->trajectory[i].x = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].y = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].theta = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].t_vel = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].r_vel = CLF_READ_DOUBLE(&current_pos);
	}

  	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
   copy_host_string(&msg->host, &current_pos);
	return current_pos;
}

char *carmen_string_to_base_velocity_message(char *string, carmen_base_velocity_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "BASEVELOCITY", 12) == 0)
   	current_pos = carmen_next_word(current_pos); 
  
  	msg->tv = CLF_READ_DOUBLE(&current_pos);
  	msg->rv = CLF_READ_DOUBLE(&current_pos);
  	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
   copy_host_string(&msg->host, &current_pos);
	return current_pos;
}
