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

void carmen_logwrite_write_robot_name(char *robot_name, carmen_FILE *outfile)
{
  carmen_fprintf(outfile, "# robot: %s\n", robot_name);
}

void carmen_logwrite_write_header(carmen_FILE *outfile)
{
  carmen_fprintf(outfile, "%s\n", CARMEN_LOGFILE_HEADER);
  carmen_fprintf(outfile, "# file format is one message per line\n");
  carmen_fprintf(outfile, "# message_name [message contents] ipc_timestamp ipc_hostname logger_timestamp\n");
  carmen_fprintf(outfile, "# message formats defined: PARAM SYNC ODOM RAWLASER1 RAWLASER2 RAWLASER3 RAWLASER4 ROBOTLASER1 ROBOTLASER2 FLASER RLASER LASER3 LASER4\n");
  carmen_fprintf(outfile, "# PARAM param_name param_value\n");
  carmen_fprintf(outfile, "# COMMENT text \n");
  carmen_fprintf(outfile, "# SYNC tagname\n");
  carmen_fprintf(outfile, "# ODOM x y theta tv rv accel\n");
  carmen_fprintf(outfile, "# TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta\n");
  carmen_fprintf(outfile, "# RAWLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
  carmen_fprintf(outfile, "# RAWLASER2 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
  carmen_fprintf(outfile, "# RAWLASER3 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
  carmen_fprintf(outfile, "# RAWLASER4 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]\n");
  carmen_fprintf(outfile, "# POSITIONLASER laserid x y z phi(roll) theta(pitch) psi(yaw) \n");
  carmen_fprintf(outfile, "# ROBOTLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values] laser_pose_x laser_pose_y laser_pose_theta robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis\n");
  carmen_fprintf(outfile, "# ROBOTLASER2 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values] laser_pose_x laser_pose_y laser_pose_theta robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis\n");
  carmen_fprintf(outfile, "# NMEAGGA gpsnr utc latitude_dm lat_orient longitude_dm long_orient gps_quality num_satellites hdop sea_level alititude geo_sea_level geo_sep data_age\n");
  carmen_fprintf(outfile, "# NMEARMC gpsnr validity utc latitude_dm lat_orient longitude_dm long_orient speed course variation var_dir date\n");
  carmen_fprintf(outfile, "# SONAR cone_angle num_sonars [sonar_reading] [sonar_offsets x y theta]\n");
  carmen_fprintf(outfile, "# BUMPER num_bumpers [bumper_reading] [bumper_offsets x y]\n");
  carmen_fprintf(outfile, "# SCANMARK start_stop_indicator laserID \n");
  carmen_fprintf(outfile, "# IMU accelerationX accelerationY accelerationZ quaternion_q0 quaternion_q1 quaternion_q2 quaternion_q3 magneticfieldX magneticfieldY magneticfieldZ gyroX gyroY gyroZ\n");
  carmen_fprintf(outfile, "# VECTORMOVE distance theta\n"); 
  carmen_fprintf(outfile, "# ROBOTVELOCITY tv rv \n"); 
  carmen_fprintf(outfile, "# FOLLOWTRAJECTORY x y theta tv rv num readings [trajectory points: x y theta tv rv]\n");
  carmen_fprintf(outfile, "# BASEVELOCITY tv rv \n");   
  carmen_fprintf(outfile, "# \n");
  carmen_fprintf(outfile, "# OLD LOG MESSAGES: \n");
  carmen_fprintf(outfile, "# (old) # FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n");
  carmen_fprintf(outfile, "# (old) # RLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n");
  carmen_fprintf(outfile, "# (old) # LASER3 num_readings [range_readings]\n");
  carmen_fprintf(outfile, "# (old) # LASER4 num_readings [range_readings]\n");
  carmen_fprintf(outfile, "# (old) # REMISSIONFLASER num_readings [range_readings remission_value]\n");
  carmen_fprintf(outfile, "# (old) # REMISSIONRLASER num_readings [range_readings remission_value]\n");
  carmen_fprintf(outfile, "# (old) # REMISSIONLASER3 num_readings [range_readings remission_value]\n");
  carmen_fprintf(outfile, "# (old) # REMISSIONLASER4 num_readings [range_readings remission_value]\n");
}

void carmen_logwrite_write_odometry(carmen_base_odometry_message *odometry, 
				    carmen_FILE *outfile,
				    double timestamp)
{
  carmen_fprintf(outfile, "ODOM %f %f %f %f %f %f %f %s %f\n", odometry->x,
		 odometry->y, odometry->theta, odometry->tv, odometry->rv, 
		 odometry->acceleration, odometry->timestamp, odometry->host,
		 timestamp);
}

void carmen_logwrite_write_arm(carmen_arm_state_message *arm, 
				    carmen_FILE *outfile,
				    double timestamp)
{
  int i;

  carmen_fprintf(outfile, "ARM %d %d", arm->flags, arm->num_joints);
  for (i = 0; i < arm->num_joints; i++)
    carmen_fprintf(outfile, " %f", arm->joint_angles[i]);
  carmen_fprintf(outfile, " %d", arm->num_currents);
  for (i = 0; i < arm->num_currents; i++)
    carmen_fprintf(outfile, " %f", arm->joint_currents[i]);
  carmen_fprintf(outfile, " %d", arm->num_vels);
  for (i = 0; i < arm->num_vels; i++)
    carmen_fprintf(outfile, " %f", arm->joint_angular_vels[i]);
  carmen_fprintf(outfile, " %d", arm->gripper_closed);

  carmen_fprintf(outfile, " %f %s %f\n", arm->timestamp, arm->host, timestamp);
}

void carmen_logwrite_write_truepos(carmen_simulator_truepos_message *truepos, 
				   carmen_FILE *outfile, double timestamp)
{

  carmen_fprintf(outfile, "TRUEPOS %f %f %f %f %f %f %f %s %f\n", 
		 truepos->truepose.x, truepos->truepose.y, 
		 truepos->truepose.theta,  truepos->odometrypose.x,
		 truepos->odometrypose.y, truepos->odometrypose.theta, 
		 truepos->timestamp, truepos->host, timestamp);
}

void carmen_logwrite_write_laser_laser(carmen_laser_laser_message *laser,
				       int laser_num, carmen_FILE *outfile,
				       double timestamp)
{
  int i;

  carmen_fprintf(outfile, "RAWLASER%d ", laser_num);
  carmen_fprintf(outfile, "%d %f %f %f %f %f %d ", 
		 laser->config.laser_type,
		 laser->config.start_angle, 
		 laser->config.fov, 
		 laser->config.angular_resolution, 
		 laser->config.maximum_range, 
		 laser->config.accuracy,
		 laser->config.remission_mode);
  carmen_fprintf(outfile, "%d ", laser->num_readings);
  for(i = 0; i < laser->num_readings; i++)
    carmen_fprintf(outfile, "%.3f ", laser->range[i]);
  carmen_fprintf(outfile, "%d ", laser->num_remissions);
  for(i = 0; i < laser->num_remissions; i++)
    carmen_fprintf(outfile, "%f ", laser->remission[i]);
  carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp,
		 laser->host, timestamp);
}

void carmen_logwrite_write_robot_laser(carmen_robot_laser_message *laser,
				       int laser_num, carmen_FILE *outfile,
				       double timestamp)
{
  int i;

  carmen_fprintf(outfile, "ROBOTLASER%d ", laser_num);
  carmen_fprintf(outfile, "%d %f %f %f %f %f %d ", laser->config.laser_type,
		 laser->config.start_angle, laser->config.fov, 
		 laser->config.angular_resolution, 
		 laser->config.maximum_range, 
		 laser->config.accuracy,
		 laser->config.remission_mode);
  carmen_fprintf(outfile, "%d ", laser->num_readings);
  for(i = 0; i < laser->num_readings; i++)
    carmen_fprintf(outfile, "%.3f ", laser->range[i]);
  carmen_fprintf(outfile, "%d ", laser->num_remissions);
  for(i = 0; i < laser->num_remissions; i++)
    carmen_fprintf(outfile, "%f ", laser->remission[i]);
  carmen_fprintf(outfile, "%f %f %f %f %f %f ", laser->laser_pose.x,
		 laser->laser_pose.y, laser->laser_pose.theta,
		 laser->robot_pose.x, laser->robot_pose.y, 
		 laser->robot_pose.theta);
  carmen_fprintf(outfile, "%f %f %f %f %f ", laser->tv, laser->rv, 
		 laser->forward_safety_dist, laser->side_safety_dist,
		 laser->turn_axis);
  carmen_fprintf(outfile, "%f %s %f\n", laser->timestamp,
		 laser->host, timestamp);
}

void carmen_logwrite_write_param(char *module, char *variable, char *value, 
				 double ipc_time, char *hostname, 
				 carmen_FILE *outfile, double timestamp)
{
  carmen_fprintf(outfile, "PARAM %s_%s %s %f %s %f\n", module, 
		 variable, value, ipc_time, hostname, timestamp);
}

void carmen_logwrite_write_sync(carmen_logger_sync_message *sync_message, 
				carmen_FILE *outfile)
{
  carmen_fprintf(outfile, "SYNC %s %f %s %f\n", sync_message->tag,
		 sync_message->timestamp, sync_message->host,
		 carmen_get_time());
}

void carmen_logwrite_write_localize(carmen_localize_globalpos_message *msg, 
				    carmen_FILE *outfile, double timestamp)
{
  carmen_fprintf(outfile, "GLOBALPOS %f %f %f %f %f %f %f %s %f\n",
		 msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta,
		 msg->odometrypos.x, msg->odometrypos.y, 
		 msg->odometrypos.theta, msg->timestamp, msg->host, timestamp);
}

void carmen_logger_write_gps_gpgga(carmen_gps_gpgga_message *gps_msg, 
				  carmen_FILE *outfile, double timestamp)
{
  char lat_o  = gps_msg->lat_orient;
  char long_o = gps_msg->long_orient;

  if (lat_o == '\0')
    lat_o = 'N';
  
  if (long_o == '\0')
    long_o = 'E';

  carmen_fprintf(outfile,"NMEAGGA %d %lf %lf %c %lf %c %d %d %lf %lf %lf %lf %lf %d %lf %s %lf\n", 
		 gps_msg->nr,
		 gps_msg->utc,
		 gps_msg->latitude_dm, 
		 lat_o, 
		 gps_msg->longitude_dm,
		 long_o, 
		 gps_msg->gps_quality, gps_msg->num_satellites,
		 gps_msg->hdop,
		 gps_msg->sea_level,
		 gps_msg->altitude,
		 gps_msg->geo_sea_level,
		 gps_msg->geo_sep,
		 gps_msg->data_age,
		 gps_msg->timestamp, gps_msg->host, timestamp);
}



void carmen_logger_write_gps_gprmc(carmen_gps_gprmc_message *gps_msg, 
				  carmen_FILE *outfile, double timestamp)
{
  char lat_o  = gps_msg->lat_orient;
  char long_o = gps_msg->long_orient;

  char vardir  = gps_msg->var_dir;


  if (lat_o == '\0')
    lat_o = 'N';
  
  if (long_o == '\0')
    long_o = 'E';

  if (vardir == '\0')
    vardir = 'E';


  carmen_fprintf(outfile,"NMEARMC %d %d %lf %lf %c %lf %c %lf %lf %lf %c %d %lf %s %lf\n", 
		 gps_msg->nr, 
		 gps_msg->validity, 
		 gps_msg->utc,
		 gps_msg->latitude_dm, 
		 lat_o, 
		 gps_msg->longitude_dm,
		 long_o, 
		 gps_msg->speed, 
		 gps_msg->true_course, 
		 gps_msg->variation, 
		 vardir,
		 gps_msg->date,
		 gps_msg->timestamp, gps_msg->host, timestamp);
}

void carmen_logwrite_write_base_sonar(carmen_base_sonar_message *sonar,
				       carmen_FILE *outfile,
				       double timestamp)
{
  int i;

  carmen_fprintf(outfile, "SONAR %f %d ", sonar->cone_angle, sonar->num_sonars);
  for (i=0; i<sonar->num_sonars; i++)
    carmen_fprintf(outfile, "%.2lf ",  sonar->range[i]);
  for (i=0; i<sonar->num_sonars; i++)
    carmen_fprintf(outfile, "%.3f %.3f %f ", sonar->sonar_offsets[i].x, sonar->sonar_offsets[i].y, sonar->sonar_offsets[i].theta);
  carmen_fprintf(outfile, "%lf %s %lf\n", sonar->timestamp, sonar->host, timestamp);
}

void carmen_logwrite_write_base_bumper(carmen_base_bumper_message *bumper,
				       carmen_FILE *outfile,
				       double timestamp)
{
  int i;

  carmen_fprintf(outfile, "BUMPER ");
  carmen_fprintf(outfile, "%d ", bumper->num_bumpers);
  for (i=0; i<bumper->num_bumpers; i++)
    carmen_fprintf(outfile, "%d ", bumper->state[i]);
  for (i=0; i<bumper->num_bumpers; i++)
    carmen_fprintf(outfile, "%.3f %.3f ", bumper->bumper_offsets[i].x, bumper->bumper_offsets[i].y);
  carmen_fprintf(outfile, "%lf %s %lf\n", bumper->timestamp, bumper->host, timestamp);
}


void carmen_logwrite_write_pantilt_scanmark(carmen_pantilt_scanmark_message *scanmark,
					    carmen_FILE *outfile, double timestamp) {

  carmen_fprintf(outfile, "SCANMARK ");

  carmen_fprintf(outfile, "%d ", scanmark->type);
  carmen_fprintf(outfile, "%d ", scanmark->laserid);

  carmen_fprintf(outfile, "%lf %s %lf\n", scanmark->timestamp, scanmark->host, timestamp);
}



void carmen_logwrite_write_pantilt_laserpos(carmen_pantilt_laserpos_message *laserpos,
					    carmen_FILE *outfile, double timestamp){

  carmen_fprintf(outfile, "POSITIONLASER %d ",laserpos->id);

  carmen_fprintf(outfile, "%f %f %f ", laserpos->x, laserpos->y, laserpos->z);
  carmen_fprintf(outfile, "%f %f %f ", laserpos->phi, laserpos->theta, laserpos->psi);

  carmen_fprintf(outfile, "%lf %s %lf\n", laserpos->timestamp, laserpos->host, timestamp);
}


void carmen_logwrite_write_imu(carmen_imu_message *msg,
			       carmen_FILE *outfile,
			       double timestamp)
{
  carmen_fprintf(outfile, "IMU ");
  carmen_fprintf(outfile, "%lf ", msg->accX);
  carmen_fprintf(outfile, "%lf ", msg->accY);
  carmen_fprintf(outfile, "%lf ", msg->accZ);

  carmen_fprintf(outfile, "%lf ", msg->q0);
  carmen_fprintf(outfile, "%lf ", msg->q1);
  carmen_fprintf(outfile, "%lf ", msg->q2);
  carmen_fprintf(outfile, "%lf ", msg->q3);

  carmen_fprintf(outfile, "%lf ", msg->magX);
  carmen_fprintf(outfile, "%lf ", msg->magY);
  carmen_fprintf(outfile, "%lf ", msg->magZ);

  carmen_fprintf(outfile, "%lf ", msg->gyroX);
  carmen_fprintf(outfile, "%lf ", msg->gyroY);
  carmen_fprintf(outfile, "%lf ", msg->gyroZ);

  carmen_fprintf(outfile, "%lf %s %lf\n", msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_robot_vector_move(carmen_robot_vector_move_message *msg,
					carmen_FILE *outfile,
					double timestamp)
{
	 carmen_fprintf(outfile, "VECTORMOVE %f %f %f %s %f\n", 
		 msg->distance, msg->theta, msg->timestamp, msg->host, timestamp);
}

void carmen_logwrite_write_robot_velocity(carmen_robot_velocity_message *msg,
					carmen_FILE *outfile,
					double timestamp)
{
	 carmen_fprintf(outfile, "ROBOTVELOCITY %f %f %f %s %f\n", 
		 msg->tv, msg->rv, msg->timestamp, msg->host, timestamp);
	
}

void carmen_logwrite_write_robot_follow_trajectory(carmen_robot_follow_trajectory_message *msg,
					carmen_FILE *outfile,
					double timestamp)
{
	carmen_fprintf(outfile, "FOLLOWTRAJECTORY %f %f %f %f %f ",
		msg->robot_position.x, msg->robot_position.y, msg->robot_position.theta, msg->robot_position.t_vel, msg->robot_position.r_vel);
	
	carmen_fprintf(outfile, "%d ", msg->trajectory_length);
	
	int i;
	for (i=0; i<msg->trajectory_length; i++)
		carmen_fprintf(outfile, "%f %f %f %f %f ",
			msg->trajectory[i].x, msg->trajectory[i].y, msg->trajectory[i].theta, msg->trajectory[i].t_vel, msg->trajectory[i].r_vel); 	
		
	carmen_fprintf(outfile, "%f %s %f\n", 
		 msg->timestamp, msg->host, timestamp);

}

void carmen_logwrite_write_base_velocity(carmen_base_velocity_message *msg,
					carmen_FILE *outfile,
					double timestamp)
{
	 carmen_fprintf(outfile, "BASEVELOCITY %f %f %f %s %f\n", 
		 msg->tv, msg->rv, msg->timestamp, msg->host, timestamp);
}


void carmen_logwrite_write_logger_comment(carmen_logger_comment_message *msg,
              carmen_FILE *outfile,
              double timestamp)
{
	unsigned int l = strlen(msg->text);
	char buffer[l+1];
	strcpy( buffer, msg->text );
	unsigned int x;
	for (x=0; x<=l; x++) 
	{
		if (msg->text[x]==' ') msg->text[x]='_';
	}
   carmen_fprintf(outfile, "COMMENT %s %f %s %f\n", 
		 msg->text, msg->timestamp, msg->host, timestamp);
}
