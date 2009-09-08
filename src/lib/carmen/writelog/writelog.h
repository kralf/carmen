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

/** @addtogroup logger libwritelog **/
// @{

/** 
 * \file writelog.h 
 * \brief Library for writing log files. 
 *
 * This library should be used to write logfiles. 
 **/


#ifndef CARMEN_LOGWRITE_H
#define CARMEN_LOGWRITE_H

#include "carmen_stdio.h"

#include "arm_messages.h"
#include "base_messages.h"
#include "camera_messages.h"
#include "gps_nmea_messages.h"
#include "imu_messages.h"
#include "laser_messages.h"
#include "localize_messages.h"
#include "logger_messages.h"
#include "map_messages.h"
#include "navigator_messages.h"
#include "pantilt_messages.h"
#include "param_messages.h"
#include "robot_messages.h"
#include "simulator_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_LOGFILE_HEADER "# CARMEN Logfile"

void carmen_logwrite_write_robot_name(char *robot_name, 
				      carmen_FILE *outfile);

void carmen_logwrite_write_header(carmen_FILE *outfile);

/** Converts the corresponding message into a string in the carmen log format.
 * @param odometry The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_odometry(carmen_base_odometry_message *odometry, 
				    carmen_FILE *outfile, double timestamp);  

/** Converts the corresponding message into a string in the carmen log format.
 * @param arm The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_arm(carmen_arm_state_message *arm, 
			       carmen_FILE *outfile,
			       double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param laser The message that should be written to the log file.
 * @param laser_num The number of the laser (1=front, 2=rear, 3=laser3, 4=laser4, ...)
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_laser_laser(carmen_laser_laser_message *laser,
				       int laser_num, carmen_FILE *outfile,
					double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param laser The message that should be written to the log file.
 * @param laser_num The number of the laser (1=front, 2=rear, 3=laser3, 4=laser4, ...)
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_robot_laser(carmen_robot_laser_message *laser,
				       int laser_num, carmen_FILE *outfile,
				       double timestamp);

/** Converts the given parameter into a string in the carmen log format.
 **/
void carmen_logwrite_write_param(char *module, char *variable, char *value, 
				 double ipc_time, char *hostname, 
				 carmen_FILE *outfile, double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param sync_message The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 **/
void carmen_logwrite_write_sync(carmen_logger_sync_message *sync_message, 
				carmen_FILE *outfile);

/** Converts the corresponding message into a string in the carmen log format.
 * @param truepos The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_truepos(carmen_simulator_truepos_message *truepos, 
				   carmen_FILE *outfile, double timestamp);  

/** Converts the corresponding message into a string in the carmen log format.
 * @param msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_localize(carmen_localize_globalpos_message *msg, 
				    carmen_FILE *outfile, double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param gps_msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logger_write_gps_gpgga(carmen_gps_gpgga_message *gps_msg, 
				   carmen_FILE *outfile, 
				   double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param gps_msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logger_write_gps_gprmc(carmen_gps_gprmc_message *gps_msg, 
				   carmen_FILE *outfile, 
				   double timestamp);


void carmen_logwrite_write_base_sonar(carmen_base_sonar_message *sonar,
				       carmen_FILE *outfile,
				       double timestamp);

void carmen_logwrite_write_base_bumper(carmen_base_bumper_message *bumper,
				       carmen_FILE *outfile,
				       double timestamp);

void carmen_logwrite_write_pantilt_scanmark(carmen_pantilt_scanmark_message* scanmark,
				       carmen_FILE* outfile,
				       double timestamp);

void carmen_logwrite_write_pantilt_laserpos(carmen_pantilt_laserpos_message* laserpos,
				       carmen_FILE* outfile,
				       double timestamp);


void carmen_logwrite_write_imu(carmen_imu_message *imu,
			       carmen_FILE *outfile,
			       double timestamp);
				   
				   
/** Converts the corresponding message into a string in the carmen log format.
 * @param msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_robot_vector_move(carmen_robot_vector_move_message *msg,
					carmen_FILE *outfile,
					double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_robot_velocity(carmen_robot_velocity_message *msg,
					carmen_FILE *outfile,
					double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_robot_follow_trajectory(carmen_robot_follow_trajectory_message *msg,
					carmen_FILE *outfile,
					double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_base_velocity(carmen_base_velocity_message *msg,
					carmen_FILE *outfile,
					double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_logger_comment(carmen_logger_comment_message *msg,
              carmen_FILE *outfile,
              double timestamp);
#ifdef __cplusplus
}
#endif

#endif
// @}
