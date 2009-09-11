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

#include "robot_central.h"
#include "robot_main.h"
#include "robot_sonar.h"

static carmen_base_sonar_message base_sonar;
static carmen_robot_sonar_message robot_sonar;

static carmen_running_average_t sonar_average;
static int sonar_count = 0;
static int sonar_ready = 0;

static double max_front_velocity = 0;
static double min_rear_velocity = -0;

static int collision_avoidance = 0;

double carmen_robot_interpolate_heading(double head1, double head2, double fraction);

static void check_message_data_chunk_sizes(void)
{
  int first=1;

  if(first) {
    robot_sonar.num_sonars = base_sonar.num_sonars;
    robot_sonar.ranges = 
      (double *)calloc(robot_sonar.num_sonars, sizeof(double));
    carmen_test_alloc(robot_sonar.ranges);
    robot_sonar.sonar_offsets = 
      (carmen_point_p)calloc(robot_sonar.num_sonars, sizeof(carmen_point_t));
    carmen_test_alloc(robot_sonar.sonar_offsets);
    first = 0;
  } else if(robot_sonar.num_sonars != base_sonar.num_sonars) {
    robot_sonar.num_sonars = base_sonar.num_sonars;
    robot_sonar.ranges = (double *)realloc
      (robot_sonar.ranges, sizeof(double) * robot_sonar.num_sonars);
    carmen_test_alloc(robot_sonar.ranges);
    robot_sonar.sonar_offsets = (carmen_point_p)
      realloc(robot_sonar.sonar_offsets,sizeof(carmen_point_t) * 
	      robot_sonar.num_sonars);
    carmen_test_alloc(robot_sonar.sonar_offsets);
  }
}
      

static void
construct_sonar_message(carmen_robot_sonar_message *msg, int low, int high, 
			double fraction)
{
  msg->robot_pose.x=carmen_robot_odometry[low].x + fraction *
    (carmen_robot_odometry[high].x - carmen_robot_odometry[low].x);
  msg->robot_pose.y= carmen_robot_odometry[low].y + fraction *
    (carmen_robot_odometry[high].y - carmen_robot_odometry[low].y);
  msg->robot_pose.theta=carmen_robot_interpolate_heading
    (carmen_robot_odometry[high].theta, 
     carmen_robot_odometry[low].theta, fraction);
  msg->robot_pose.theta=carmen_normalize_theta(msg->robot_pose.theta);


  msg->tv = carmen_robot_odometry[low].tv + 
    fraction*(carmen_robot_odometry[high].tv - 
	      carmen_robot_odometry[low].tv);
  msg->rv = carmen_robot_odometry[low].rv + 
    fraction*(carmen_robot_odometry[high].rv - 
	      carmen_robot_odometry[low].rv);
}

void carmen_robot_correct_sonar_and_publish(void) 
{
  
  double sonar_skew;
  double fraction;
  int low, high;
  IPC_RETURN_TYPE err;

  if(!sonar_ready) 
    return;

  sonar_ready = carmen_robot_get_skew(sonar_count, &sonar_skew, 
				      &sonar_average, base_sonar.host);
  if (!sonar_ready) {
    carmen_warn("Waiting for sonar data to accumulate\n");
    return;
  }

  fraction = carmen_robot_get_fraction(base_sonar.timestamp, sonar_skew,
				       &low, &high);

  if (!carmen_robot_config.interpolate_odometry)
    fraction=0;

  construct_sonar_message(&robot_sonar, low, high, fraction);

  err = IPC_publishData(CARMEN_ROBOT_SONAR_NAME, &robot_sonar);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_ROBOT_SONAR_NAME);
  fprintf(stderr, "s");
  sonar_ready = 0;
}

static void sonar_handler(void)
{
  int i;
  double safety_distance;
  
  double theta;
  carmen_traj_point_t robot_posn;
  carmen_traj_point_t obstacle_pt;
  double max_velocity;
  double velocity;

  int tooclose = -1;
  double tooclose_theta = 0;

  int min_index;
  //double min_theta = 0;
  double min_dist;
  
  check_message_data_chunk_sizes();

  carmen_robot_update_skew(&sonar_average, &sonar_count, 
			   base_sonar.timestamp, base_sonar.host);
  
  memcpy(robot_sonar.ranges, base_sonar.range, robot_sonar.num_sonars * 
	 sizeof(double));
  memcpy(robot_sonar.sonar_offsets, base_sonar.sonar_offsets, robot_sonar.num_sonars * 
	 sizeof(carmen_point_t));
  robot_sonar.timestamp = base_sonar.timestamp;
  robot_sonar.cone_angle=base_sonar.cone_angle;

  robot_sonar.host =  base_sonar.host;
 

  carmen_robot_sensor_time_of_last_update = carmen_get_time();


  if (collision_avoidance) {
    safety_distance = 
      carmen_robot_config.length / 2.0 + carmen_robot_config.approach_dist + 
      0.5 * carmen_robot_latest_odometry.tv * carmen_robot_latest_odometry.tv / 
      carmen_robot_config.acceleration +
      carmen_robot_latest_odometry.tv * carmen_robot_config.reaction_time;
    
    max_velocity = carmen_robot_config.max_t_vel;
    
    robot_posn.x = 0;
    robot_posn.y = 0;
    robot_posn.theta = 0;
    
    for(i=0; i<robot_sonar.num_sonars; i++) {
      theta=robot_sonar.sonar_offsets[i].theta;
      obstacle_pt.x=robot_sonar.sonar_offsets[i].x+robot_sonar.ranges[i]*
	cos(theta);
      obstacle_pt.y=robot_sonar.sonar_offsets[i].y+robot_sonar.ranges[i]*
	sin(theta);
      carmen_geometry_move_pt_to_rotating_ref_frame(&obstacle_pt, carmen_robot_latest_odometry.tv, carmen_robot_latest_odometry.rv);
      velocity=carmen_geometry_compute_velocity(robot_posn, obstacle_pt, &carmen_robot_config);
      if(velocity < carmen_robot_config.max_t_vel) {
	if(velocity<max_velocity) {
	  tooclose=i;
	  tooclose_theta=theta;
	  max_velocity = velocity;
	}
      }
      
    }
    
    min_index=0;
    min_dist = robot_sonar.ranges[0];
    for(i=1; i<robot_sonar.num_sonars; i++) {
      if(robot_sonar.ranges[i]<min_dist) {
	min_dist=robot_sonar.ranges[i];
	min_index=i;
      }
    }
    
    
    if(max_velocity<=0 && max_velocity<CARMEN_ROBOT_MIN_ALLOWED_VELOCITY)
      max_velocity=0.0;
    if(max_velocity<=0 && carmen_robot_latest_odometry.tv>0.0)
      carmen_robot_stop_robot(CARMEN_ROBOT_ALLOW_ROTATE);
  }

  sonar_ready=1;
}

double carmen_robot_sonar_max_front_velocity(void) 
{
  return max_front_velocity;
}

double carmen_robot_sonar_min_rear_velocity(void) 
{
  return min_rear_velocity;
}

void carmen_robot_add_sonar_handler(void) 
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_ROBOT_SONAR_NAME,
		      IPC_VARIABLE_LENGTH,
		      CARMEN_ROBOT_SONAR_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_SONAR_NAME);

  carmen_base_subscribe_sonar_message(&base_sonar,
				    (carmen_handler_t)sonar_handler,
				    CARMEN_SUBSCRIBE_LATEST);

  carmen_running_average_clear(&sonar_average);
}

void carmen_robot_add_sonar_parameters(char *progname __attribute__ ((unused)) ) 
{

}

