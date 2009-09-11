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
#include "robot_bumper.h"

#include "base_messages.h"
#include "robot_messages.h"

static carmen_base_bumper_message base_bumper;
static carmen_robot_bumper_message robot_bumper;

static carmen_running_average_t bumper_average;
static int bumper_count = 0;
static int bumper_ready = 0;
static int num_bumpers;

double carmen_robot_interpolate_heading(double head1, double head2, 
					double fraction);

static void check_message_data_chunk_sizes(void)
{
  int first = 1;

  if (first) {
    robot_bumper.num_bumpers = num_bumpers;
    robot_bumper.state = (char *)calloc(num_bumpers, sizeof(char));
    carmen_test_alloc(robot_bumper.state);
    first = 0;
  } 
}
      

static void construct_bumper_message(carmen_robot_bumper_message *msg, 
				     int low, int high, double fraction)
{
  int i;

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

  robot_bumper.timestamp = base_bumper.timestamp;
  robot_bumper.host = base_bumper.host;
  robot_bumper.num_bumpers = base_bumper.num_bumpers;

  for(i=0; i< msg->num_bumpers; i++)
    msg->state[i] = base_bumper.state[i];
}

void carmen_robot_correct_bumper_and_publish(void) 
{  
  double bumper_skew;
  double fraction;
  int low, high;
  
  if(!bumper_ready)
    return;

  check_message_data_chunk_sizes();
  
  bumper_ready = carmen_robot_get_skew(bumper_count, &bumper_skew,
				       &bumper_average, 
				       base_bumper.host);
  if (!bumper_ready) {
    carmen_warn("Waiting for bumper data to accumulate\n");
    return;
  }

  fraction = carmen_robot_get_fraction(base_bumper.timestamp, bumper_skew,
				       &low, &high);

  if (!carmen_robot_config.interpolate_odometry)
    fraction=0;

  construct_bumper_message(&robot_bumper, low, high, fraction);
    
  IPC_RETURN_TYPE err;
  err = IPC_publishData(CARMEN_ROBOT_BUMPER_NAME, &robot_bumper);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_ROBOT_BUMPER_NAME);

  fprintf(stderr, "b");
  bumper_ready = 0;  
}

int carmen_robot_bumper_on() 
{
  int i;

  for (i = 0; i < base_bumper.num_bumpers; i++)
    if (base_bumper.state[i] == 0)
      return 1;

  return 0;
}

static void bumper_handler(void)
{
  carmen_robot_update_skew(&bumper_average, &bumper_count, 
			   base_bumper.timestamp, base_bumper.host);
  
  bumper_ready=1;  
}

void carmen_robot_add_bumper_handler(void) 
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_ROBOT_BUMPER_NAME,
		      IPC_VARIABLE_LENGTH,
		      CARMEN_ROBOT_BUMPER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_BUMPER_NAME);

  carmen_base_subscribe_bumper_message(&base_bumper,
				       (carmen_handler_t)bumper_handler,
				       CARMEN_SUBSCRIBE_LATEST);

  carmen_running_average_clear(&bumper_average);
}

void carmen_robot_add_bumper_parameters(char *progname __attribute__ ((unused)) ) 
{

}

