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

#ifndef NO_ZLIB
#include <zlib.h>
#endif

#include "planner.h"
#include "navigator_ipc.h"
#include "navigator.h"

void carmen_navigator_publish_status(void)
{
  carmen_planner_status_t status;
  carmen_navigator_status_message status_msg;
  IPC_RETURN_TYPE err;

  status_msg.timestamp = carmen_get_time();
  status_msg.host = carmen_get_host();
  carmen_planner_get_status(&status);

  status_msg.autonomous = carmen_navigator_autonomous_status();
  status_msg.goal_set = status.goal_set;
  status_msg.goal = status.goal;
  status_msg.robot = status.robot;

  err = IPC_publishData(CARMEN_NAVIGATOR_STATUS_NAME, &status_msg);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_NAVIGATOR_STATUS_NAME);

  if (status.path.length > 0)
    free(status.path.points);
}

void carmen_navigator_publish_autonomous_stopped(carmen_navigator_reason_t
						 reason)
{
  carmen_navigator_autonomous_stopped_message msg;
  IPC_RETURN_TYPE err;

  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();
  msg.reason = reason;

  err = IPC_publishData(CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_NAME, &msg);
  carmen_test_ipc_exit(err, "Could not publish",
		       CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_NAME);
}

static void navigator_status_query_handler(MSG_INSTANCE msgRef,
					   BYTE_ARRAY callData,
					   void *clientData __attribute__
					   ((unused)))
{
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;
  carmen_navigator_status_message status_msg;
  carmen_planner_status_t status;

  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_freeByteArray(callData);

  status_msg.timestamp = carmen_get_time();
  status_msg.host = carmen_get_host();
  carmen_planner_get_status(&status);

  status_msg.autonomous = carmen_navigator_autonomous_status();
  status_msg.goal_set = status.goal_set;
  status_msg.goal = status.goal;
  status_msg.robot = status.robot;

  err = IPC_respondData(msgRef, CARMEN_NAVIGATOR_STATUS_NAME, &status_msg);
  carmen_test_ipc(err, "Could not respond", CARMEN_NAVIGATOR_STATUS_NAME);

  if (status.path.length > 0)
    free(status.path.points);
}

static int paths_are_same(carmen_planner_path_p path1,
			  carmen_planner_path_p path2)
{
  carmen_traj_point_t point1, point2;
  int index;

  if (path1->length != path2->length)
    return 0;

  for (index = 0; index < path1->length; index++)
    {
      point1 = path1->points[index];
      point2 = path2->points[index];
      if (fabs(point1.x - point2.x) > 0.1 || fabs(point1.y - point2.y) > 0.1)
	return 0;
    }
  return 1;
}

void carmen_navigator_publish_plan(void)
{
  carmen_planner_status_t status;
  carmen_navigator_plan_message plan_msg;
  IPC_RETURN_TYPE err;
  /* cyrill, 10.07.2003 */
  static carmen_point_t prev_goal        = {-1.0, -1.0, -1.0};

  static carmen_planner_path_t prev_path = {NULL, 0, 0};

  plan_msg.timestamp = carmen_get_time();
  plan_msg.host = carmen_get_host();

  carmen_planner_get_status(&status);

  /* cyrill 10.07.2003: if 1st goal was unreachable and 2nd goal also,
                        there is no planner_status_msg published
			(and that's bad)! Solution: check if goals
			have changed!  */

  if (paths_are_same(&(status.path), &prev_path) &&
      (prev_goal.x == status.goal.x && prev_goal.y == status.goal.y))
    {
      if (status.path.length > 0)
	free(status.path.points);
      return;
    }
  else
    {
      /* cyrill 10.07.2003, update old goal */
      prev_goal = status.goal;

      if (prev_path.length > 0)
	free(prev_path.points);
      prev_path.length = status.path.length;
      prev_path.points = status.path.points;
      prev_path.capacity = status.path.length;
    }

  plan_msg.path_length = status.path.length;
  plan_msg.path = status.path.points;

  err = IPC_publishData(CARMEN_NAVIGATOR_PLAN_NAME, &plan_msg);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_NAVIGATOR_PLAN_NAME);
}

static void navigator_plan_query_handler(MSG_INSTANCE msgRef,
					 BYTE_ARRAY callData,
					 void *clientData __attribute__
					 ((unused)))
{
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;
  carmen_navigator_plan_message plan_msg;
  carmen_planner_status_t status;

  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_freeByteArray(callData);

  plan_msg.timestamp = carmen_get_time();
  plan_msg.host = carmen_get_host();

  carmen_planner_get_status(&status);

  plan_msg.path_length = status.path.length;
  plan_msg.path = status.path.points;

  err = IPC_respondData(msgRef, CARMEN_NAVIGATOR_PLAN_NAME, &plan_msg);
  carmen_test_ipc(err, "Could not respond", CARMEN_NAVIGATOR_PLAN_NAME);

  if (status.path.length > 0)
    free(status.path.points);
}


static void navigator_set_goal_triplet_handler(MSG_INSTANCE msgRef,
					       BYTE_ARRAY callData,
					       void *clientData
					       __attribute__ ((unused)))
{
  carmen_navigator_set_goal_triplet_message goal_msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err = IPC_OK;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &goal_msg,
			   sizeof(carmen_navigator_set_goal_triplet_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall",
			 IPC_msgInstanceName(msgRef));

  carmen_navigator_goal_triplet(&goal_msg.goal);
}

static void navigator_set_goal_handler(MSG_INSTANCE msgRef,
				       BYTE_ARRAY callData,
				       void *clientData
				       __attribute__ ((unused)))
{
  carmen_navigator_set_goal_message goal_msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err = IPC_OK;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &goal_msg,
			   sizeof(carmen_navigator_set_goal_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall",
			 IPC_msgInstanceName(msgRef));

  carmen_verbose("Got new goal at %f %f\n", goal_msg.x, goal_msg.y);

  carmen_navigator_goal(goal_msg.x, goal_msg.y);
}

static void navigator_set_goal_place_handler(MSG_INSTANCE msgRef,
					     BYTE_ARRAY callData,
					     void *clientData
					     __attribute__ ((unused)))
{
  carmen_navigator_placename_message robot_msg;
  carmen_navigator_return_code_message response;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err = IPC_OK;
  int return_code;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &robot_msg,
			   sizeof(carmen_navigator_placename_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall",
			 IPC_msgInstanceName(msgRef));

  return_code = carmen_navigator_goal_place(robot_msg.placename);

  if (return_code == 0) {
    response.code = 0;
    response.error = NULL;
  } else {
    response.code = 1;
    response.error = carmen_new_string("No such place");
  }

  response.timestamp = carmen_get_time();
  response.host = carmen_get_host();

  err = IPC_respondData(msgRef, CARMEN_NAVIGATOR_RETURN_CODE_NAME, &response);
  carmen_test_ipc(err, "Could not respond", CARMEN_NAVIGATOR_RETURN_CODE_NAME);

  if (response.error != NULL)
    free(response.error);
}

static void navigator_stop_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
				   void *clientData __attribute__ ((unused)))
{
  carmen_navigator_stop_message stop_msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &stop_msg,
			   sizeof(carmen_default_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall",
			 IPC_msgInstanceName(msgRef));

  carmen_navigator_stop_autonomous();
}


static void navigator_go_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
				 void *clientData __attribute__ ((unused)))
{
  carmen_navigator_go_message go_msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &go_msg,
			   sizeof(carmen_default_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall",
			 IPC_msgInstanceName(msgRef));

  carmen_navigator_start_autonomous();
}

static void navigator_map_request_handler(MSG_INSTANCE msgRef,
					  BYTE_ARRAY callData,
					  void *clientData
					  __attribute__ ((unused)))
{
  carmen_navigator_map_request_message request_msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;
  carmen_navigator_map_message *map_msg;

#ifndef NO_ZLIB
  uLong compress_buf_size;
  int compress_return;
  unsigned char *compressed_map;
#endif

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &request_msg,
			   sizeof(carmen_navigator_map_request_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall",
			 IPC_msgInstanceName(msgRef));

  map_msg = carmen_planner_get_map_message(request_msg.map_type);

#ifndef NO_ZLIB
  compress_buf_size = map_msg->size*1.01+12;
  compressed_map = (unsigned char *)
    calloc(compress_buf_size, sizeof(unsigned char));
  carmen_test_alloc(compressed_map);
  compress_return = compress((void *)compressed_map,
			     (uLong *)&compress_buf_size,
			     (void *)map_msg->data,
			     (uLong)map_msg->size);
  if (compress_return != Z_OK) {
    free(compressed_map);
    map_msg->compressed = 0;
  } else {
    free(map_msg->data);
    map_msg->size = compress_buf_size;
    map_msg->data = compressed_map;
    map_msg->compressed = 1;
  }
#else
  map_msg->compressed = 0;
#endif

  map_msg->timestamp = carmen_get_time();
  map_msg->host = carmen_get_host();

  err = IPC_respondData(msgRef, CARMEN_NAVIGATOR_MAP_NAME, map_msg);
  carmen_test_ipc(err, "Could not respond", CARMEN_NAVIGATOR_MAP_NAME);

  free(map_msg->data);
  free(map_msg);
}

int carmen_navigator_initialize_ipc(void)
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_NAVIGATOR_STATUS_QUERY_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_STATUS_QUERY_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_STATUS_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_STATUS_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_STATUS_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_PLAN_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_PLAN_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_PLAN_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_PLAN_QUERY_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_PLAN_QUERY_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_SET_GOAL_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_SET_GOAL_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_SET_GOAL_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME,
		      IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME,
		      IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_SET_GOAL_PLACE_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_STOP_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_STOP_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_NAME,
		      IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_GO_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_GO_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_MAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_MAP_REQUEST_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_MAP_REQUEST_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_MAP_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_MAP_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_MAP_NAME);

  err = IPC_defineMsg(CARMEN_NAVIGATOR_RETURN_CODE_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_NAVIGATOR_RETURN_CODE_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_NAVIGATOR_RETURN_CODE_NAME);

  /* setup incoming message handlers */
  err = IPC_subscribe(CARMEN_NAVIGATOR_STATUS_QUERY_NAME,
		      navigator_status_query_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_STATUS_QUERY_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_STATUS_QUERY_NAME, 100);

  err = IPC_subscribe(CARMEN_NAVIGATOR_PLAN_QUERY_NAME,
		      navigator_plan_query_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_PLAN_QUERY_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_PLAN_QUERY_NAME, 100);

  err = IPC_subscribe(CARMEN_NAVIGATOR_SET_GOAL_NAME,
		      navigator_set_goal_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_SET_GOAL_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_SET_GOAL_NAME, 1);

  err = IPC_subscribe(CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME,
		      navigator_set_goal_triplet_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME, 1);

  err = IPC_subscribe(CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME,
		      navigator_set_goal_place_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME, 1);

  err = IPC_subscribe(CARMEN_NAVIGATOR_GO_NAME, navigator_go_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_GO_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_GO_NAME, 1);

  err = IPC_subscribe(CARMEN_NAVIGATOR_STOP_NAME, navigator_stop_handler,
		      NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_STOP_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_STOP_NAME, 1);

  err = IPC_subscribe(CARMEN_NAVIGATOR_MAP_REQUEST_NAME,
		      navigator_map_request_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_MAP_REQUEST_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_MAP_REQUEST_NAME, 1);

  /* set up localize initialization message */

  err = IPC_defineMsg(CARMEN_LOCALIZE_INITIALIZE_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_INITIALIZE_FMT);
  carmen_test_ipc_exit(err, "Could not define message",
		       CARMEN_LOCALIZE_INITIALIZE_NAME);

  return 0;
}
