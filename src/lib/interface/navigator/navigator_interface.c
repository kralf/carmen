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

#include <carmen/carmen.h>
#ifndef NO_ZLIB
#include <zlib.h>
#endif

void
carmen_navigator_subscribe_status_message(carmen_navigator_status_message 
					  *status,
					  carmen_handler_t handler,
					  carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_NAVIGATOR_STATUS_NAME, 
                           CARMEN_NAVIGATOR_STATUS_FMT,
                           status, sizeof(carmen_navigator_status_message), 
			   handler, subscribe_how);
}

void
carmen_navigator_unsubscribe_status_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_NAVIGATOR_STATUS_NAME, handler);
}

void
carmen_navigator_subscribe_plan_message(carmen_navigator_plan_message *plan,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_NAVIGATOR_PLAN_NAME, 
                           CARMEN_NAVIGATOR_PLAN_FMT,
                           plan, sizeof(carmen_navigator_plan_message), 
			   handler, subscribe_how);
}

void
carmen_navigator_unsubscribe_plan_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_NAVIGATOR_PLAN_NAME, handler);
}

void
carmen_navigator_subscribe_autonomous_stopped_message(carmen_navigator_autonomous_stopped_message *autonomous_stopped,
						      carmen_handler_t handler,
						      carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_NAME, 
                           CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_FMT,
                           autonomous_stopped, 
			   sizeof(carmen_navigator_autonomous_stopped_message),
			   handler, subscribe_how);
}

void
carmen_navigator_unsubscribe_autonomous_stopped_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_NAVIGATOR_AUTONOMOUS_STOPPED_NAME, 
			     handler);
}

static unsigned int timeout = 500;

int
carmen_navigator_query_status(carmen_navigator_status_message **status_msg) 
{
  IPC_RETURN_TYPE err;
  carmen_navigator_status_query_message *msg;
  static int initialized = 0;

  msg = carmen_default_message_create();

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_NAVIGATOR_STATUS_QUERY_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_DEFAULT_MESSAGE_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_NAVIGATOR_STATUS_QUERY_NAME);
      initialized = 1;
    }

  err = IPC_queryResponseData(CARMEN_NAVIGATOR_STATUS_QUERY_NAME, msg, 
			      (void **)status_msg, timeout);
  carmen_test_ipc_return_int(err, "Could not query navigator status", 
			     CARMEN_NAVIGATOR_STATUS_QUERY_NAME);

  return 0;
}

int 
carmen_navigator_query_plan(carmen_navigator_plan_message **plan_msg) 
{
  IPC_RETURN_TYPE err;
  carmen_navigator_plan_query_message *msg;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_NAVIGATOR_PLAN_QUERY_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_DEFAULT_MESSAGE_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_NAVIGATOR_PLAN_QUERY_NAME);
      initialized = 1;
    }

  msg = carmen_default_message_create();

  err = IPC_queryResponseData(CARMEN_NAVIGATOR_PLAN_QUERY_NAME, msg, 
			      (void **)plan_msg, timeout);
  carmen_test_ipc_return_int(err, "Could not query plans", 
			     CARMEN_NAVIGATOR_PLAN_QUERY_NAME);
	
  return 0;
}

int 
carmen_navigator_set_goal(double x, double y) 
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_navigator_set_goal_message msg;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_NAVIGATOR_SET_GOAL_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_NAVIGATOR_SET_GOAL_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_NAVIGATOR_SET_GOAL_NAME);
      initialized = 1;
    }

  msg.x = x;
  msg.y = y;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_NAVIGATOR_SET_GOAL_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_SET_GOAL_NAME);

  return 0;
}

int 
carmen_navigator_set_goal_triplet(carmen_point_p goal) 
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_navigator_set_goal_triplet_message msg;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME);
      initialized = 1;
    }


  msg.goal = *goal;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", 
		  CARMEN_NAVIGATOR_SET_GOAL_TRIPLET_NAME);

  return 0;
}

int 
carmen_navigator_set_goal_place(char *placename)
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_navigator_placename_message msg;
  carmen_navigator_return_code_message *return_msg;
  int return_code;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_NAVIGATOR_SET_GOAL_PLACE_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME);
      initialized = 1;
    }


  msg.placename = placename;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_queryResponseData(CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME, &msg, 
			      (void **)&return_msg, timeout);
  carmen_test_ipc(err, "Could not set goal by place", 
		  CARMEN_NAVIGATOR_SET_GOAL_PLACE_NAME);

  if (err == IPC_OK) {
    if (return_msg->code) {
      carmen_warn(return_msg->error);
      free(return_msg->error);
    }

    return_code = return_msg->code;
    free(return_msg);
  } else 
    return_code = err;

  return return_code;
}

int 
carmen_navigator_stop(void) 
{
  IPC_RETURN_TYPE err;
  carmen_navigator_stop_message *msg;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_NAVIGATOR_STOP_NAME, 
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_NAVIGATOR_STOP_NAME);
    initialized = 1;
  }
  
  msg = carmen_default_message_create();

  err = IPC_publishData(CARMEN_NAVIGATOR_STOP_NAME, msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_STOP_NAME);

  return 0;
}

int 
carmen_navigator_go(void) 
{
  IPC_RETURN_TYPE err;
  carmen_navigator_go_message *msg;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_NAVIGATOR_GO_NAME, 
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_NAVIGATOR_GO_NAME);
      initialized = 1;
    }

  msg = carmen_default_message_create();

  err = IPC_publishData(CARMEN_NAVIGATOR_GO_NAME, msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_GO_NAME);

  return 0;
}


int 
carmen_navigator_get_map(carmen_navigator_map_t map_type, 
			 carmen_map_t *map) 
{
  IPC_RETURN_TYPE err;
  carmen_navigator_map_request_message msg;
  carmen_navigator_map_message *response = NULL;
  int index;

#ifndef NO_ZLIB
  int uncompress_return;
  int uncompress_size;
  int uncompress_size_result;
  unsigned char *uncompressed_data;
#endif

  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_NAVIGATOR_MAP_REQUEST_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_NAVIGATOR_MAP_REQUEST_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_NAVIGATOR_MAP_REQUEST_NAME);
      initialized = 1;
    }

  msg.map_type = map_type;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_queryResponseData(CARMEN_NAVIGATOR_MAP_REQUEST_NAME, &msg, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not get map", CARMEN_NAVIGATOR_MAP_REQUEST_NAME);

#ifndef NO_ZLIB
  if (response && response->compressed) 
    {
      uncompress_size = response->config.x_size*
	response->config.y_size;
      uncompressed_data = (unsigned char *)
	calloc(uncompress_size, sizeof(float));
      carmen_test_alloc(uncompressed_data);
      uncompress_size_result = uncompress_size*sizeof(float);
      uncompress_return = uncompress((void *)uncompressed_data,   
				     (uLong *)&uncompress_size_result,
				     (void *)response->data, 
				     response->size);
      response->data = uncompressed_data;
      response->size = uncompress_size_result;
    }
#else
  if (response && response->compressed) 
    {
      carmen_warn("Received compressed map from server. This program was\n"
		  "compiled without zlib support, so this map cannot be\n"
		  "used. Sorry.\n");
      free(response->data);
      free(response);
      response = NULL;
    }
#endif

  if (response)
    {
      if (map)
	{
	  map->config = response->config;
	  map->complete_map = (float *)response->data;
	  map->map = (float **)calloc(map->config.x_size, sizeof(float));
	  carmen_test_alloc(map->map);
	  
	  for (index = 0; index < map->config.x_size; index++)
	    map->map[index] = map->complete_map+index*map->config.y_size;
	}
      else
	free(response->data);
      free(response);
    } 

  return 0;
}
