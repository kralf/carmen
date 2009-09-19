#ifndef LOGGER_INTERFACE_H_
#define LOGGER_INTERFACE_H_

#include "logger_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

#define          ODOM_ID                          0
#define          FRONTLASER_ID                    1
#define          ROBOT_FRONTLASER_ID              2
#define          POSITION_ID                      3
#define          ROBOT_OLD_FRONTLASER_ID          4
#define          REARLASER_ID                     5
#define          ROBOT_REARLASER_ID               6
#define          ROBOT_OLD_REARLASER_ID           7
#define          SYNC_ID                          8
#define          PARAM_ID                         9
#define          VERSION_ID                      10

void carmen_logger_subscribe_comment_message(carmen_logger_comment_message *msg, 
					     carmen_handler_t t_handler,
					     carmen_subscribe_t subscribe_how );
  
void carmen_logger_comment(char *text);


#ifdef __cplusplus
}
#endif

#endif 
