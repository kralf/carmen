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

#include "map_io.h"

#include "localize_interface.h"

static int simulation = 0;
static carmen_hmap_t hmap;
static int map_zone;
carmen_localize_globalpos_message global_pos;
static int global_pos_reset = 1;

static void set_robot_pose(carmen_point_t pose) {

  carmen_point_t std = {0.2, 0.2, carmen_degrees_to_radians(4.0)};
  carmen_localize_initialize_gaussian_command(pose, std);
  if (simulation)
    carmen_simulator_set_truepose(&pose);
  global_pos_reset = 1;
}

static carmen_point_t transform_robot_pose(carmen_point_t pose0,
					   carmen_point_t pose1) {
  carmen_point_t robot_pose;
  carmen_point_t offset;

  offset.x = pose1.x - pose0.x;
  offset.y = pose1.y - pose0.y;
  offset.theta = pose1.theta - pose0.theta;

  robot_pose = global_pos.globalpos;
  robot_pose.x -= pose0.x;
  robot_pose.y -= pose0.y;
  robot_pose.theta -= pose0.theta;
  robot_pose.x = cos(offset.theta)*robot_pose.x - sin(offset.theta)*robot_pose.y;
  robot_pose.y = sin(offset.theta)*robot_pose.x + cos(offset.theta)*robot_pose.y;
  robot_pose.x += pose0.x;
  robot_pose.y += pose0.y;
  robot_pose.theta += pose0.theta;
  robot_pose.x += offset.x;
  robot_pose.y += offset.y;
  robot_pose.theta += offset.theta;
  robot_pose.theta = carmen_normalize_theta(robot_pose.theta);

  return robot_pose;
}

static carmen_hmap_link_p get_closest_link(int type) {
  
  int i, j, imin;
  double d, dmin;
  carmen_hmap_link_p link = NULL;

  imin = -1;
  dmin = 0;

  if (type == CARMEN_HMAP_LINK_ELEVATOR) {
    for (i = 0; i < hmap.num_links; i++) {
      link = &hmap.links[i];
      if (link->type == CARMEN_HMAP_LINK_ELEVATOR) {
	for (j = 0; j < link->degree; j++) {
	  if (link->keys[j] == map_zone) {
	    d = carmen_distance(&global_pos.globalpos, &link->points[j]);
	    if (imin < 0 || d < dmin) {
	      imin = i;
	      dmin = d;
	    }
	    break;
	  }
	}
      }
    }
    if (imin < 0)
      link = NULL;
    else
      link = &hmap.links[imin];
  }

  return link;
}

static void level_up() {

  carmen_hmap_link_p link;
  int i;

  printf("\nlevel_up()\n");

  link = get_closest_link(CARMEN_HMAP_LINK_ELEVATOR);
  
  if (link == NULL || link->keys[link->degree-1] == map_zone)
    return;

  for (i = 0; i < link->degree; i++)
    if (link->keys[i] == map_zone)
      break;

  carmen_map_change_map_zone(hmap.zone_names[link->keys[i+1]]);
  carmen_ipc_sleep(1.0);
  set_robot_pose(transform_robot_pose(link->points[i], link->points[i+1]));
}

static void level_down() {

  carmen_hmap_link_p link;
  int i;

  printf("\nlevel_down()\n");

  link = get_closest_link(CARMEN_HMAP_LINK_ELEVATOR);
  
  if (link == NULL || link->keys[0] == map_zone)
    return;

  for (i = 0; i < link->degree; i++)
    if (link->keys[i] == map_zone)
      break;

  carmen_map_change_map_zone(hmap.zone_names[link->keys[i-1]]);
  carmen_ipc_sleep(1.0);
  set_robot_pose(transform_robot_pose(link->points[i], link->points[i-1]));
}

static void map_zone_handler(char **zone_name) {

  int i;

  for (i = 0; i < hmap.num_zones; i++) {
    if (!strcmp(hmap.zone_names[i], *zone_name)) {
      map_zone = i;
      break;
    }
  }

  printf("map zone changed: %s\n", *zone_name);

  global_pos_reset = 1;  
}


// returns > 0 if p3 and p4 are on same side of line that goes through p1 and p2, and returns < 0 otherwise
static double line_side_test(carmen_point_t p1, carmen_point_t p2, carmen_point_t p3, carmen_point_t p4) {

  p2.x -= p1.x;
  p3.x -= p1.x;
  p4.x -= p1.x;
  p2.y -= p1.y;
  p3.y -= p1.y;
  p4.y -= p1.y;

  return (p2.x*p3.y - p2.y*p3.x)*(p2.x*p4.y - p2.y*p4.x);
}

static int segment_intersect(carmen_point_t p1, carmen_point_t p2, carmen_point_t p3, carmen_point_t p4) {

  return (line_side_test(p1, p2, p3, p4) < 0 && line_side_test(p3, p4, p1, p2) < 0);
}

static void hmap_warp_check(carmen_point_t old_pose, carmen_point_t new_pose) {

  int i;
  double x1, x2, x3, x4, y1, y2, y3, y4;
  static int warping = 0;
  static char *warp_zone;
  static carmen_point_t warp_pose;
  static carmen_point_t warp_src;
  static carmen_point_t warp_dst;
  static carmen_point_t warp_post1;
  static carmen_point_t warp_post2;

  if (warping) {
    if (line_side_test(warp_post1, warp_post2, warp_pose, new_pose) < 0) {
      if (carmen_distance(&warp_pose, &new_pose) > 0.5) {  //dbug: param?
	carmen_map_change_map_zone(warp_zone);
	carmen_ipc_sleep(1.0);
	set_robot_pose(transform_robot_pose(warp_src, warp_dst));
	warping = 0;
      }
    }
    else
      warping = 0;
  }
  else {
    for (i = 0; i < hmap.num_links; i++) {
      if (hmap.links[i].type == CARMEN_HMAP_LINK_DOOR) {
	if (hmap.links[i].keys[0] == map_zone) {
	  if (segment_intersect(old_pose, new_pose, hmap.links[i].points[0], hmap.links[i].points[1])) {
	    warping = 1;
	    warp_zone = hmap.zone_names[hmap.links[i].keys[1]];
	    warp_pose = old_pose;
	    x1 = hmap.links[i].points[0].x;
	    x2 = hmap.links[i].points[1].x;
	    x3 = hmap.links[i].points[2].x;
	    x4 = hmap.links[i].points[3].x;
	    y1 = hmap.links[i].points[0].y;
	    y2 = hmap.links[i].points[1].y;
	    y3 = hmap.links[i].points[2].y;
	    y4 = hmap.links[i].points[3].y;
	    warp_src.x = (x1 + x2) / 2.0;
	    warp_src.y = (y1 + y2) / 2.0;
	    warp_src.theta = atan2(y2-y1, x2-x1);
	    warp_dst.x = (x3 + x4) / 2.0;
	    warp_dst.y = (y3 + y4) / 2.0;
	    warp_dst.theta = atan2(y4-y3, x4-x3);
	    warp_post1 = hmap.links[i].points[0];
	    warp_post2 = hmap.links[i].points[1];
	  }
	}
	else if (hmap.links[i].keys[1] == map_zone) {
	  if (segment_intersect(old_pose, new_pose, hmap.links[i].points[2], hmap.links[i].points[3])) {
	    warping = 1;
	    warp_zone = hmap.zone_names[hmap.links[i].keys[0]];
	    warp_pose = old_pose;
	    x1 = hmap.links[i].points[0].x;
	    x2 = hmap.links[i].points[1].x;
	    x3 = hmap.links[i].points[2].x;
	    x4 = hmap.links[i].points[3].x;
	    y1 = hmap.links[i].points[0].y;
	    y2 = hmap.links[i].points[1].y;
	    y3 = hmap.links[i].points[2].y;
	    y4 = hmap.links[i].points[3].y;
	    warp_dst.x = (x1 + x2) / 2.0;
	    warp_dst.y = (y1 + y2) / 2.0;
	    warp_dst.theta = atan2(y2-y1, x2-x1);
	    warp_src.x = (x3 + x4) / 2.0;
	    warp_src.y = (y3 + y4) / 2.0;
	    warp_src.theta = atan2(y4-y3, x4-x3);
	    warp_post1 = hmap.links[i].points[2];
	    warp_post2 = hmap.links[i].points[3];
	  }
	}
      }
    }
  }
}

static void localize_handler(carmen_localize_globalpos_message *new_global_pos) {

  carmen_point_t old_pose = global_pos.globalpos;

  printf(".");
  fflush(0);

  global_pos = *new_global_pos;

  if (!global_pos_reset) {
    if (carmen_distance(&old_pose, &global_pos.globalpos) < 1.0)  // dbug: param?
      hmap_warp_check(old_pose, global_pos.globalpos);
  }
  else
    global_pos_reset = 0;
}

static void ipc_init() {

  carmen_localize_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_handler,
					      CARMEN_SUBSCRIBE_LATEST);
  carmen_map_subscribe_map_zone_message(NULL, (carmen_handler_t) map_zone_handler, CARMEN_SUBSCRIBE_LATEST);
}

void shutdown_module(int sig) {
  carmen_ipc_disconnect();
  
  sig = 0;
  exit(0);
}

void print_hmap() {

  int i, j;

  for (i = 0; i < hmap.num_zones; i++)
    printf("%s ", hmap.zone_names[i]);
  printf("\n");
  printf("num_links = %d\n\n", hmap.num_links);
  for (i = 0; i < hmap.num_links; i++) {
    if (hmap.links[i].type == CARMEN_HMAP_LINK_ELEVATOR)
      printf("type = ELEVATOR, ");
    else
      printf("type = DOOR, ");
    printf("degree = %d\n", hmap.links[i].degree);
    printf("keys = ");
    for (j = 0; j < hmap.links[i].degree; j++)
      printf("%d ", hmap.links[i].keys[j]);
    printf("\n");
    printf("points = ");
    for (j = 0; j < hmap.links[i].num_points; j++)
      printf("(%.2f, %.2f, %.2f) ", hmap.links[i].points[j].x, hmap.links[i].points[j].y,
	     hmap.links[i].points[j].theta);
    printf("\n\n");
  }
}

int main(int argc, char** argv) {

  int c;

  carmen_ipc_initialize(argc,argv);
  carmen_param_check_version(argv[0]);
  signal(SIGINT, shutdown_module);

  if (argc > 1 && !strcmp(argv[1], "-sim"))
    simulation = 1;

  if (carmen_map_get_hmap(&hmap) < 0)
    carmen_die("no hmap found\n");
  print_hmap(); //dbug
  carmen_map_change_map_zone(hmap.zone_names[0]);  //dbug: this should go in the map server
  map_zone = 0;

  ipc_init();

  carmen_terminal_cbreak(0);

  while(1) {
    carmen_ipc_sleep(0.01);
    c = getchar();
    /*
    if (c == EOF)
      break;
    */
    if (c != 27)
      continue;
    c = getchar();
    /*
    if (c == EOF)
      break;
    */
    if (c != 91)
      continue;
    c = getchar();
    /*
    if (c == EOF)
      break;
    */
    switch(c) {
    case 65: level_up(); break;
    case 66: level_down();
    }
  }
   
  return 0;
}
