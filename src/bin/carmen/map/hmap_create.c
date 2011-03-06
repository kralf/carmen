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

static char *map_filenames[1000];

carmen_hmap_t get_hmap() {
  
  char line[1024], *s, *ptr;
  carmen_hmap_t hmap;
  int i, j, k, n, len, num_points;

  printf("Enter map files: ");
  fflush(stdout);
  
  fgets(line, 1024, stdin);
  line[strcspn(line, "\n")] = '\0';
  
  s = line;
  s += strspn(s, " \t");
  n = 0;
  while (*s != '\0') {
    s += strcspn(s, " \t");
    s += strspn(s, " \t");
    n++;
  }
  
  hmap.num_zones = n;
  hmap.zone_names = (char **) calloc(n, sizeof(char *));
  carmen_test_alloc(hmap.zone_names);
  s = line + strspn(line, " \t");
  for (i = 0; *s != '\0'; i++) {
    n = strcspn(s, " \t");
    hmap.zone_names[i] = (char *) calloc(n+1, sizeof(char));
    carmen_test_alloc(hmap.zone_names[i]);
    strncpy(hmap.zone_names[i], s, n);
    hmap.zone_names[i][n] = '\0';
    if (!carmen_map_file(hmap.zone_names[i])) {
      if (strchr(hmap.zone_names[i], '.') && !strncmp(strchr(hmap.zone_names[i], '.'), ".map", 4))
	carmen_die("Error: %s does not appear to be a valid carmen map file;\n" 
		   "if it is gzipped, make sure it has a \".gz\" extension\n",
		   hmap.zone_names[i]);
      else {
	hmap.zone_names[i] = (char *) realloc(hmap.zone_names[i], (n+4+1)*sizeof(char));
	carmen_test_alloc(hmap.zone_names[i]);
	strcpy(hmap.zone_names[i] + n, ".map");
	if (!carmen_map_file(hmap.zone_names[i])) {
	  hmap.zone_names[i] = (char *) realloc(hmap.zone_names[i], (n+7+1)*sizeof(char));
	  carmen_test_alloc(hmap.zone_names[i]);
	  strcpy(hmap.zone_names[i] + n, ".map.gz");
	  if (!carmen_map_file(hmap.zone_names[i]))
	    carmen_die("Error: %s does not appear to be a valid carmen map file;\n" 
		       "if it is gzipped, make sure it has a \".gz\" extension\n",
		       hmap.zone_names[i]);
	}
      }
    }
    map_filenames[i] = carmen_new_string(hmap.zone_names[i]);
    if (strstr(hmap.zone_names[i], ".map"))
      *strchr(hmap.zone_names[i], '.') = '\0';
    s += n;
    s += strspn(s, " \t");
  }
  
  printf("Enter interface nodes in the following format:\n"
	 "<type> <degree/num maps> <map name> <points> <map name> <points> ... (degree times)\n");
  printf("where type is either DOOR or ELEVATOR and points are triples <x y theta(degrees)>\n");
  hmap.links = NULL;
  for (i = 0; 1; i++) {
    hmap.links = (carmen_hmap_link_p) realloc(hmap.links, (i+1) * sizeof(carmen_hmap_link_t));
    carmen_test_alloc(hmap.links);
    printf("Interface Node %d: ", i+1);
    fflush(stdout);
    fgets(line, 1024, stdin);
    if (*line == '\n')
      break;
    line[strcspn(line, "\n")] = '\0';
    s = line + strspn(line, " \t");

    // get link type
    len = strcspn(s, " \t");
    if (!strncmp(s, "DOOR", strlen("DOOR")))
      hmap.links[i].type = CARMEN_HMAP_LINK_DOOR;
    else if (!strncmp(s, "ELEVATOR", strlen("ELEVATOR")))
      hmap.links[i].type = CARMEN_HMAP_LINK_ELEVATOR;
    else
      carmen_die("Error: link type must be either 'DOOR' or 'ELEVATOR'\n");

    s += len;
    s += strspn(s, " \t");

    // get link degree
    hmap.links[i].degree = (int) strtol(s, &ptr, 0);
    if (hmap.links[i].degree == 0 || strspn(ptr, " \t") == 0) {
      s[strcspn(s, " \t")] = '\0';
      carmen_die("Error: invalid link degree: %s\n", s);
    }

    s += strcspn(s, " \t");
    s += strspn(s, " \t");    

    hmap.links[i].keys = (int *) calloc(hmap.links[i].degree, sizeof(int));
    carmen_test_alloc(hmap.links[i].keys);

    /*
    hmap.links[i].points = (carmen_point_t **) calloc(hmap.links[i].degree, sizeof(carmen_point_t *));
    carmen_test_alloc(hmap.links[i].points);
    */

    num_points = 0;
    switch (hmap.links[i].type) {
    case CARMEN_HMAP_LINK_DOOR:      num_points = 2; break;
    case CARMEN_HMAP_LINK_ELEVATOR:  num_points = 1; break;
    }

    hmap.links[i].num_points = num_points * hmap.links[i].degree;

    hmap.links[i].points = (carmen_point_p) calloc(hmap.links[i].num_points, sizeof(carmen_point_t));
    carmen_test_alloc(hmap.links[i].points);

    for (j = 0; j < hmap.links[i].degree; j++) {
      // get link key
      len = strcspn(s, " \t");
      for (k = 0; k < hmap.num_zones; k++)
	if (((int)strlen(hmap.zone_names[k]) == len) && !strncmp(s, hmap.zone_names[k], len))
	  break;
      if (k == hmap.num_zones) {
	s[len] = '\0';
	carmen_die("Error: map %s not found in hmap.zone_names\n", s);
      }
      hmap.links[i].keys[j] = k;

      s += strcspn(s, " \t");
      s += strspn(s, " \t");

      // get link point
      //hmap.links[i].points[j] = (carmen_point_t *) calloc(num_points, sizeof(carmen_point_t));
      //carmen_test_alloc(hmap.links[i].points[j]);
      for (k = 0; k < num_points; k++) {
	hmap.links[i].points[j*num_points+k].x = strtod(s, &ptr);
	if (strspn(ptr, " \t") == 0) {
	  s[strcspn(s, " \t")] = '\0';
	  carmen_die("Error: %s is not a valid number\n", s);
	}
	s += strcspn(s, " \t");
	s += strspn(s, " \t");
	hmap.links[i].points[j*num_points+k].y = strtod(s, &ptr);
	if (strspn(ptr, " \t") == 0) {
	  s[strcspn(s, " \t")] = '\0';
	  carmen_die("Error: %s is not a valid number\n", s);
	}
	s += strcspn(s, " \t");
	s += strspn(s, " \t");
        hmap.links[i].points[j*num_points+k].theta = carmen_degrees_to_radians(strtod(s, &ptr));
	if (strspn(ptr, " \t") == 0 &&
	    !(j == hmap.links[i].degree-1 && k == num_points-1 && *ptr == '\0')) {
	  s[strcspn(s, " \t")] = '\0';
	  carmen_die("Error: %s is not a valid number\n", s);
	}
	if (!(j == hmap.links[i].degree-1 && k == num_points-1)) {
	  s += strcspn(s, " \t");
	  s += strspn(s, " \t");
	}
      }
    }     
  }

  hmap.num_links = i;

  return hmap;
}

int main(int argc, char **argv) {

  char *out_file;
  carmen_FILE *out_fp;
  carmen_hmap_t hmap;
  int i;

  if (argc != 2)
    carmen_die("Usage: create_hmap <mapfile>\n");

  out_file = argv[1];

  out_fp = carmen_fopen(out_file, "w");
  if (out_fp == NULL)
    carmen_die("Error: Can't open file %s for writing\n", out_file);

  if(carmen_map_write_comment_chunk(out_fp, 0, 0, 0, NULL, NULL) < 0)
    return -1;
  if (carmen_map_write_id(out_fp) < 0)
    return -1;

  hmap = get_hmap();

  if (carmen_map_write_hmap_chunk(out_fp, &hmap) < 0)
    carmen_die("Error: couldn't write hmap chunk\n");

  carmen_fclose(out_fp);

  for (i = 0; i < hmap.num_zones; i++) {
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_GRIDMAP_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], out_file, CARMEN_MAP_GRIDMAP_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name gridmap chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_OFFLIMITS_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], out_file, CARMEN_MAP_OFFLIMITS_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name offlimits chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_PLACES_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], out_file, CARMEN_MAP_PLACES_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name places chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_EXPECTED_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], out_file, CARMEN_MAP_EXPECTED_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name expected chunk\n");
    if (carmen_map_chunk_exists(map_filenames[i], CARMEN_MAP_CREATOR_CHUNK))
      if (carmen_map_name_chunk(map_filenames[i], out_file, CARMEN_MAP_CREATOR_CHUNK, hmap.zone_names[i]) < 0)
	carmen_die("Error: couldn't name creator chunk\n");
  }

  printf("\n");
  return 0;
}
