/*
  visib_init.hpp
  Thibaut AGHNATIOS

  Header file 
 
 */

#ifndef _VISIB_INIT_HPP_
#define _VISIB_INIT_HPP_


#include "HighLevelCommand.hpp"
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <math.h> 
#include <std_msgs/Int8.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#define pi 3.14159265358979323846

#include <fstream>

#include "tinyxml.h"
#include <ros/package.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <vector>
#include <string>

struct table	//image structure
	{
	int **data;
	int cols;
	int rows;
};

void displayGraphVisib(Graph g, float x1[], float y1[], float t[]);

void Ecriture_carte_visib();

table pgm_imread(char *argv);

#endif
