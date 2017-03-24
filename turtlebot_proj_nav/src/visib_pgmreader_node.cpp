/*
  visib_pgmreader_node.cpp
  Thibaut AGHNATIOS

  ROS Node which read pgm file of markers visibility map and return the number of markers for the position of the robot in the map
  !!! Don't work correctly and limited for 4 markers visible at the same time
 
 */

#include "ros/ros.h"
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
#include "graph.hpp"
#include "visib_init.hpp"

using namespace std;

bool markersVisibility(turtlebot_proj_nav::MarkersVisibility::Request  &req, turtlebot_proj_nav::MarkersVisibility::Response &res)
    {
        ROS_INFO("request: x=%lf, y=%lf", req.x, req.y);
        struct table image = pgm_imread("src/TurtleBot-pkgs/turtlebot_proj_nav/map/visib.pgm");
 
        int x,y;
        x = (req.x+12.2)/0.05;
        y = -(req.y-18.2)/0.05;

		// For the position of the robot (x,y), comparing the value to return the number of markers visible for this position    
        if(image.data[x][y]==12)
		{
			res.markers=1;
		}
		else if(image.data[x][y]==9)
		{
			res.markers=2;
		}
		else if(image.data[x][y]==6)
		{
			res.markers=3;
		}
		else if(image.data[x][y]==3)
		{
			res.markers=4;
		}
		else if(image.data[x][y]==15)
		{
			res.markers=0;
			ROS_INFO("No visible markers : %d", res.markers);
		}
        else
        {
            res.markers=0;
			ROS_INFO("Undetermined (5 visible markers at the same time or in the center of marker detected) : %d", res.markers);
        }
        
        ROS_INFO("Sending back visible markers : [%d]", (int)res.markers);
        return true;
    }


int main(int argc, char **argv)
{
    // ROS node init
    ROS_INFO("Launching reader visibility ...");
    ros::init(argc, argv, "visibility_pgmreader_node");
    ros::NodeHandle node;

	// Launching service which return the number of visible markers
    ros::ServiceServer serviceMarkersVisibility = node.advertiseService("/nav/visibility_marker", markersVisibility);
    ROS_INFO("Ready send visible markers.");
    ros::spin();

    return 0;
}
