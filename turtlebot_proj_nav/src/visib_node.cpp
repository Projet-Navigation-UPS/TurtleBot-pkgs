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


int main(int argc, char **argv)
{
	int r=0;

    ROS_INFO("Launching visibility ...");
    ros::init(argc, argv, "visib");
    ros::NodeHandle node;

	ros::Publisher pubVisib(node.advertise<std_msgs::Int8>("/nav/Visib", 1));

	std_msgs::Int8 visib;
	visib.data=0;

   	ros::Rate loop_rate(5); // 2Hz 

	Ecriture_carte_visib();
	

    while (ros::ok() && r<1) 
    {
		ros::spinOnce();
    	
		cout << "Ending visibility " << endl;
		r=1;
		loop_rate.sleep();
    }
	return 0;
}
