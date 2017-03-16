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
	int x=410,y=360;
	
	//int x1=920;
	//int y1=900;

    ROS_INFO("Launching reader visibility ...");
    ros::init(argc, argv, "visibility_pgmreader_node");
    ros::NodeHandle node;

	ros::Publisher pubVisib(node.advertise<std_msgs::Int8>("/nav/visibility_marker", 1));

	std_msgs::Int8 visib;
	visib.data=0;

   	ros::Rate loop_rate(5); // 2Hz 

	struct table image = pgm_imread("src/TurtleBot-pkgs/turtlebot_proj_nav/map/visib.pgm");

	printf("%d\n",image.data[x][y]);

	              
         
    while (ros::ok()) 
    {
		ros::spinOnce();
    	
        // !!! recup les positions x et y du robot et les mettre ici

        if(image.data[x][y]==12)
					{
						visib.data=1;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						//printf("Numéro de l amer détectable : 0 \n");						
					}
					else if(image.data[x][y]==9)
					{
						visib.data=2;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						//printf("Numéros des amers détectables : %d et %d \n",k-1,k);
					}
					else if(image.data[x][y]==6)
					{
						visib.data=3;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						//printf("Numéros des amers détectables : %d, %d et %d \n",k-1,k,k+1);
					}
					else if(image.data[x][y]==3)
					{
						visib.data=4;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						//printf("Numéros des amers détectables : %d, %d, %d et %d \n",k-1,k,k+1,k+2);
					}
					else if(image.data[x][y]==15)
					{
						visib.data=0;
						ROS_INFO("Pas d amer visible : %d", visib.data);
						pubVisib.publish(visib);
					}

		cout << "Ending reader visibility " << endl;
		
		loop_rate.sleep();
    }
	return 0;
}
