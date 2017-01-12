#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>


int main(int argc, char **argv)
{
    ROS_INFO("Launching highLevelCommande_node ...\n");
    ros::init(argc, argv, "highLevelCommande_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(1); // 1Hz 

    HighLevelCommand HLC(node);


   while (ros::ok()) 
   {
         ros::spinOnce();






         loop_rate.sleep();
    }
    
    return 0;
}
