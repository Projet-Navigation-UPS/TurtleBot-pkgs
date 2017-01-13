#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>




int main(int argc, char **argv)
{
    ROS_INFO("Launching highLevelCommande_node ...");
    ros::init(argc, argv, "highLevelCommande_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(1); // 1Hz 

    HighLevelCommand HLC(node);
    
    int currentState, nextState;
    
    currentState = 0;
    //nextState = 0;

    while (ros::ok()) 
    {
        ros::spinOnce();

        switch (currentState)
        {
            case 0:
                ROS_INFO("Wait for location...");
                if(HLC.location_Ready())
                {
                    HLC.plan_Path();
                    currentState = 1;
                }
                else currentState = 0;
                break;
            case 1:
                ROS_INFO("Wait for path planning...");
                if(HLC.path_Found())
                {
                    HLC.follow_Path();
                    currentState = 2;
                }
                else currentState = 1;
                break;
            case 2:
                ROS_INFO("Wait for the command to follow the path...");
                if(HLC.command_Finished())
                {
                    currentState = 3;
                }
                else currentState = 2;
                break;
            case 3:
                ROS_INFO("You've arrived !");
                currentState = 3;
                break;
            default:
                currentState = 0;
                
                break;

        }

        loop_rate.sleep();
    }
    
    return 0;
}
