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
    
    int currentState;
    
    currentState = 0;

    while (ros::ok()) 
    {
        ros::spinOnce();

        /*switch (currentState)
        {
            case 0:
                
                if(HLC.location_Ready())
                {
                    ROS_INFO("Location ready...");
                    currentState = 1;
                }
                else 
                {
                    ROS_INFO("Wait for location...");
                    currentState = 0;
                }    
                break;
            case 1:
                if(HLC.near_Goal())
                {
                    ROS_INFO("Goal reached...");
                    currentState = 1;
                }
                else 
                {   
                    ROS_INFO("Search path to reach goal...");
                    HLC.plan_Path();
                    currentState = 2;
                }
                break;
            case 2:
                if(HLC.path_Found())
                {
                    ROS_INFO("Launch mouvement...");
                    HLC.follow_Path();
                    currentState = 3;
                }
                else
                {
                    ROS_INFO("Wait for path planning...");
                    currentState = 2;
                }
                break;
            case 3:
                if(HLC.command_Finished())
                {
                    ROS_INFO("Mouvement finished...");
                    currentState = 0;
                }
                else
                {
                    ROS_INFO("Wait for the robot to follow the path...");
                    currentState = 3;
                }
                break;
            default:
                currentState = 0;
                break;

        }*/

        loop_rate.sleep();
    }
    
    return 0;
}
