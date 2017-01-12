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
                ROS_INFO("Wait location...");
                currentState = 0;
                //if(HLC.locationReady())
                {
                    HLC.ask_Path();
                    currentState = 1;
                }
                break;
            case 1:
                ROS_INFO("Wait path...");
                currentState = 1;
                if(HLC.path_Found())
                {
                    HLC.ask_Command();
                    currentState = 2;
                }
                break;
            case 2:
                ROS_INFO("Wait command...");
                currentState = 2;
                if(HLC.command_Finished())
                {
                    currentState = 3;
                }
                break;
            case 3:
                ROS_INFO("You've arrived !");
                currentState = 3;
                break;
            default:
                ROS_INFO("FATAL ERROR... ");
                break;

        }
        


        HLC.publish();
        loop_rate.sleep();
    }
    
    return 0;
}
