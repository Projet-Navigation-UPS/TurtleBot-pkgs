#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>



int main(int argc, char **argv)
{
    ROS_INFO("Launching highLevelCommande_node ...");
    ros::init(argc, argv, "highLevelCommande_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(0.5); // 1Hz 

    HighLevelCommand HLC(node,4);
    
    int hlcCurrentState;
    
    hlcCurrentState = 0;

    while (ros::ok()) 
    {
        

        switch (hlcCurrentState)
        {
            //Location
            case 0:
                
                if(HLC.location())
                {
                    ROS_INFO("Location ready...");
                    hlcCurrentState = 1;
                }
                else 
                {
                    ROS_INFO("Wait for location...");
                    hlcCurrentState = 0;
                }    
                break;
            
            //Goals    
            case 1:
                if(HLC.finalGoal())
                {
                    hlcCurrentState = 3;
                }
                else 
                {   
                    ROS_INFO("Send goal...");
                    HLC.sendGoal();
                    hlcCurrentState = 2;
                }
                break;
            
            //Movement    
            case 2:
                if(!HLC.intermediateGoal()) ROS_INFO("Moving...");
                else hlcCurrentState = 0;
                break;
            
            //Final goal reached    
            case 3:
                ROS_INFO("Goal reached...");
                hlcCurrentState = 4;
                break;
            
            //Find new global goal
            case 4:
                ROS_INFO("Choosing new global Goal...");
                hlcCurrentState = 0;
                break;    
            default:
                hlcCurrentState = 0;
                HLC.findGlobalGoal();
                break;

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
