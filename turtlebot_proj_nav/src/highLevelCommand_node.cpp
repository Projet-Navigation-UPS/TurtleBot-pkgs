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
            //Marker
            case 0:
                
                if(HLC.marker())
                {
                    ROS_INFO("Marker seen...");
                    hlcCurrentState = 1;
                }
                else 
                {
                    ROS_INFO("Seeking marker...");
                    HLC.seekMarker();
                    hlcCurrentState = 0;
                }    
                break;
            
            //Location
            case 1:
                
                if(HLC.location())
                {
                    ROS_INFO("Location ready...");
                    hlcCurrentState = 2;
                }
                else 
                {
                    ROS_INFO("Wait for location...");
                    hlcCurrentState = 1;
                }    
                break;
            
            //Goals    
            case 2:
                if(HLC.finalGoal())
                {
                    hlcCurrentState = 4;
                }
                else 
                {   
                    ROS_INFO("Send goal...");
                    HLC.sendGoal();
                    hlcCurrentState = 3;
                }
                break;
            
            //Movement    
            case 3:
                if(!HLC.intermediateGoal()) ROS_INFO("Moving...");
                else hlcCurrentState = 0;
                break;
            
            //Final goal reached    
            case 4:
                ROS_INFO("Goal reached...");
                HLC.playSound(SOUND_CLEANINGEND);
                hlcCurrentState = 5;
                break;
            
            //Find new global goal
            case 5:
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
