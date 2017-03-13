#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>



int main(int argc, char **argv)
{
    ROS_INFO("Launching highLevelCommande_node ...");
    ros::init(argc, argv, "highLevelCommande_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(1); // 1Hz 

    HighLevelCommand HLC(node,4);
    
    int hlcCurrentState;
    
    hlcCurrentState = 0;

    while (ros::ok()) 
    {
        

        switch (hlcCurrentState)
        {
            //Perception
            case 0:
                
                if(HLC.marker()==1)
                {
                    ROS_INFO("Marker seen...");
                    hlcCurrentState = 2;
                }
                else if(HLC.marker()==0) 
                {
                    ROS_INFO("Seeking marker...");
                    HLC.seekMarker();
                    hlcCurrentState = 0;
                }
                else 
                {
                    ROS_INFO("Waiting for perception...");
                    hlcCurrentState = 0;
                }    
                break;
            
            //Marker
            /*case 1:
                
                if(HLC.marker())
                {
                    ROS_INFO("Marker seen...");
                    hlcCurrentState = 1;
                }
                else 
                {
                    ROS_INFO("Seeking marker...");
                    HLC.seekMarker();
                    hlcCurrentState = 6;
                }    
                break;*/
            
            //Location
            case 2:
                
                if(HLC.location())
                {
                    ROS_INFO("Location ready...");
                    hlcCurrentState = 3;
                }
                else 
                {
                    ROS_INFO("Wait for location...");
                    hlcCurrentState = 2;
                }    
                break;
            
            //Goals    
            case 3:
                if(HLC.finalGoal())
                {
                    hlcCurrentState = 5;
                }
                else 
                {   
                    ROS_INFO("Send goal...");
                    HLC.sendGoal();
                    hlcCurrentState = 4;
                }
                break;
            
            //Movement    
            case 4:
                if(!HLC.intermediateGoal()) ROS_INFO("Moving...");
                else hlcCurrentState = 0;
                break;
            
            //Final goal reached    
            case 5:
                ROS_INFO("Goal reached...");
                HLC.playSound(SOUND_CLEANINGEND);
                hlcCurrentState = 6;
                break;
            
            //Find new global goal
            case 6:
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
