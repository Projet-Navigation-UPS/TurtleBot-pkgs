#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>



int main(int argc, char **argv)
{
    ROS_INFO("Launching highLevelCommande_node ...");
    ros::init(argc, argv, "highLevelCommande_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(0.5); // 1Hz 

    HighLevelCommand HLC(node,0);
    
    int hlcCurrentState;
    
    hlcCurrentState = -2;

    while (ros::ok()) 
    {
        ros::spinOnce();
        switch (hlcCurrentState)
        {
            //Init
            case -2:
                ROS_INFO("Init...");
                hlcCurrentState = -1;    
                break;
        
            //Visible markers
            case -1:
                //if((HLC.markersVisibility() != -1) && (HLC.markersVisibility() != 0))
                if(true)
                {
                    ROS_INFO("In visiblity zone...");
                    hlcCurrentState = 1;
                }
                else if(HLC.markersVisibility() == -1)
                {
                    hlcCurrentState = -1;
                }
                else
                {
                    ROS_INFO("Not in visiblity zone...");
                    hlcCurrentState = 7;
                } 
                break;

            //Perception
            case 0:
                if(HLC.marker() != -1)
                {
                    ROS_INFO("Marker seen...");
                    hlcCurrentState = 2;
                }
                else
                {
                    ROS_INFO("Seeking marker...");
                    HLC.seekMarker();
                    hlcCurrentState = 1;
                } 
                break;
            
            //AskForMarker
            case 1:
                //std::cout<<HLC.getAskMarker()<<std::endl;
                //std::cout<<HLC.markerResponse()<<std::endl;
                
                if(HLC.markerResponse()) hlcCurrentState = 0;
                else if(!HLC.getAskMarker())
                { 
                    ROS_INFO("AskForMarker...");
                    HLC.askForMarker();
                }
                break;
            
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
                else 
                {
                    ROS_INFO("Intermediate goal reached...");
                    hlcCurrentState = 1;
                }
                break;
            
            //Final goal reached    
            case 5:
                ROS_INFO("Goal reached...");
                hlcCurrentState = 6;
                break;
            
            //Find new global goal
            case 6:
                ROS_INFO("Choosing new global Goal...");
                hlcCurrentState = 0;
                break; 
            
            //Find visibility zone
            case 7:
                ROS_INFO("Seeking a visibility zone...");
                hlcCurrentState = 7;
                break; 
                   
            default:
                hlcCurrentState = 0;
                HLC.findGlobalGoal();
                break;

        }

        //usleep(2000000);
        loop_rate.sleep();
    }
    
    return 0;
}
