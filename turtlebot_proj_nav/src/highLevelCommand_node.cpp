#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>



int main(int argc, char **argv)
{
    ROS_INFO("Launching highLevelCommande_node ...");
    
    //Return usage if the node is not properly launched
    if (argc != 5)
    {
        ROS_ERROR("USAGE: highLevelCommande_node X_goal Y_goal XY_threshold XY_precision");
        return 1;
    }
    
    ros::init(argc, argv, "highLevelCommande_node");
    ros::NodeHandle node;
    // Rate of the node
    ros::Rate loop_rate(1); // 1Hz 

    // Object which provides hight level commands to control navigation behavior
    HighLevelCommand HLC(node,atof(argv[1]),atof(argv[2]));
    
    // Initialization of FSM
    int hlcCurrentState = 0;
    
    while (ros::ok()) 
    {
        // Launches callbacks which received messages
        ros::spinOnce();
        
        // FSM
        switch (hlcCurrentState)
        {
            //Initializarion
            case 0:
                ROS_INFO("Init..."); 
                // If localization is available
                if(HLC.location()) 
                {
                    // Calculate the closest marker to the goal
                    // If there is a closest marler according to the XY_threshold
                    if(HLC.init(atof(argv[3])) != -1) hlcCurrentState = 1;
                    else hlcCurrentState = 7;
                }
                break;
            
            //Visible markers
            case 1:
                // If there is more than one visibible marker
                //if(HLC.markersVisibility() > 0)
                // The service doesn't work yet so we assume that there are always at least one visible marker
                if(true) 
                {
                    ROS_INFO("In visiblity zone...");
                    hlcCurrentState = 3;
                }
                else if(HLC.markersVisibility() == -1)
                {
                    // Service call failed
                    hlcCurrentState = 1;
                }
                else
                {
                    ROS_INFO("Not in visiblity zone...");
                    hlcCurrentState = 9;
                } 
                break;

            //Perception
            case 2:
                // If a markers is seen
                if(HLC.marker() != -1)
                {
                    ROS_INFO("Marker seen...");
                    hlcCurrentState = 4;
                }
                else
                {
                    ROS_INFO("Seeking marker...");
                    // Calls a FSM to seek markers
                    HLC.seekMarker();
                    hlcCurrentState = 3;
                } 
                break;
            
            //AskForMarker
            case 3: 
                // If there is a resonse from marker's detection               
                if(HLC.markerResponse()) hlcCurrentState = 2;
                // If there is no request yet and the robot is not moving
                else if(!HLC.getAskMarker() && !HLC.getCommandBusy())
                { 
                    ROS_INFO("Ask for marker...");
                    // Ask to location_node if it sees a marker
                    HLC.askForMarker();
                }
                break;
            
            //Location
            case 4:
                // Wait the new localisation after seen a marker
                if(HLC.location())
                {
                    ROS_INFO("Location ready...");
                    hlcCurrentState = 5;
                }
                else 
                {
                    ROS_INFO("Wait for location...");
                    hlcCurrentState = 4;
                }    
                break;
            
            //Marker Goals    
            case 5:
                // Test is the closest marker to the goal is reached
                if(HLC.finalMarkerGoal())
                {
                    hlcCurrentState = 7;
                    ROS_INFO("Final Marker Goal reached...");
                }
                else 
                {  
                    // If it's not, send a goal toward a next marker 
                    ROS_INFO("Send goal...");
                    HLC.sendGoal();
                    hlcCurrentState = 6;
                }
                break;
            
            //Movement    
            case 6:
                // If the movement is aborted, star over to testing visibility zone
                if(HLC.getGoalAborted()) hlcCurrentState = 1;
                // While the intermediate goal is not reached, the robot is moving
                else if(!HLC.intermediateGoal()) ROS_INFO("Moving...");
                else 
                {
                    ROS_INFO("Intermediate goal reached...");
                    hlcCurrentState = 3;
                }
                break;
            
            //Final marker goal reached    
            case 7:
                ROS_INFO("Send last goal...");
                // Send goal to X_goal Y_goal
                HLC.sendGoal();
                hlcCurrentState = 8;
                break;
            
            //Final Goal reaching
            case 8:
                // While the final goal is not reached, the robot is moving
                if(!HLC.intermediateGoal()) ROS_INFO("Moving...");
                else 
                {
                    ROS_INFO("Final goal reached...");
                    // If the goal is reahced with a precision under XY_precision, a special sound is played
                    HLC.finalGoal(atof(argv[4]));
                    hlcCurrentState = 10;
                };
                break; 
            
            //Find visibility zone
            case 9:
                ROS_INFO("Seeking a visibility zone...");
                // Not implemented yet...
                hlcCurrentState = 9;
                break; 
                
            //End
            case 10:
                // The supervision is closed
                return 1; 
                break; 
                   
            default:
                hlcCurrentState = 0;
                break;

        }

        loop_rate.sleep();
    }
    
    return 0;
}
