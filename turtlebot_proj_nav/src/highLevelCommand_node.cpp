#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>



int main(int argc, char **argv)
{
    ROS_INFO("Launching highLevelCommande_node ...");
    
    //Return usage if the node is not properly launched
    if (argc != 6)
    {
        ROS_ERROR("USAGE: highLevelCommande_node X_GOAL Y_GOAL XY_THRESHOLD XY_PRECISION DISTANCE_TO_MARKERS");
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
                ROS_INFO("[STATE %d] - Init...",hlcCurrentState); 
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
                    ROS_INFO("[STATE %d] - In visiblity zone...",hlcCurrentState);
                    hlcCurrentState = 3;
                }
                else if(HLC.markersVisibility() == -1)
                {
                    // Service call failed
                    hlcCurrentState = 1;
                }
                else
                {
                    ROS_INFO("[STATE %d] - Not in visiblity zone...",hlcCurrentState);
                    hlcCurrentState = 9;
                } 
                break;

            //Perception
            case 2:
                // If a markers is seen
                if(HLC.markerID() != -1)
                {
                    ROS_INFO("[STATE %d] - Marker seen...",hlcCurrentState);
                    hlcCurrentState = 4;
                }
                else
                {
                    ROS_INFO("[STATE %d] - Seeking marker...",hlcCurrentState);
                    // Calls a FSM to seek markers
                    HLC.seekMarker();
                    hlcCurrentState = 3;
                } 
                break;
            
            //AskForMarker
            case 3: 
                // If there is a resonse from marker's detection               
                if(HLC.getMarkerResponse()) hlcCurrentState = 2;
                // If there is no request yet and the robot is not moving
                else if(!HLC.getMarkerAsked() && !HLC.getCommandState())
                { 
                    ROS_INFO("[STATE %d] - Ask for marker...",hlcCurrentState);
                    // Ask to location_node if it sees a marker
                    HLC.askForMarker();
                }
                break;
            
            //Location
            case 4:
                // Wait the new localisation after seen a marker
                if(HLC.location())
                {
                    ROS_INFO("[STATE %d] - Location ready...",hlcCurrentState);
                    hlcCurrentState = 5;
                }
                else 
                {
                    ROS_INFO("[STATE %d] - Wait for location...",hlcCurrentState);
                    hlcCurrentState = 4;
                }    
                break;
            
            //Marker Goals    
            case 5:
                // Test if the closest marker to the goal is reached
                if(HLC.finalMarkerGoal())
                {
                    ROS_INFO("[STATE %d] - Final Marker Goal reached...",hlcCurrentState);
                    hlcCurrentState = 7;
                }
                else 
                {  
                    // If it's not, send a goal towards a next marker 
                    ROS_INFO("[STATE %d] - Send goal...",hlcCurrentState);
                    HLC.sendMarkerGoal(atof(argv[5]));
                    hlcCurrentState = 6;
                }
                break;
            
            //Movement    
            case 6:
                // If the movement is aborted, star over to testing visibility zone
                if(HLC.getGoalAborted()) hlcCurrentState = 1;
                // While the intermediate goal is not reached, the robot is moving
                else if(!HLC.intermediateGoal()) ROS_INFO("[STATE %d] - Moving...",hlcCurrentState);
                else 
                {
                    ROS_INFO("[STATE %d] - Intermediate goal reached...",hlcCurrentState);
                    hlcCurrentState = 1;
                }
                break;
            
            //Final marker goal reached    
            case 7:
                ROS_INFO("[STATE %d] - Send last goal...",hlcCurrentState);
                // Send goal to X_goal Y_goal
                HLC.sendFinalGoal();
                hlcCurrentState = 8;
                break;
            
            //Final Goal reaching
            case 8:
                // While the final goal is not reached, the robot is moving
                if(!HLC.intermediateGoal()) ROS_INFO("[STATE %d] - Moving...",hlcCurrentState);
                else 
                {
                    ROS_INFO("[STATE %d] - Final goal reached...",hlcCurrentState);
                    // If the goal is reahced with a precision under XY_precision, a special sound is played
                    HLC.finalGoal(atof(argv[4]));
                    hlcCurrentState = 10;
                };
                break; 
            
            //Find visibility zone
            case 9:
                ROS_INFO("[STATE %d] - Seeking a visibility zone...",hlcCurrentState);
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

        // Synchronize with rate
        loop_rate.sleep();
    }
    
    return 0;
}
