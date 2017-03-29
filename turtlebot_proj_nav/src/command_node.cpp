/*
  command_node.cpp
  Bruno Dato and Tristan Klempka

  ROS Node to command the turtlebot in distance and angle.
  This node was at first made for searching a ball but it's also used to seek a marker 
 
 */
#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include <cmath>

#define FREQ 10 //10 Hz

int main(int argc, char **argv)
{
    ROS_INFO("Launching command_node ...");
    ros::init(argc, argv, "command_node");
    ros::NodeHandle node;
    
    // Object which provides the functions to command in speed a turtlebot 
    // Here we will only use turning on itself and moving forward
    TurtleBotCommand turtlebotCommand(node);
    
    // Time when a mouvement will start
    ros::WallTime startTime;
    // Duration of a mouvement 
    ros::WallDuration duration;
    
    // Initialization of the command FSM	    
    int commandCurrentState = 1;
    
    // Rate of the node
    ros::Rate r(FREQ); 

    while(ros::ok())
	{
	    // Launches callbacks which received messages
	    ros::spinOnce();
	    
	    switch (commandCurrentState)
        {
            //Starting and waiting state
            case 1:
                if(turtlebotCommand.start())
                {
                    // A mouvement has been asked on the topic meant for that purpose
                    ROS_INFO("Mouvement asked...");
                    commandCurrentState = 2;
                }
                break;
                
            //Duration calculus
            case 2:
                if(turtlebotCommand.turtleBotTurning())
                {
                    ROS_INFO("Preparation for turning...");
                    // Calculation of the turning duration
                    duration = turtlebotCommand.turningDuration();
                    commandCurrentState = 3;
                }
                else
                {
                    ROS_INFO("Preparation for moving...");
                    // Calculation of the linear moving duration
                    duration = turtlebotCommand.movingDuration();
                    commandCurrentState = 3;
                }
                ROS_INFO("Duration  : %lf",duration.toSec() ); 
                // Get startTime now      
		        startTime = ros::WallTime::now();
		        ROS_INFO("Begin...");
                break;
                
            //Mouvement
            case 3:
                if((ros::WallTime::now() - startTime) > duration)
                {
                    // If the duration is over, we quit the state in which the robot is turning or moving"
                    ROS_INFO("Duration over...");
                    commandCurrentState = 4;
                }
                else if(turtlebotCommand.turtleBotTurning())
                {
                    ROS_INFO("Turning... "); 
                    // If it's a turning phase, we send the command to turn
		            turtlebotCommand.turn();
                    commandCurrentState = 3;
                }
                else if(turtlebotCommand.turtleBotMoving())
                {
                    ROS_INFO("Moving..."); 
                    // If it's a moving phase, we send the command to move forward
		            turtlebotCommand.move();
                    commandCurrentState = 3;
                }
                
                break; 
                
            //Duration over
            case 4:
                
                if(turtlebotCommand.turtleBotTurning() && !turtlebotCommand.turtleBotMoving())
			    {
			        ROS_INFO( "Turning finished...");
			        // If the turning phase is over and the moving phase has not passed yet
			        turtlebotCommand.turningOver();
			        commandCurrentState = 1;
			    }
		        else
		        {
		            ROS_INFO( "All mouvements finished...");
		            // Turning and moving phase are over
		            turtlebotCommand.movingOver();
		            commandCurrentState = 1;
			    }
                break;
                      
            default:
                commandCurrentState = 1;
                break;

        }
        // After every loop, we publish the command state again as a security
        turtlebotCommand.publishCommandState();
        
        // Synchroniztation with the rate
	    r.sleep();
	}

    return 0;
}
