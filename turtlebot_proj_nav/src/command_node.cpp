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
    ros::WallDuration duration = ros::WallDuration(0);
    
    
    // Initialization of the command FSM	    
    int rotationcommandCurrentState = 1;
    
    // Rate of the node
    ros::Rate r(FREQ); 

    while(ros::ok())
	{
	    // Launches callbacks which received messages
	    ros::spinOnce();
	    
	    
	    
	    
	    /*if(turtlebotCommand.start())
                {
                    // A mouvement has been asked on the topic meant for that purpose
                    ROS_INFO("Mouvement asked...");
                    duration = turtlebotCommand.turningDuration();
                    ROS_INFO("Duration  : %lf",duration.toSec() );
                    // Get startTime now      
		            startTime = ros::WallTime::now();
		            ROS_INFO("Begin...");
		            while((ros::WallTime::now() - startTime) < duration){
		                ROS_INFO("Turning...");
		                turtlebotCommand.turn();
	                }
                    
                    // When the duration is over, we quit the state in which the robot is turning or moving"
                    ROS_INFO("Duration over...");
                    ROS_INFO( "Turning finished...");
		            turtlebotCommand.movingOver();
                    }   
                }*/
	    
	    
	    /*switch (rotationcommandCurrentState)
        {
            //Starting and waiting state
            case 1:
                if(turtlebotCommand.start())
                {
                    // A mouvement has been asked on the topic meant for that purpose
                    ROS_INFO("Mouvement asked...");
                    rotationcommandCurrentState = 2;
                }
                break;
                
            //Duration calculus
            case 2:
                // Calculation of the turning duration
                duration = turtlebotCommand.turningDuration();
                ROS_INFO("Duration  : %lf",duration.toSec() ); 
                // Get startTime now      
		        startTime = ros::WallTime::now();
		        ROS_INFO("Begin...");
		        rotationcommandCurrentState = 3;
                break;
                
            //Mouvement
            case 3:
                if((ros::WallTime::now() - startTime) > duration)
                {
                    // If the duration is over, we quit the state in which the robot is turning or moving"
                    ROS_INFO("Duration over...");
                    rotationcommandCurrentState = 4;
                }
                else 
                {
                    ROS_INFO("Turning... "); 
                    // If it's a turning phase, we send the command to turn
		            turtlebotCommand.turn();
                    rotationcommandCurrentState = 3;
                }
                
                break; 
                
            //Duration over
            case 4:
                turtlebotCommand.movingOver();
	            rotationcommandCurrentState = 1;

                break;
                      
            default:
                rotationcommandCurrentState = 1;
                break;

        }
        // After every loop, we publish the command state again as a security
        turtlebotCommand.publishCommandState();
        
        // Synchroniztation with the rate
	    r.sleep();
	}

    return 0;
}
