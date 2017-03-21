#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include <cmath>

#define FREQ 20 //10 Hz

int main(int argc, char **argv)
{
    ROS_INFO("Launching command_node ...");
    ros::init(argc, argv, "command_node");
    ros::NodeHandle node;
     
    TurtleBotCommand turtlebotCommand(node);
    
    ros::WallTime startTime;
    ros::WallDuration duration;
       	    
    int commandCurrentState = 1;
    
    ros::Rate r(FREQ); 

    while(ros::ok())
	{
	    ros::spinOnce();
	    switch (commandCurrentState)
        {
            
            		
            //Command enable
            case 0:
                
                if(turtlebotCommand.commandEnabled())
                {
                    ROS_INFO("Command enabled...");
                    commandCurrentState = 1;
                }
                break;
            
            //Start
            case 1:
                
                if(turtlebotCommand.start())
                {
                    ROS_INFO("Mouvement asked...");
                    commandCurrentState = 2;
                }
                break;
                
            //Durations
            case 2:
                
                if(turtlebotCommand.turtleBotTurning())
                {
                    ROS_INFO("Preparation for turning...");
                    duration = turtlebotCommand.turningDuration();
                    commandCurrentState = 3;
                }
                else
                {
                    ROS_INFO("Preparation for moving...");
                    duration = turtlebotCommand.movingDuration();
                    commandCurrentState = 3;
                }
                ROS_INFO("Duration  : %lf",duration.toSec() );       
		        startTime = ros::WallTime::now();
		        ROS_INFO("Begin...");
                break;
                
            //Mouvement
            case 3:
                if((ros::WallTime::now() - startTime) > duration)
                {
                    ROS_INFO("Duration over...");
                    commandCurrentState = 4;
                }
                else if(turtlebotCommand.turtleBotTurning())
                {
                    ROS_INFO("Turning... "); 
		            turtlebotCommand.turn();
                    commandCurrentState = 3;
                }
                else if(turtlebotCommand.turtleBotMoving())
                {
                    ROS_INFO("Moving..."); 
		            turtlebotCommand.move();
                    commandCurrentState = 3;
                }
                
                break; 
                
            //Duration over
            case 4:
                
                if(turtlebotCommand.turtleBotTurning() && !turtlebotCommand.turtleBotMoving())
			    {
			        ROS_INFO( "Turning finished...");
			        turtlebotCommand.turningOver();
			        commandCurrentState = 1;
			    }
		        else
		        {
		            ROS_INFO( "All mouvements finished...");
		            turtlebotCommand.movingOver();
		            commandCurrentState = 1;
			    }
                break;
                      
            default:
                commandCurrentState = 1;
                break;

        }
        turtlebotCommand.publishCommandState();
        
	    r.sleep();
	}

    return 0;
}
