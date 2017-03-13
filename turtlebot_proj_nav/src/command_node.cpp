#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include <cmath>

#define FREQ 10 //10 Hz

int main(int argc, char **argv)
{
    ROS_INFO("Launching command_node ...");
    ros::init(argc, argv, "command_node");
    ros::NodeHandle node;
     
    TurtleBotCommand turtlebotCommand(node);
    
    ros::WallTime startTime;
    ros::WallDuration duration;
       	    
    int commandCurrentState = 0;
    
    ros::Rate r(FREQ); 

    while(ros::ok())
	{
	    
	    switch (commandCurrentState)
        {
            ros::spinOnce();
            		
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
                ROS_INFO("Duration  : %lf\n",duration.toSec() );       
		        startTime = ros::WallTime::now();
		        ROS_INFO("Begin...\n");
                break;
                
            //Mouvement
            case 3:
                if(turtlebotCommand.turtleBotTurning())
                {
                    ROS_INFO("Turning... \n"); 
		            turtlebotCommand.turn();
                    commandCurrentState = 3;
                }
                else if(turtlebotCommand.turtleBotMoving())
                {
                    ROS_INFO("Moving...\n"); 
		            turtlebotCommand.move();
                    commandCurrentState = 3;
                }
                else if((ros::WallTime::now() - startTime) > duration)
                {
                    ROS_INFO("Duration over...\n");
                    commandCurrentState = 4;
                }
                break; 
                
            //Duration over
            case 4:
                
                if(turtlebotCommand.turtleBotTurning() && !turtlebotCommand.turtleBotMoving())
			    {
			        ROS_INFO( "Turning finished...\n");
			        turtlebotCommand.turningOver();
			        commandCurrentState = 1;
			    }
		        else
		        {
		            ROS_INFO( "All mouvements finished...\n");
		            turtlebotCommand.movingOver();
		            commandCurrentState = 5;
			    }
                break;
                
            case 5:
                
                if(turtlebotCommand.stop2())
			    {
			        turtlebotCommand.stop();
		            commandCurrentState = 5;
			    }
		        else
		        {
		             commandCurrentState = 0;
			    }
                break;
                      
            default:
                commandCurrentState = 5;
                break;

        }
	    r.sleep();
	}

    return 0;
}
