#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include "recherche_balle_tp1/command.h"

#include <cmath.h>

#define FREQ 2



void updateBallCB(const recherche_balle_tp1::command::ConstPtr& msg)
{
    float linearVelocity = msg->linearVelocity;
    float angularVelocity = msg->angularVelocity;
    float distance = msg->distance;
    float angle = msg->angle;

    // start by turning
    float duration = std::abs(angle/angularVelocity);	    
    bool turning = true;
    bool moving = false;
    bool stop = false;
    bool start = true;
	    
    ros::WallTime startTime;
    ros::WallDuration duration(duration);
    bool done = false;
    ros::Rate r(FREQ); // 10 hz
    while(!done)
	{
	    if(start) 
		{
		    std::cout << "Begin : " << time << std::endl;
		    startTime = ros::WallTime::now();
		    start = false;
		}	    
	    else if(stop)
		{
		    std::cout << "STOP : " << ros::WallTime::now() << std::endl;
		    turtleBot.stop();
		    done = false;
		}
		    
	    if(turning)
		{
		    turtleBot.turn(angularVelocity);
		}
	    else if(moving)
		{
		    turtleBot.move(linearVelocity);
		}
		    
	    if((ros::WallTime::now() - time) > duration) 
		{
		    if(turning && !moving)
			{
			    duration = abs(distance/linearVelocity);
			    turning = false;
			    moving = true;
			    start = true;
			}
		    else
			stop = true;
		}
	    r.sleep();
	}

} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commande");
    ros::NodeHandle node;
     
    busy = false;
    ros::Subscriber subBallPos = node.subscribe("/nav/ballref", 1000, updateBallCB);
    
    ros::spin();
 
    return 0;
}
