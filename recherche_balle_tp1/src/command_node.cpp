#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include "recherche_balle_tp1/command.h"

#define FREQ 10

void updateBallCB(const recherche_balle_tp1::command::ConstPtr& msg)
{
    if(!busy)
	{
	    bool busy = true;
	    linearVelocity = msg->linearVelocity;
	    angularVelocity = msg->angularVelocity;
	    distance = msg->distance;
	    angle = msg->angle;

	    // start by turning
	    duration = abs(angle/angularVelocity);	    
	    bool turning = true;
	    bool moving = false;
	    bool stop = false;
	    bool start = true;
	    
	    ros::WallTime startTime;
	    ros::WallDuration duration(duration);
	    bool done = false;
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
		}
	    busy = true;
	}
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commande");
    ros::NodeHandle node;
    ros::Rate loop_rate(FREQ);
        
    ros::Subscriber subBallPos = node.subscribe("/nav/ballref", 1000, updateBallCB);
               
    while(ros::ok())
	{
	    //Launching Callbacks and synchronizing with loop_rate
	    ros::spinOnce(); 
	    loop_rate.sleep();
	}
    return 0;
}
