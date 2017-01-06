#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include "command.h"
#include "std_msgs/Bool.h"

#include <cmath>

#define FREQ 2

typedef struct ballNav
{
    float linearVelocity;
    float angularVelocity;
    float distance;
    float angle;
    bool drop;
    bool turning;
    bool moving;
    bool stop;
    bool start;
    std_msgs::Bool busy;

} BallNav;

void updateBallCB(const recherche_balle_tp1::command::ConstPtr& msg, BallNav* ballNav)
{
	    ballNav->linearVelocity = msg->linearVelocity;
	    ballNav->angularVelocity = msg->angularVelocity;
	    ballNav->distance = msg->distance;
	    ballNav->angle = msg->angle;
	    ballNav->turning = true;
	    ballNav->moving = false;
	    ballNav->stop = false;
	    ballNav->start = true;
	    ballNav->busy.data = true;

} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commande");
    ros::NodeHandle node;
     
    TurtleBotCommand turtleBot(node);

    BallNav ballNav;
    ballNav.linearVelocity = 0;
    ballNav.angularVelocity = 0;
    ballNav.distance = 0;
    ballNav.angle = 0;
    ballNav.busy.data = false;
    ballNav.drop = false;
    ballNav.turning = false;
    ballNav.moving = false;
    ballNav.stop = true;
    ballNav.start = false;
    
    ros::Subscriber subBallPos = node.subscribe<recherche_balle_tp1::command>("/nav/ballref", 1, boost::bind(updateBallCB, _1, &ballNav));
    ros::Publisher pubCommandState = node.advertise<std_msgs::Bool>("/nav/CommandState", 1);
       	    
    ros::WallTime startTime;
    ros::WallDuration duration;
    
    ros::Rate r(FREQ); // 10 hz

    while(ros::ok())
	{
	    if(ballNav.start) 
		{
		    std::cout << "Begin : " << startTime << std::endl;
                    if(ballNav.turning) duration = ros::WallDuration(std::abs(ballNav.angle/ballNav.angularVelocity));
                    else duration = ros::WallDuration(std::abs(ballNav.distance/ballNav.linearVelocity));
		    startTime = ros::WallTime::now();
		    ballNav.start = false;
		}	    
	    else if(ballNav.stop)
		{
		    std::cout << "STOP : " << ros::WallTime::now() << std::endl;
		    turtleBot.stop();
		    ballNav.busy.data = false;
		}
		    
	    if(ballNav.turning)
		{
		    turtleBot.turn(ballNav.angularVelocity);
		}
	    else if(ballNav.moving)
		{
		    turtleBot.move(ballNav.linearVelocity);
		}
		    
	    if((ros::WallTime::now() - startTime) > duration) 
		{
		    if(ballNav.turning && !ballNav.moving)
			{
			    ballNav.turning = false;
			    ballNav.moving = true;
			    ballNav.start = true;
			}
		    else
			ballNav.stop = true;
		}
	    ros::spinOnce();
	    r.sleep();
	}

    return 0;
}
