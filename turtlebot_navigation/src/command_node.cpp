#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include "command.h"
#include "std_msgs/Bool.h"

#include <cmath>

#define FREQ 10 //10 Hz

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
        ROS_INFO("COMMAND RECEIVED !");
        //std::cout<<*msg<<std::endl;
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
    ROS_INFO("Launching command_node ...");
    ros::init(argc, argv, "command_node");
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
    
    ros::Subscriber subBallPos = node.subscribe<recherche_balle_tp1::command>("/nav/ball_reference", 1, boost::bind(updateBallCB, _1, &ballNav));
    ros::Publisher pubCommandState = node.advertise<std_msgs::Bool>("/nav/command_busy", 1);
       	    
    ros::WallTime startTime;
    ros::WallDuration duration;
    
    ros::Rate r(FREQ); 

    while(ros::ok())
	{
	    
	    pubCommandState.publish(ballNav.busy);
	
	    if(ballNav.start) 
		{
		       
                if(ballNav.turning) 
                    {
                        if(ballNav.angularVelocity ==0 || ballNav.angle ==0) 
                        {
                            duration = ros::WallDuration(0);
                        }
                        else duration = ros::WallDuration(std::abs(ballNav.angle/ballNav.angularVelocity));
                    }
                    
                else 
                    {
                        if(ballNav.linearVelocity == 0 || ballNav.distance == 0) 
                        {
                            duration = ros::WallDuration(0);
                        }
                        else duration = ros::WallDuration(std::abs(ballNav.distance/ballNav.linearVelocity));
                    }
                  
            ROS_INFO("Duration  : %lf\n",duration.toSec() );       
		    startTime = ros::WallTime::now();
		    ROS_INFO("Begin\n");
		    ballNav.start = false;
		}	    
	    else if(ballNav.stop)
		{
		    //ROS_INFO("STOP\n");
		    turtleBot.stop();
		    ballNav.busy.data = false;
		}
		    
	    if(ballNav.turning)
		{
		    ROS_INFO("Turning... \n"); 
		    turtleBot.turn(ballNav.angularVelocity);
		}
	    else if(ballNav.moving)
		{
		    ROS_INFO("Moving...\n"); 
		    turtleBot.move(ballNav.linearVelocity);
		}
		    
	    if((ros::WallTime::now() - startTime) > duration && (ballNav.turning || ballNav.moving)) 
		{
		    if(ballNav.turning && !ballNav.moving)
			{
			    ROS_INFO( "Turning finished\n");
			    ballNav.turning = false;
			    ballNav.moving = true;
			    ballNav.start = true;
			}
		    else
		    {
		        ROS_INFO( "All finished\n");
		        ballNav.moving = false;
			    ballNav.stop = true;
			}
		}
		
		
	    ros::spinOnce();
	    r.sleep();
	}

    return 0;
}
