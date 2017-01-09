#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include "command.h"
#include "std_msgs/Bool.h"

#include <cmath>

#define FREQ 10

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
        std::cout<<*msg<<std::endl;
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
    std::cout<<"Launching command_node ..."<<std::endl;
    ros::init(argc, argv, "command");
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
    
    ros::Rate r(FREQ); // 10 hz

    while(ros::ok())
	{
	    /*std::cout << "############################################################" << std::endl;
	    std::cout << "linearVelocity : " << ballNav.linearVelocity << std::endl;
	    std::cout << "angularVelocity : " << ballNav.angularVelocity << std::endl;
	    std::cout << "distance : " << ballNav.distance << std::endl;
	    std::cout << "angle : " << ballNav.angle << std::endl;
	    std::cout << "drop : " << ballNav.drop << std::endl;
	    std::cout << "turning : " << ballNav.turning << std::endl;
	    std::cout << "moving : " << ballNav.moving << std::endl;
	    std::cout << "stop : " << ballNav.stop << std::endl;
	    std::cout << "start : " << ballNav.start << std::endl;
	    std::cout << "busy : " << ballNav.busy << std::endl;*/
	    
	    pubCommandState.publish(ballNav.busy);
	
	    if(ballNav.start) 
		{
		    
		    
                if(ballNav.turning) 
                    {
                        if(ballNav.angularVelocity ==0 || ballNav.angle ==0) 
                        {
                            //ballNav.turning = false;
                            duration = ros::WallDuration(0);
                        }
                        else duration = ros::WallDuration(std::abs(ballNav.angle/ballNav.angularVelocity));
                    }
                    
                else 
                    {
                        if(ballNav.linearVelocity == 0 || ballNav.distance == 0) 
                        {
                            //ballNav.moving = false;
                            duration = ros::WallDuration(0);
                        }
                        else duration = ros::WallDuration(std::abs(ballNav.distance/ballNav.linearVelocity));
                    }
                  
            std::cout << "Duration  : " << duration << std::endl;       
		    startTime = ros::WallTime::now();
		    std::cout << "Begin : " << startTime << std::endl;
		    ballNav.start = false;
		}	    
	    else if(ballNav.stop)
		{
		    //std::cout << "STOP : " << ros::WallTime::now() << std::endl;
		    turtleBot.stop();
		    ballNav.busy.data = false;
		}
		    
	    if(ballNav.turning)
		{
		    std::cout << "Turning  : " << ros::WallTime::now() << std::endl; 
		    turtleBot.turn(ballNav.angularVelocity);
		}
	    else if(ballNav.moving)
		{
		    std::cout << "Moving  : " << ros::WallTime::now() << std::endl; 
		    turtleBot.move(ballNav.linearVelocity);
		}
		    
	    if((ros::WallTime::now() - startTime) > duration && (ballNav.turning || ballNav.moving)) 
		{
		    if(ballNav.turning && !ballNav.moving)
			{
			    std::cout << "Turning finished : " << ros::WallTime::now() << std::endl;
			    ballNav.turning = false;
			    ballNav.moving = true;
			    ballNav.start = true;
			}
		    else
		    {
		        std::cout << "All finished : " << ros::WallTime::now() << std::endl;
		        ballNav.moving = false;
			    ballNav.stop = true;
			}
		}
		
		
	    ros::spinOnce();
	    r.sleep();
	}

    return 0;
}
