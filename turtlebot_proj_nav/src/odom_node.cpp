#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include "Odom.hpp"
#include "command.h"
#include "std_msgs/Bool.h"

#include <cmath>

#define FREQ 20 //10 Hz
//#define LINEAR_VELOCITY 0.25 //m.s-1 max 0.25
//#define ANGULAR_VELOCITY 3.0 //rad.s-1  max 2




int main(int argc, char **argv)
{
    ROS_INFO("Launching odom_tests_node ...");
    ros::init(argc, argv, "odom_tests_node");
    ros::NodeHandle node;
     
    TurtleBotCommand turtleBot(node);
	Odom odom(node);

       	    
    ros::WallTime startTime;
    ros::WallDuration durationLine, durationAngle;
    
    ros::Rate r(FREQ); 
    
    int currentState = 0;
    bool start = false;
    
    float angularVelocity = 4.0;
    float linearVelocity = 0.2;
    float angle = 3.1416/2;
    float distance = 1.0;
    
    
    if (angularVelocity<0) durationAngle = ros::WallDuration(-angle/angularVelocity);
    else durationAngle = ros::WallDuration(angle/angularVelocity);
    
    if (linearVelocity<0) durationLine = ros::WallDuration(-distance/linearVelocity);
    else durationLine = ros::WallDuration(distance/linearVelocity);

    std::cout<<"angularVelocity : "<<angularVelocity<<std::endl;
    std::cout<<"linearVelocity : "<<linearVelocity<<std::endl;
    std::cout<<"angle : "<<angle<<std::endl;
    std::cout<<"distance : "<<distance<<std::endl;
    std::cout<<"durationAngle : "<<durationAngle<<std::endl;
    std::cout<<"durationLine : "<<durationLine<<std::endl;
	//std::cout<<"ticR: "<<odomy.getMobileBaseSensorsCore()<<std::endl;
	//std::cout<<"ticL: "<<odomy.left_encoder<<std::endl;

    while(ros::ok())
	{
	    
	    //std::cout<<"currentState : "<<currentState<<std::endl;
	    switch (currentState)
        {
            case 0:
                if(!start) 
                {
                    startTime = ros::WallTime::now();
                    start = true;
                }
                turtleBot.move(linearVelocity);
                //ROS_INFO("Moving...");
                if ((ros::WallTime::now() - startTime) > durationLine ) 
                {
                    currentState = 1;
                    start = false;    
                }    
                break;
            case 1:
                if(!start) 
                {
                    startTime = ros::WallTime::now();
                    start = true;
                }
                turtleBot.move(angularVelocity);
				
                //ROS_INFO("Turning...");
                if ((ros::WallTime::now() - startTime) > durationAngle ) 
                {
                    currentState = 0;
                    start = false;  
					//std::cout<<"ticR: "<<right_encoder<<std::endl;
					//std::cout<<"ticL: "<<left_encoder<<std::endl;  
                }
                break;
            default:
                currentState = 0;
                break;
		}
		
	    ros::spinOnce();
	    r.sleep();
	}

    return 0;
}
