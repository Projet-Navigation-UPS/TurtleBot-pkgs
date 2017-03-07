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
    ros::WallDuration durationLine, durationAngle, durationWait(10);
    kobuki_msgs::SensorState tic;
    ros::Rate r(FREQ); 
    
    int currentState = 0;
    bool start = false;
    

    float angularVelocity =4.0;
    float linearVelocity = 0.2;
    float angle = 3.1416/2;
    float distance = .4;
	int ticR= tic.right_encoder;
    int ticL= tic.left_encoder;
    
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
	//std::cout<<"ticR:"<<ticR<<std::endl;
	//std::cout<<"ticL:"<<ticL<<std::endl;

	//std::cout<<"ticL: "<<odomy.left_encoder<<std::endl;

	

    while(ros::ok())
	{	
        ros::spinOnce();

	    //std::cout<<"currentState : "<<currentState<<std::endl;
	    switch (currentState)
        {
            case 0:
                if(!start) 
                {
                    startTime = ros::WallTime::now();
                    start = true;
					ROS_INFO("Before Moving...");
					odom.displayMobileBaseSenorsCore();
					ROS_INFO("Moving...");
                }
                turtleBot.move(linearVelocity);
                
				//ticR= tic.right_encoder;
	    		//std::cout<<"ticR:"<<ticR<<std::endl;
				ticL= tic.left_encoder;
	    			//std::cout<<"ticL:"<<ticL<<std::endl;
                if ((ros::WallTime::now() - startTime) > durationLine ) 
                {
                    currentState = 3;
                    start = false;  
	  				//ticR= tic.right_encoder;
	    			//std::cout<<"ticR:"<<ticR<<std::endl;
					
                }    
                break;
            case 1:
                if(!start) 
                {
                    startTime = ros::WallTime::now();
                    start = true;
					ROS_INFO("Before Turning...");
					odom.displayMobileBaseSenorsCore();
					ROS_INFO("Turning...");
                }
                turtleBot.turn(angularVelocity);
				
                
                if ((ros::WallTime::now() - startTime) > durationAngle ) 
                {
                    currentState = 2; //18008  10276
                    start = false;    //14379  13876 turn gauche
			//ROS_INFO("Turning...");
					//std::cout<<"ticR: "<<right_encoder<<std::endl;
					//std::cout<<"ticL: "<<left_encoder<<std::endl;  
                }
                break;
	    case 2:
		if(!start) 
                {
                    startTime = ros::WallTime::now();
					ROS_INFO("After Turning...");
					odom.displayMobileBaseSenorsCore();
                    start = true;
                }
                
				
                //ROS_INFO("Waiting...");
                if ((ros::WallTime::now() - startTime) > durationWait) 
                {
                    currentState = 0;
                    start = false;   
                }
                break;
	   case 3:
		if(!start) 
                {
                    startTime = ros::WallTime::now();
					ROS_INFO("After Moving...");
					odom.displayMobileBaseSenorsCore();
                    start = true;
                }
                
				
                //ROS_INFO("Waiting2...");
                if ((ros::WallTime::now() - startTime) > durationWait ) 
                {
                    currentState = 1;
                    start = false;   
                }
                break;
            default:
                currentState = 0;
                break;
		}
		
	    
	    r.sleep();
	}

    return 0;
}
