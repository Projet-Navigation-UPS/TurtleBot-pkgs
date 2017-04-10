/*
  odom_node.cpp
  Bruno Dato & Thibaut Aghnatios

  ROS Node to command the turtlebot to get odometry errors.
  The command is a square
 
 */
#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include "Odom.hpp"
#include "std_msgs/Bool.h"
#include <cmath>

#define FREQ 20 //20 Hz


int main(int argc, char **argv)
{
    ROS_INFO("Launching odom_tests_node ...");
    ros::init(argc, argv, "odom_tests_node");
    ros::NodeHandle node;
    
    // Command and odometry objects 
    TurtleBotCommand turtleBot(node);
	Odom odom(node);

    // Starting time and duration   	    
    ros::WallTime startTime;
    ros::WallDuration durationLine, durationAngle, durationWait(10);
    
    // Frequency
    ros::Rate r(FREQ); 
    
    // FSM initialization
    int currentState = 0;
    bool start = false;
    
    // Set velocities, angle and distance
    float angularVelocity = 1.5;
    float linearVelocity = 0.2;
    float angle = 3.1416/2;
    float distance = 1.0;
    
    // Sensors message
    kobuki_msgs::SensorState tic;
    int ticR = tic.right_encoder;
    int ticL = tic.left_encoder;
    
    
    // Turning duration
    if (angularVelocity<0) durationAngle = ros::WallDuration(-angle/angularVelocity);
    else durationAngle = ros::WallDuration(angle/angularVelocity);
    
    // Moving duration
    if (linearVelocity<0) durationLine = ros::WallDuration(-distance/linearVelocity);
    else durationLine = ros::WallDuration(distance/linearVelocity);

    // Initialization displays
    std::cout<<"angularVelocity : "<<angularVelocity<<std::endl;
    std::cout<<"linearVelocity : "<<linearVelocity<<std::endl;
    std::cout<<"angle : "<<angle<<std::endl;
    std::cout<<"distance : "<<distance<<std::endl;
    std::cout<<"durationAngle : "<<durationAngle<<std::endl;
    std::cout<<"durationLine : "<<durationLine<<std::endl;
    // Wheels encoders
	odom.displayMobileBaseSenorsCore();

    while(ros::ok())
	{
	    //std::cout<<"currentState : "<<currentState<<std::endl;
	    switch (currentState)
        {
        
        //Moving in line 
        case 0:
            //Start line phase
            if(!start) 
            {
                //Start time of the mouvement
                startTime = ros::WallTime::now();
                start = true;
		        ROS_INFO("Before Moving...");
		        //Get wheels encoders before the mouvement
		        odom.displayMobileBaseSenorsCore();
		        ROS_INFO("Moving...");
            }
            //Make the robot move
            turtleBot.move(linearVelocity);
            
            //When duration line is over, quit the moving state
            if ((ros::WallTime::now() - startTime) > durationLine ) 
            {
                currentState = 3;
                start = false;  
		
            }    
            break;
            
        //Turning    
        case 1:
            // Start turning phase
            if(!start) 
            {
                //Start time of the mouvement
                startTime = ros::WallTime::now();
                start = true;
		        ROS_INFO("Before Turning...");
		        //Get wheels encoders before the mouvement
		        odom.displayMobileBaseSenorsCore();
		        ROS_INFO("Turning...");
            }
            //Make the robot turn
            turtleBot.turn(angularVelocity);
	
            //When duration angle is over, quit the turning state
            if ((ros::WallTime::now() - startTime) > durationAngle ) 
            {
                currentState = 2; 
                start = false;    
            }
            break;
        //Waiting after turning
	    case 2:
            if(!start) 
            {
                //Start time of wait
                startTime = ros::WallTime::now();
	            ROS_INFO("After Turning...");
	            //Get wheels encoders after turning
	            odom.displayMobileBaseSenorsCore();
                start = true;
            }


            //When duration wait is over switch to moving phase
            if ((ros::WallTime::now() - startTime) > durationWait) 
            {
                currentState = 0;
                start = false;   
            }
            break;
       //Waiting after moving
	   case 3:
		    if(!start) 
            {
                //Start time of wait
                startTime = ros::WallTime::now();
				ROS_INFO("After Moving...");
				//Get wheels encoders after moving
				odom.displayMobileBaseSenorsCore();
                start = true;
            }
            
			
            //When duration wait is over switch to turning phase
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
		
	    ros::spinOnce();
	    r.sleep();
	}

    return 0;
}
