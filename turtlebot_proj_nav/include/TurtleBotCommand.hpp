#ifndef _TURTLEBOTCOMMAND_
#define _TURTLEBOTCOMMAND_

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

#ifndef pi
   #define pi 3.14159265358979323846
#endif

const float ROBOT_LINEAR_MAX_VELOCITY = 0.25;
const float ROBOT_ANGULAR_MAX_VELOCITY = 2;

const float ROBOT_MAX_LINEAR_VELOCITY = 0.5f;
const float ROBOT_MAX_ANGULAR_VELOCITY = 3.14f;

class TurtleBotCommand 
{    
private:
        
    //Publishers
	ros::Publisher publisherMobileBaseCommandsVelocity;
	ros::Publisher publisherOdom;
	ros::Publisher publisherIm;
	ros::Publisher publisherWheel;
    
    //Messages
    geometry_msgs::Twist mobileBaseCommandsVelocity;
	nav_msgs::Odometry odom;    
	sensor_msgs::Imu im;    
	sensor_msgs::JointState wheel;

    ros::NodeHandle& m_node;
public:

    TurtleBotCommand(ros::NodeHandle& node);
    ~TurtleBotCommand();
    
    //Motion
    void stop();
    void move(const float linearVelocity, const float mile);
    void turn(const float angularVelocity, const float angu);
	//void folcom(std::vector<std::vector<const float> > tabp, const float linearVelocity, const float angularVelocity);

    //Diplays
    void displayMobileBaseCommandsVelocity();
    void moveAndTurn(const float linearVelocity, const float angularVelocity);

private:
    geometry_msgs::Twist getMobileBaseCommandsVelocity();
	sensor_msgs::JointState getWheel();
	//sensor_msgs::Imu getOdom();
	nav_msgs::Odometry getOdom();	
	

	void setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ);
	//void setOdom(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ);
	//void setWheel(const float whv, const float whef);
    
};

#endif
