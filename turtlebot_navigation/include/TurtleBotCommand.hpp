#ifndef _TURTLEBOTCOMMAND_
#define _TURTLEBOTCOMMAND_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

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
    void move(const float linearVelocity);
    void turn(const float angularVelocity);
	void TurtleBotCommand::folcom(const float tabp[][], const float linearVelocity, const float angularVelocity);

    //Diplays
    void displayOdom();
    void moveAndTurn(const float linearVelocity, const float angularVelocity);

private:
    geometry_msgs::Twist getMobileBaseCommandsVelocity();
	sensor_msgs::JointState TurtleCommand::Wheel();
	sensor_msgs::Imu TurtleCommand::getOdom();
	
	void TurtleBotCommand::setOdom(const float posecov, const float twist);
	void TurtleBotCommabd::setWheel(const float whp, const float whv, const float whef);
	void TurtleBotCommand::setOdom(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ, const float posX, const float posY, const float posZ, const float angleX, const float angleY, const float angleZ, const float anglew)


    
};

#endif
