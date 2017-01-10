#ifndef _TURTLEBOTCOMMAND_
#define _TURTLEBOTCOMMAND_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "geometry_msgs/Twist.h"

const float ROBOT_LINEAR_MAX_VELOCITY = 0.25;
const float ROBOT_ANGULAR_MAX_VELOCITY = 2;

const float ROBOT_MAX_LINEAR_VELOCITY = 0.5f;
const float ROBOT_MAX_ANGULAR_VELOCITY = 3.14f;

class TurtleBotCommand 
{    
private:
        
    //Publishers
    ros::Publisher publisherMobileBaseCommandsVelocity;
    
    //Messages
    geometry_msgs::Twist mobileBaseCommandsVelocity;
        
    ros::NodeHandle& m_node;
public:

    TurtleBotCommand(ros::NodeHandle& node);
    ~TurtleBotCommand();
    
    //Motion
    void stop();
    void move(const float linearVelocity);
    void turn(const float angularVelocity);

    void displayMobileBaseCommandsVelocity();
    void moveAndTurn(const float linearVelocity, const float angularVelocity);

private:
    geometry_msgs::Twist getMobileBaseCommandsVelocity();
    void setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ);
    
};

#endif
