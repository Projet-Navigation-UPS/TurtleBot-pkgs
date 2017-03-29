#ifndef _TURTLEBOTCOMMAND_
#define _TURTLEBOTCOMMAND_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "turtlebot_proj_nav/command.h"
#include "std_msgs/Bool.h"

const float ROBOT_LINEAR_MAX_VELOCITY = 0.25;
const float ROBOT_ANGULAR_MAX_VELOCITY = 2;

const float ROBOT_MAX_LINEAR_VELOCITY = 0.5f;
const float ROBOT_MAX_ANGULAR_VELOCITY = 3.14f;

class TurtleBotCommand 
{    
private:

    bool turning;
    bool moving;
    bool stopMouvement;
    bool startMouvement;
    
    
    //Publishers
    ros::Publisher publisherMobileBaseCommandsVelocity, pubCommandState;
    
    //Subscibers
    ros::Subscriber subCommandReceived;
    
    //Messages
    geometry_msgs::Twist mobileBaseCommandsVelocity;
    std_msgs::Bool busy;
    turtlebot_proj_nav::command commandAsked;
    
    //CallBacks
    void callBackCommandReceived(const turtlebot_proj_nav::command& msg);
        
public:

    TurtleBotCommand(ros::NodeHandle& node);
    ~TurtleBotCommand();
    
    //Motion
    void stop();
    void move(const float linearVelocity);
    void move();
    void turn(const float angularVelocity);
    void turn();

    //Diplays
    void displayMobileBaseCommandsVelocity();
    void moveAndTurn(const float linearVelocity, const float angularVelocity);
    
    //States
    bool start();
    bool stop2();
    bool turtleBotMoving();
    bool turtleBotTurning();
    bool commandBusy();
    bool commandEnabled();
    
    //Durations
    ros::WallDuration turningDuration();
    ros::WallDuration movingDuration();
    
    void turningOver();
    void movingOver();
    
    void publishCommandState();

private:
    geometry_msgs::Twist getMobileBaseCommandsVelocity();
    void setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ);
    
};

#endif
