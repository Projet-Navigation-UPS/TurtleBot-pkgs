/*
  TurtleBotCommand.hpp
  Bruno Dato & Tristant Klempka

  Header file 
 
 */
#ifndef _TURTLEBOTCOMMAND_
#define _TURTLEBOTCOMMAND_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "turtlebot_proj_nav/command.h"
#include "std_msgs/Bool.h"


const float ROBOT_MAX_LINEAR_VELOCITY = 0.5f;
const float ROBOT_MAX_ANGULAR_VELOCITY = 3.1416f;

class TurtleBotCommand 
{    
private:

    // Boolean states
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
    
    
    // Speed command
    void setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ);
        
public:

    TurtleBotCommand(ros::NodeHandle& node);
    ~TurtleBotCommand();
    
    //Motions
    void stop();
    void move(const float linearVelocity);
    void move();
    void turn(const float angularVelocity);
    void turn();
    void moveAndTurn(const float linearVelocity, const float angularVelocity);

    //Diplays
    void displayMobileBaseCommandsVelocity();
    
    //States
    bool start();
    bool movingPhase();
    bool turningPhase();
    bool commandBusy();
    
    //Durations
    ros::WallDuration turningDuration();
    ros::WallDuration movingDuration();
    
    // End of motions
    void turningOver();
    void movingOver();
    
    // Publishing
    void publishCommandState();
    
    // Debug function
    geometry_msgs::Twist getMobileBaseCommandsVelocity();
    
};

#endif
