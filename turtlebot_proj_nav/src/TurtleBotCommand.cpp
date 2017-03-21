#include "TurtleBotCommand.hpp"

TurtleBotCommand::TurtleBotCommand(ros::NodeHandle& node):
    //subscribers
    subCommandReceived(node.subscribe("/nav/open_loop_command", 1, &TurtleBotCommand::callBackCommandReceived,this)),
    subCommandState(node.subscribe("/nav/command/state", 1, &TurtleBotCommand::callBackEnableCommand,this)),

    //publishers
    pubCommandState(node.advertise<std_msgs::Bool>("/nav/command_busy", 1)),
    publisherMobileBaseCommandsVelocity(node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1))
{
    commandAsked.linearVelocity = 0;
    commandAsked.angularVelocity = 0;
    commandAsked.distance = 0;
    commandAsked.angle = 0;
    busy.data = false;
    enabled.data = true;
    turning = false;
    moving = false;
    stopMouvement = true;
    startMouvement = false;
    stop();
    pubCommandState.publish(busy);
}

TurtleBotCommand::~TurtleBotCommand()
{}

geometry_msgs::Twist TurtleBotCommand::getMobileBaseCommandsVelocity() 
{
    return mobileBaseCommandsVelocity;
}

//Callbacks
void TurtleBotCommand::callBackCommandReceived(const turtlebot_proj_nav::command& msg)
{
        ROS_INFO("Command received...");
        //std::cout<<*msg<<std::endl;
	    commandAsked = msg;
	    turning = true;
	    moving = false;
	    stopMouvement = false;
	    startMouvement = true;
	    busy.data = true;
	    pubCommandState.publish(busy);
}

void TurtleBotCommand::callBackEnableCommand(const std_msgs::Bool& msg)
{
	    enabled.data = !(msg.data); 
}

void TurtleBotCommand::setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ)
{
    mobileBaseCommandsVelocity.linear.x=linearX;
    mobileBaseCommandsVelocity.linear.y=linearY;
    mobileBaseCommandsVelocity.linear.z=linearZ;
    mobileBaseCommandsVelocity.angular.x=angularX;
    mobileBaseCommandsVelocity.angular.y=angularY;
    mobileBaseCommandsVelocity.angular.z=angularZ;
    publisherMobileBaseCommandsVelocity.publish(mobileBaseCommandsVelocity);
}


void TurtleBotCommand::displayMobileBaseCommandsVelocity()
{
    std::cout<<mobileBaseCommandsVelocity<<std::endl;
}

//Motions
void TurtleBotCommand::stop()
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, 0);
    //busy.data = false;
    //pubCommandState.publish(busy);
}

void TurtleBotCommand::move(const float linearVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, 0);
}

void TurtleBotCommand::move()
{
    TurtleBotCommand::move(commandAsked.linearVelocity);
}

void TurtleBotCommand::turn(const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, angularVelocity);
}

void TurtleBotCommand::turn()
{
    TurtleBotCommand::turn(commandAsked.angularVelocity*1.42);
}

void TurtleBotCommand::moveAndTurn(const float linearVelocity, const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, angularVelocity);
}


//States
bool TurtleBotCommand::start(){return startMouvement;}

bool TurtleBotCommand::stop2(){return stopMouvement;}

bool TurtleBotCommand::turtleBotMoving(){return moving;}

bool TurtleBotCommand::turtleBotTurning(){return turning;}

bool TurtleBotCommand::commandBusy(){return busy.data;}

bool TurtleBotCommand::commandEnabled(){return enabled.data;}

//Durations
ros::WallDuration TurtleBotCommand::turningDuration()
{
    ros::WallDuration duration;
    if(commandAsked.angularVelocity ==0 || commandAsked.angle == 0) 
    {
        duration = ros::WallDuration(0);
    }
    else 
    {
        if (commandAsked.angularVelocity<0) duration = ros::WallDuration(-commandAsked.angle/commandAsked.angularVelocity);
        else duration = ros::WallDuration(commandAsked.angle/commandAsked.angularVelocity);
    }
    startMouvement = false;
    return duration;
}


ros::WallDuration TurtleBotCommand::movingDuration()
{
    ros::WallDuration duration;
    if(commandAsked.linearVelocity == 0 || commandAsked.distance == 0) 
    {
        duration = ros::WallDuration(0);
    }
    else 
    {
        if (commandAsked.linearVelocity<0) duration = ros::WallDuration(-commandAsked.distance/commandAsked.linearVelocity);
        else duration = ros::WallDuration(commandAsked.distance/commandAsked.linearVelocity);
    }
    startMouvement = false;
    return duration;
}

//

void TurtleBotCommand::turningOver()
{
    turning = false;
    moving = true;
    startMouvement = true;
    stop();
}

void TurtleBotCommand::movingOver()
{
    moving = false;
    stopMouvement = true;
    stop();
    busy.data = false;
    pubCommandState.publish(busy);
}

void TurtleBotCommand::publishCommandState()
{
    pubCommandState.publish(busy);
}





