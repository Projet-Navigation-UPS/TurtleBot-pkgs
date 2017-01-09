#include "TurtleBotCommand.hpp"

TurtleBotCommand::TurtleBotCommand(ros::NodeHandle& node):
    m_node(node),
    publisherMobileBaseCommandsVelocity(node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1))
{
    TurtleBotCommand::stop();
}

TurtleBotCommand::~TurtleBotCommand()
{}

geometry_msgs::Twist TurtleBotCommand::getMobileBaseCommandsVelocity() 
{
    return mobileBaseCommandsVelocity;
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
}

void TurtleBotCommand::move(const float linearVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, 0);
}

void TurtleBotCommand::turn(const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, angularVelocity);
}

void TurtleBotCommand::moveAndTurn(const float linearVelocity, const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, angularVelocity);
}
