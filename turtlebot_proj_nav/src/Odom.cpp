#include "Odom.hpp"




Odom::Odom(ros::NodeHandle& node):
    m_node(node),
    subscriberMobileBaseSensorsCore(node.advertise<kobuki_msgs::SensorState>("/mobile_base/sensors/core", 1)),
{
    
}


Odom::~Odom
{}


kobuki_msgs::SensorState TurtleBotCommand::getMobileBaseSensorsCore() 
{
    return mobileBaseSensorsCore;
}


void odom::displayMobileBaseSenorsCore()
{
    std::cout<<mobileBaseSensorsCore<<std::endl;
}



