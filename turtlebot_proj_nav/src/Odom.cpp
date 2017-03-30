/*
  Odom.cpp
  Bruno Dato & Thibaut Aghnatios

  Class that provides sensors data
 
 */
#include "Odom.hpp"

//Constructor
Odom::Odom(ros::NodeHandle& node):
	subscriberMobileBaseSensorsCore(node.subscribe("/mobile_base/sensors/core", 1, &Odom::callbackTicWheel,this))
{}

//Destructor
Odom::~Odom(){}

//Callback
// Updates wheels encoders
void Odom::callbackTicWheel(const kobuki_msgs::SensorState& msg)
{
    mobileBaseSensorsCore.left_encoder = msg.left_encoder;
	mobileBaseSensorsCore.right_encoder = msg.right_encoder;
    //std::cout<<"Wheels encoders"<<std::endl;
    //std::cout<<mobileBaseSensorsCore.left_encoder<<std::endl;
    //std::cout<<mobileBaseSensorsCore.right_encoder<<std::endl;

}

// Returns the current wheels encoder state
kobuki_msgs::SensorState Odom::getMobileBaseSensorsCore() 
{
    return mobileBaseSensorsCore;
}

// Displays the wheels encoder state
void Odom::displayMobileBaseSenorsCore()
{
    std::cout<<mobileBaseSensorsCore.left_encoder<<std::endl;
	std::cout<<mobileBaseSensorsCore.right_encoder<<std::endl;
}
