#include "Odom.hpp"




Odom::Odom(ros::NodeHandle& node):
	subscriberMobileBaseSensorsCore(node.subscribe("/mobile_base/sensors/core", 1, &Odom::callbackTicWheel,this))
{  
	 
}

Odom::~Odom(){}

//Callbacks
void Odom::callbackTicWheel(const kobuki_msgs::SensorState& msg)
{
	//std::cout<<"Tic R & L"<<std::endl;
	mobileBaseSensorsCore.left_encoder = msg.left_encoder;
	mobileBaseSensorsCore.right_encoder = msg.right_encoder;

    //std::cout<<"MESSAGE"<<std::endl;
    //std::cout<<mobileBaseSensorsCore.left_encoder<<std::endl;
    //std::cout<<mobileBaseSensorsCore.right_encoder<<std::endl;

}

kobuki_msgs::SensorState Odom::getMobileBaseSensorsCore() 
{
    return mobileBaseSensorsCore;
}

void Odom::displayMobileBaseSenorsCore()
{
    std::cout<<mobileBaseSensorsCore.left_encoder<<std::endl;
	std::cout<<mobileBaseSensorsCore.right_encoder<<std::endl;
}



