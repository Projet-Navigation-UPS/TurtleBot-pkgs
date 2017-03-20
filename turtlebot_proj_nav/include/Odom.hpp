#ifndef _ODOM_
#define _ODOM_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "kobuki_msgs/SensorState.h"


class Odom
{    
private:
        
    //Publishers

    
	//Subscribers
	ros::Subscriber subscriberMobileBaseSensorsCore;

    //Messages
	kobuki_msgs::SensorState mobileBaseSensorsCore;
	
        
    ros::NodeHandle& m_node;


public:

    Odom(ros::NodeHandle& node);
    ~Odom();
	kobuki_msgs::SensorState getMobileBaseSensorsCore();


    //Motion

    //Diplays
	void displayMobileBaseSenorsCore();

private:
	
};

#endif
