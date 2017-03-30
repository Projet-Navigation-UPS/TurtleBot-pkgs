/*
  Odom.cpp
  Bruno Dato & Thibaut Aghnatios

  Header file
 
 */
#ifndef _ODOM_
#define _ODOM_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "kobuki_msgs/SensorState.h"


class Odom
{    
private:

	//Subscriber
	ros::Subscriber subscriberMobileBaseSensorsCore;

    //Message
	kobuki_msgs::SensorState mobileBaseSensorsCore;

public:

    Odom(ros::NodeHandle& node);
    ~Odom();

    //Get sensors data
    kobuki_msgs::SensorState getMobileBaseSensorsCore();

    //CB
    void callbackTicWheel(const kobuki_msgs::SensorState& msg);

    //Diplays 
    void displayMobileBaseSenorsCore();

	
};

#endif
