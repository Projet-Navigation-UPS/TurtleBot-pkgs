#ifndef _TURTLEBOT_
#define _TURTLEBOT_

#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Image.h"

class Turtlebot 
{
	private:

	//Subscribers
	ros::Subscriber subscriberCameraRgbImage_raw ;
	ros::Subscriber subscriberCameraRgbImage_color ;
	ros::Subscriber subscriberCameraRgbImage_rect_color ;

	//Publishers
	//ros::Publisher ... ;

	//
	sensor_msgs::Image cameraRgbImage_raw;
	sensor_msgs::Image cameraRgbImage_color;
	sensor_msgs::Image cameraRgbImage_rect_color;

	public:

	Turtlebot(ros::NodeHandle nod);

	sensor_msgs::Image getCameraRgbImage_raw();
	sensor_msgs::Image getCameraRgbImage_color();
	sensor_msgs::Image getCameraRgbImage_rect_color();


	//Callbacks
	void callbackCameraRgbImage_raw(const sensor_msgs::Image& msg);
	void callbackCameraRgbImage_color(const sensor_msgs::Image& msg);
	void callbackCameraRgbImage_rect_color(const sensor_msgs::Image& msg);


	unsigned char* convertSensor_msgsImageToRaw(sensor_msgs::Image sensor_msgsImage);
	void displaySensor_msgsImage(std::string type, sensor_msgs::Image sensor_msgsImage);


};


#endif
