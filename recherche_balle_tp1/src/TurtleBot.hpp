#ifndef _TURTLEBOT_
#define _TURTLEBOT_

#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"

class TurtleBot 
{
	private:

	//Subscribers
	ros::Subscriber subscriberCameraRgbImage_raw ;
	ros::Subscriber subscriberCameraRgbImage_color ;
	ros::Subscriber subscriberCameraRgbImage_rect_color ;

	//Publishers
	ros::Publisher publisherMobile_baseCommandsVelocity ;


	//
	sensor_msgs::Image cameraRgbImage_raw;
	sensor_msgs::Image cameraRgbImage_color;
	sensor_msgs::Image cameraRgbImage_rect_color;

	geometry_msgs::Twist mobile_baseCommandsVelocity;

	public:

	TurtleBot(ros::NodeHandle nod);

	//Getters
	sensor_msgs::Image getCameraRgbImage_raw();
	sensor_msgs::Image getCameraRgbImage_color();
	sensor_msgs::Image getCameraRgbImage_rect_color();
	geometry_msgs::Twist getMobile_baseCommandsVelocity();

	//Setters
	void setMobile_baseCommandsVelocity(float linearX, float linearY, float linearZ, float angularX, float angularY, float angularZ);


	//Callbacks
	void callbackCameraRgbImage_raw(const sensor_msgs::Image& msg);
	void callbackCameraRgbImage_color(const sensor_msgs::Image& msg);
	void callbackCameraRgbImage_rect_color(const sensor_msgs::Image& msg);

	//Publications
	void sendMobile_baseCommandsVelocity();

	//Image convertion
	unsigned char* convertSensor_msgsImageToRaw(sensor_msgs::Image sensor_msgsImage);

	//Displays
	void displaySensor_msgsImage(std::string type, sensor_msgs::Image sensor_msgsImage);
	void displayMobile_baseCommandsVelocity();

	//Motions
	void stop();
	void moveForward();
	void moveBackward();
	void turnRight();
	void turnLeft();
	void moveForwardTurningRight();
	void moveForwardTurningLeft();
	void moveBackwardTurningRight();
	void moveBackwardTurningLeft();

};


#endif
