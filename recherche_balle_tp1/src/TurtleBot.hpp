#ifndef _TURTLEBOT_
#define _TURTLEBOT_

#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

class TurtleBot 
{
	private:

	//Subscribers
	ros::Subscriber subscriberCameraRgbImageRaw;
	ros::Subscriber subscriberCameraRgbImageColor;
	ros::Subscriber subscriberCameraRgbImageRectColor;
	ros::Subscriber subscriberJointStates ;

	//Publishers
	ros::Publisher publisherMobileBaseCommandsVelocity;


	//Messages
	sensor_msgs::Image cameraRgbImageRaw;
	sensor_msgs::Image cameraRgbImageColor;
	sensor_msgs::Image cameraRgbImageRectColor;
	sensor_msgs::JointState JointStates;

	geometry_msgs::Twist mobileBaseCommandsVelocity;

	public:

	TurtleBot(ros::NodeHandle nod);

	//Getters
	sensor_msgs::Image getCameraRgbImageRaw();
	sensor_msgs::Image getCameraRgbImageColor();
	sensor_msgs::Image getCameraRgbImageRectColor();
	geometry_msgs::Twist getMobileBaseCommandsVelocity();

	//Setters
	void setMobileBaseCommandsVelocity(float linearX, float linearY, float linearZ, float angularX, float angularY, float angularZ);


	//Callbacks
	void callbackCameraRgbImageRaw(const sensor_msgs::Image& msg);
	void callbackCameraRgbImageColor(const sensor_msgs::Image& msg);
	void callbackCameraRgbImageRectColor(const sensor_msgs::Image& msg);

	//Publications
	void sendMobileBaseCommandsVelocity();

	//Image convertion
	unsigned char* convertSensor_msgsImageToRaw(sensor_msgs::Image sensorMsgsImage);

	//Displays
	void displaySensorMsgsImage(std::string type, sensor_msgs::Image sensorMsgsImage);
	void displayMobileBaseCommandsVelocity();

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
