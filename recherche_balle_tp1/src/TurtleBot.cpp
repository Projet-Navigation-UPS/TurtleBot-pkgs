#include "TurtleBot.hpp"
#include <vector>

TurtleBot::TurtleBot(ros::NodeHandle nod) :

	
	//Publishers
	publisherMobile_baseCommandsVelocity (nod.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)),

	//Subcribers
	subscriberCameraRgbImage_raw(nod.subscribe("/camera/rgb/image_raw", 1, &TurtleBot::callbackCameraRgbImage_raw,this)),
	subscriberCameraRgbImage_color(nod.subscribe("/camera/rgb/image_color", 1, &TurtleBot::callbackCameraRgbImage_color,this)),
	subscriberCameraRgbImage_rect_color(nod.subscribe("/camera/rgb/image_rect_color", 1, &TurtleBot::callbackCameraRgbImage_rect_color,this)),
{
	mobile_baseCommandsVelocity.
}

//Getters
sensor_msgs::Image TurtleBot::getCameraRgbImage_raw()
{
	return cameraRgbImage_raw;
}

sensor_msgs::Image TurtleBot::getCameraRgbImage_color()
{
	return cameraRgbImage_color;
}

sensor_msgs::Image TurtleBot::getCameraRgbImage_rect_color()
{
	return cameraRgbImage_rect_color;
}



void TurtleBot::callbackCameraRgbImage_raw(const sensor_msgs::Image& msg)
{
	cameraRgbImage_raw = msg;
}

void TurtleBot::callbackCameraRgbImage_color(const sensor_msgs::Image& msg)
{
	cameraRgbImage_color = msg;
}

void TurtleBot::callbackCameraRgbImage_rect_color(const sensor_msgs::Image& msg)
{
	cameraRgbImage_rect_color = msg;
}


unsigned char* TurtleBot::convertSensor_msgsImageToRaw(sensor_msgs::Image sensor_msgsImage)
{
	std::vector<unsigned char> vector_Raw(sensor_msgsImage.height*sensor_msgsImage.width);
	vector_Raw=sensor_msgsImage.data;
	unsigned char* raw = &vector_Raw[0];
	return raw;	
}

void TurtleBot::displaySensor_msgsImage(std::string type, sensor_msgs::Image sensor_msgsImage)
{
	std::cout<<"--------"<<type<<"---------"<<std::endl;
  	std::cout<<sensor_msgsImage.height<<std::endl;
	std::cout<<sensor_msgsImage.width<<std::endl;
	std::cout<<sensor_msgsImage.encoding<<std::endl;
	std::cout<<sensor_msgsImage.is_bigendian<<std::endl;
	std::cout<<sensor_msgsImage.step<<std::endl;
}

