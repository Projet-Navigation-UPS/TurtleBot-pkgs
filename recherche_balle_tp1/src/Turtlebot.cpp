#include "Turtlebot.hpp"
#include <vector>

Turtlebot::Turtlebot(ros::NodeHandle nod) :

	
	//Publishers
	//pub_enable(noeud.advertise<msg>("topic", 1)),

	//Subcribers
	subscriberCameraRgbImage_raw(nod.subscribe("/camera/rgb/image_raw", 1, &Turtlebot::callbackCameraRgbImage_raw,this)),
	subscriberCameraRgbImage_color(nod.subscribe("/camera/rgb/image_color", 1, &Turtlebot::callbackCameraRgbImage_color,this)),
	subscriberCameraRgbImage_rect_color(nod.subscribe("/camera/rgb/image_rect_color", 1, &Turtlebot::callbackCameraRgbImage_rect_color,this))
{
}

//Getters
sensor_msgs::Image Turtlebot::getCameraRgbImage_raw()
{
	return cameraRgbImage_raw;
}

sensor_msgs::Image Turtlebot::getCameraRgbImage_color()
{
	return cameraRgbImage_color;
}

sensor_msgs::Image Turtlebot::getCameraRgbImage_rect_color()
{
	return cameraRgbImage_rect_color;
}


void Turtlebot::callbackCameraRgbImage_raw(const sensor_msgs::Image& msg)
{
	cameraRgbImage_raw = msg;
}

void Turtlebot::callbackCameraRgbImage_color(const sensor_msgs::Image& msg)
{
	cameraRgbImage_color = msg;
}

void Turtlebot::callbackCameraRgbImage_rect_color(const sensor_msgs::Image& msg)
{
	cameraRgbImage_rect_color = msg;
}


unsigned char* Turtlebot::convertSensor_msgsImageToRaw(sensor_msgs::Image sensor_msgsImage)
{
	std::vector<unsigned char> vector_Raw(sensor_msgsImage.height*sensor_msgsImage.width);
	vector_Raw=sensor_msgsImage.data;
	unsigned char* raw = &vector_Raw[0];
	return raw;	
}

void Turtlebot::displaySensor_msgsImage(std::string type, sensor_msgs::Image sensor_msgsImage)
{
	std::cout<<"--------"<<type<<"---------"<<std::endl;
  	std::cout<<sensor_msgsImage.height<<std::endl;
	std::cout<<sensor_msgsImage.width<<std::endl;
	std::cout<<sensor_msgsImage.encoding<<std::endl;
	std::cout<<sensor_msgsImage.is_bigendian<<std::endl;
	std::cout<<sensor_msgsImage.step<<std::endl;
}

