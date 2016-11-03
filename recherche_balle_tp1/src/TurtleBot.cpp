#include "TurtleBot.hpp"
#include <vector>

#define TRANLATION_MAX_VELOCITY 0.25
#define ANGULAR_MAX_VELOCITY 1


TurtleBot::TurtleBot(ros::NodeHandle nod) :

	
	//Publishers
	publisherMobile_baseCommandsVelocity (nod.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)),

	//Subcribers
	subscriberCameraRgbImage_raw(nod.subscribe("/camera/rgb/image_raw", 1, &TurtleBot::callbackCameraRgbImage_raw,this)),
	subscriberCameraRgbImage_color(nod.subscribe("/camera/rgb/image_color", 1, &TurtleBot::callbackCameraRgbImage_color,this)),
	subscriberCameraRgbImage_rect_color(nod.subscribe("/camera/rgb/image_rect_color", 1, &TurtleBot::callbackCameraRgbImage_rect_color,this))
{
	TurtleBot::stop();
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

geometry_msgs::Twist TurtleBot::getMobile_baseCommandsVelocity() 
{
	return mobile_baseCommandsVelocity;
}



//Setters
void TurtleBot::setMobile_baseCommandsVelocity(float linearX, float linearY, float linearZ, float angularX, float angularY, float angularZ)
{
	mobile_baseCommandsVelocity.linear.x=linearX;
	mobile_baseCommandsVelocity.linear.y=linearY;
	mobile_baseCommandsVelocity.linear.z=linearZ;
	mobile_baseCommandsVelocity.angular.x=angularX;
	mobile_baseCommandsVelocity.angular.y=angularY;
	mobile_baseCommandsVelocity.angular.z=angularZ;
}

//Callbacks
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


//Publications
void TurtleBot::sendMobile_baseCommandsVelocity()
{
	publisherMobile_baseCommandsVelocity.publish(mobile_baseCommandsVelocity);
}


//Image convertion
unsigned char* TurtleBot::convertSensor_msgsImageToRaw(sensor_msgs::Image sensor_msgsImage)
{
	std::vector<unsigned char> vector_Raw(sensor_msgsImage.height*sensor_msgsImage.width);
	vector_Raw=sensor_msgsImage.data;
	unsigned char* raw = &vector_Raw[0];
	return raw;	
}


//Displays
void TurtleBot::displaySensor_msgsImage(std::string type, sensor_msgs::Image sensor_msgsImage)
{
	std::cout<<"--------"<<type<<"---------"<<std::endl;
  	std::cout<<sensor_msgsImage.height<<std::endl;
	std::cout<<sensor_msgsImage.width<<std::endl;
	std::cout<<sensor_msgsImage.encoding<<std::endl;
	std::cout<<sensor_msgsImage.is_bigendian<<std::endl;
	std::cout<<sensor_msgsImage.step<<std::endl;
}

void TurtleBot::displayMobile_baseCommandsVelocity()
{
	std::cout<<TurtleBot::getMobile_baseCommandsVelocity()<<std::endl;
}


//Motions
void TurtleBot::stop()
{
	TurtleBot::setMobile_baseCommandsVelocity(0, 0, 0, 0, 0, 0);
}

void TurtleBot::moveForward()
{
	TurtleBot::setMobile_baseCommandsVelocity(TRANLATION_MAX_VELOCITY, 0, 0, 0, 0, 0);
}

void TurtleBot::moveBackward()
{
	TurtleBot::setMobile_baseCommandsVelocity(-TRANLATION_MAX_VELOCITY, 0, 0, 0, 0, 0);
}

void TurtleBot::turnRight()
{
	TurtleBot::setMobile_baseCommandsVelocity(0, 0, 0, 0, 0, -ANGULAR_MAX_VELOCITY);
}

void TurtleBot::turnLeft()
{
	TurtleBot::setMobile_baseCommandsVelocity(0, 0, 0, 0, 0, ANGULAR_MAX_VELOCITY);
}

void TurtleBot::moveForwardTurningRight()
{
	TurtleBot::setMobile_baseCommandsVelocity(TRANLATION_MAX_VELOCITY, 0, 0, 0, 0, -ANGULAR_MAX_VELOCITY);
}

void TurtleBot::moveForwardTurningLeft()
{
	TurtleBot::setMobile_baseCommandsVelocity(TRANLATION_MAX_VELOCITY, 0, 0, 0, 0, ANGULAR_MAX_VELOCITY);
}

void TurtleBot::moveBackwardTurningRight()
{
	TurtleBot::setMobile_baseCommandsVelocity(-TRANLATION_MAX_VELOCITY, 0, 0, 0, 0, -ANGULAR_MAX_VELOCITY);
}

void TurtleBot::moveBackwardTurningLeft()
{
	TurtleBot::setMobile_baseCommandsVelocity(-TRANLATION_MAX_VELOCITY, 0, 0, 0, 0, ANGULAR_MAX_VELOCITY);
}
