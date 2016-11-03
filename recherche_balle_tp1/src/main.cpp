#include "ros/ros.h"
#include "Turtlebot.hpp"




int main(int argc, char **argv)
{
  	std::cout<<"Launching vision_node ..."<<std::endl;
  	ros::init(argc, argv, "vision");
  	ros::NodeHandle n;
	ros::Rate loop_rate(0.5); // 0.5Hz

  	Turtlebot turtlebot(n);

  	while(ros::ok())
	{
		turtlebot.displaySensor_msgsImage("RAW", turtlebot.getCameraRgbImage_raw());

		turtlebot.displaySensor_msgsImage("COLOR", turtlebot.getCameraRgbImage_color());

		turtlebot.displaySensor_msgsImage("COLOR_RECT", turtlebot.getCameraRgbImage_raw());


		ros::spinOnce(); 
		loop_rate.sleep();
	}
  	

  	return 0;
}
