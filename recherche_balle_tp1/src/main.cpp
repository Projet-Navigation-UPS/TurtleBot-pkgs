#include "ros/ros.h"
#include "TurtleBot.hpp"




int main(int argc, char **argv)
{
  	std::cout<<"Launching recherche_balle_tp1 ..."<<std::endl;
  	ros::init(argc, argv, "vision");
  	ros::NodeHandle n;
	ros::Rate loop_rate(0.5); // 0.5Hz

  	TurtleBot turtleBot(n);

  	while(ros::ok())
	{
		turtleBot.displaySensor_msgsImage("RAW", turtleBot.getCameraRgbImage_raw());

		turtleBot.displaySensor_msgsImage("COLOR", turtleBot.getCameraRgbImage_color());

		turtleBot.displaySensor_msgsImage("COLOR_RECT", turtleBot.getCameraRgbImage_raw());


		ros::spinOnce(); 
		loop_rate.sleep();
	}
  	

  	return 0;
}
