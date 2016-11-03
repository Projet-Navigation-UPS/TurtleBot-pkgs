#include "ros/ros.h"
#include "TurtleBot.hpp"

int main(int argc, char **argv)
{
  	std::cout<<"Launching recherche_balle_tp1 ..."<<std::endl;
  	ros::init(argc, argv, "vision");
  	ros::NodeHandle n;
	ros::Rate loop_rate(2); // 2Hz 

  	TurtleBot turtleBot(n);

	//turtleBot.moveForward();
	//turtleBot.moveBackward();
	//turtleBot.turnRight();
	//turtleBot.turnLeft();
	//turtleBot.moveForwardTurningRight();
	//turtleBot.moveForwardTurningLeft();
	//turtleBot.moveBackwardTurningRight();
	//turtleBot.moveBackwardTurningLeft();

  	while(ros::ok())
	{
		//Displays
		turtleBot.displaySensor_msgsImage("RAW", turtleBot.getCameraRgbImage_raw());
		turtleBot.displaySensor_msgsImage("COLOR", turtleBot.getCameraRgbImage_color());
		turtleBot.displaySensor_msgsImage("COLOR_RECT", turtleBot.getCameraRgbImage_raw());
		turtleBot.displayMobile_baseCommandsVelocity();

		//Publications
		turtleBot.sendMobile_baseCommandsVelocity();

		//Launching Callbacks and synchronizing with loop_rate
		ros::spinOnce(); 
		loop_rate.sleep();
	}
  	

  	return 0;
}
