#include "ros/ros.h"
#include "TurtleBot.hpp"

#define SOUND_ON 0
#define SOUND_OFF 1
#define SOUND_RECHARGE 2
#define SOUND_BUTTON 3
#define SOUND_ERROR 4
#define SOUND_CLEANINGSTART 5
#define SOUND_CLEANINGEND 6

int main(int argc, char **argv)
{
  	std::cout<<"Launching recherche_balle_tp1 ..."<<std::endl;
  	ros::init(argc, argv, "vision");
  	ros::NodeHandle node;
	ros::Rate loop_rate(2); // 2Hz 

  	TurtleBot turtleBot(node);

	//turtleBot.moveForward();
	//turtleBot.moveBackward();
	//turtleBot.turnRight();
	//turtleBot.turnLeft();
	//turtleBot.moveForwardTurningRight();
	//turtleBot.moveForwardTurningLeft();
	//turtleBot.moveBackwardTurningRight();
	//turtleBot.moveBackwardTurningLeft();

	//turtleBot.setMobileBaseCommandsSound(SOUND_ON);

  	while(ros::ok())
	{
		//Displays
		turtleBot.displaySensorMsgsImage("RAW", turtleBot.getCameraRgbImageRaw());
		turtleBot.displaySensorMsgsImage("COLOR", turtleBot.getCameraRgbImageColor());
		turtleBot.displaySensorMsgsImage("COLOR_RECT", turtleBot.getCameraRgbImageRectColor());
		turtleBot.displayMobileBaseCommandsVelocity();
		turtleBot.displayJointStates();
		turtleBot.displayMobileBaseCommandsSound();

		//Publications
		turtleBot.sendMobileBaseCommandsVelocity();
		//turtleBot.sendMobileBaseCommandsSound();

		//Launching Callbacks and synchronizing with loop_rate
		ros::spinOnce(); 
		loop_rate.sleep();
	}
  	

  	return 0;
}
