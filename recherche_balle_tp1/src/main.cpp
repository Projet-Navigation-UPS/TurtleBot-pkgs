#include "ros/ros.h"
#include "TurtleBot.hpp"
#include "GraphicServer.hpp"

int main(int argc, char **argv)
{
	std::cout<<"Launching recherche_balle_tp1 ..."<<std::endl;
	ros::init(argc, argv, "recherche_balle_tp1");
	ros::NodeHandle node;
	ros::Rate loop_rate(0.5); // 0.5Hz 

	TurtleBot turtleBot(node);
	GraphicServer graphicServer(node,"/image/display");
	while(ros::ok())
	{
		//Displays
		turtleBot.displayMobileBaseCommandsVelocity();
		turtleBot.displayJointStates();
		turtleBot.displayMobileBaseCommandsSound();


		graphicServer.sendImageDisplay(turtleBot.getCameraRgbImageColor());
		
		//Launching Callbacks and synchronizing with loop_rate
		ros::spinOnce(); 
		loop_rate.sleep();
	}
    return 0;
}
