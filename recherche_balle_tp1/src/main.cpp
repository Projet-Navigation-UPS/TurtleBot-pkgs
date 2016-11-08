#include "ros/ros.h"
#include "TurtleBot.hpp"
#include "GraphicServer.hpp"

int main(int argc, char **argv)
{
    std::cout<<"Launching recherche_balle_tp1 ..."<<std::endl;
    ros::init(argc, argv, "recherche_balle_tp1");
    ros::NodeHandle node;
    ros::Rate loop_rate(2); // 2Hz 

    TurtleBot turtleBot(node);
    GraphicServer graphicServer("/image/display");
    while(ros::ok())
	{
	    //Displays
	    turtleBot.displaySensorMsgsImage("RAW", turtleBot.getCameraRgbImageRaw());
	    turtleBot.displaySensorMsgsImage("COLOR", turtleBot.getCameraRgbImageColor());
	    turtleBot.displaySensorMsgsImage("COLOR_RECT", turtleBot.getCameraRgbImageRectColor());
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
