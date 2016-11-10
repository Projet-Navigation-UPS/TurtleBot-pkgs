#include "ros/ros.h"
#include "TurtleBot.hpp"
#include "GraphicServer.hpp"
#include "traitement.hpp"
#include "analyse.hpp"

int main(int argc, char **argv)
{
    std::cout<<"Launching Demo ..."<<std::endl;
    ros::init(argc, argv, "recherche_balle_tp1_demo");
    ros::NodeHandle node;
    ros::Rate loop_rate(2); // 2Hz 


    TurtleBot turtleBot(node);
    GraphicServer graphicServer(node,"/image/display");
    
    float time = 0.0f;
    
    while(ros::ok())
	{
        if(time>4) time = 0.0f;
	    
        //Motion example
	   
	    turtleBot.moveForwardTurningRight();
        /*if (time<3) turtleBot.turnLeft();
        else turtleBot.moveForward();  */            
        
        //Displays
        turtleBot.displaySensorMsgsImage("COLOR", turtleBot.getCameraRgbImageColor());
	    
	    
        graphicServer.sendImageDisplay(turtleBot.getCameraRgbImageColor());
		
        //Launching Callbacks and synchronizing with loop_rate
        ros::spinOnce(); 
        loop_rate.sleep();
	    
	    time += 0.5f;
	}
    return 0;
}
