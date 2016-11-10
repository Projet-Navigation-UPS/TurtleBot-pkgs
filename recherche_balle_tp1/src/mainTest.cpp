#include "ros/ros.h"
#include "TurtleBot.hpp"
#include "GraphicServer.hpp"
#include "traitement.hpp"
#include "analyse.hpp"

int main(int argc, char **argv)
{
    std::cout<<"Launching recherche_balle_tp1 ..."<<std::endl;
    ros::init(argc, argv, "recherche_balle_tp1");
    ros::NodeHandle node;
    ros::Rate loop_rate(1); // 2Hz 


    TurtleBot turtleBot(node);
    GraphicServer graphicServer(node,"/image/display");
    GraphicServer graphicServerConvert(node,"/image/test");
    
    sensor_msgs::Image image;
    unsigned char* raw;

    unsigned char* rawGet_red_channel = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_RGB/3)];
    unsigned char* rawGet_green_channel = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_RGB/3)];
    unsigned char* rawGet_blue_channel = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_RGB/3)];
    
    while(ros::ok())
	{
	    //Displays
	    turtleBot.displaySensorMsgsImage("COLOR", turtleBot.getCameraRgbImageColor());
	    
	    raw = turtleBot.getCameraRgbImageColorRaw();
	    rawGet_red_channel = get_red_channel(raw, CAMERA_WIDTH, CAMERA_HEIGHT);
	    rawGet_green_channel = get_green_channel(raw, CAMERA_WIDTH, CAMERA_HEIGHT);
	    rawGet_blue_channel = get_blue_channel(raw, CAMERA_WIDTH, CAMERA_HEIGHT);
                
	    image = turtleBot.convertRawToSensorMsgsImage(rawGet_blue_channel, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_RGB/3);
                 
	    turtleBot.displaySensorMsgsImage("TEST", image);
        
        
	    graphicServerConvert.sendImageDisplay(image);
	    graphicServer.sendImageDisplay(turtleBot.getCameraRgbImageColor());
		
	    //Launching Callbacks and synchronizing with loop_rate
	    ros::spinOnce(); 
	    loop_rate.sleep();
	}
    return 0;
}
