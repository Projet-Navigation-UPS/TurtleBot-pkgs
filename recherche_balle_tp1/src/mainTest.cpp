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
    ros::Rate loop_rate(2); // 2Hz 


    TurtleBot turtleBot(node);
    GraphicServer graphicServer(node,"/image/display");
    GraphicServer graphicServerConvert(node,"/image/test");
    
    sensor_msgs::Image image;
    unsigned char* raw;    
    unsigned char* rawFiltrageImage = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_MONO)];
    
    while(ros::ok())
	{
	    //Displays
	    turtleBot.displaySensorMsgsImage("COLOR", turtleBot.getCameraRgbImageColor());
	    
	    
	    raw = turtleBot.getCameraRgbImageColorRaw();
	    rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 1);
                
	    image = turtleBot.convertRawToSensorMsgsImage(rawFiltrageImage, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_MONO);
                 
	    
        
        turtleBot.displaySensorMsgsImage("TEST", image);
	    graphicServerConvert.sendImageDisplay(image);
	    graphicServer.sendImageDisplay(turtleBot.getCameraRgbImageColor());
		
	    //Launching Callbacks and synchronizing with loop_rate
	    ros::spinOnce(); 
	    loop_rate.sleep();
	}
    return 0;
}
