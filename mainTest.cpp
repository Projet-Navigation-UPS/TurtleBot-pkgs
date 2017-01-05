#include "ros/ros.h"
#include "TurtleBot.hpp"
#include "GraphicServer.hpp"
#include "BallSearch.cpp"
#include "ros/console.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

  
int main(int argc, char **argv)
{
    std::cout<<"Launching recherche_balle_tp1 ..."<<std::endl;
    ros::init(argc, argv, "recherche_balle_tp1");
    ros::NodeHandle node;
    ros::Rate loop_rate(2); // 2Hz 
    ros::init(argc, argv, "talker");

  ros::NodeHandle n2;

 
  ros::Publisher chatter_pub = n2.advertise<std_msgs::String>("chatter", 1000);




    TurtleBot turtleBot(node);
    GraphicServer graphicServer(node,"/image/display");
    GraphicServer graphicServerConvert(node,"/image/test");
    
    sensor_msgs::Image image;
    unsigned char* raw;    
    unsigned char* rawFiltrageImage = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_MONO)];
    
   while(ros::ok()) {
	   

	    //Displays
	    turtleBot.displaySensorMsgsImage("COLOR", turtleBot.getCameraRgbImageColor());
	    
	    
	    raw = turtleBot.getCameraRgbImageColorRaw();
          Objet * f = Recherche_balle(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0) ;

	   rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0);
		image = turtleBot.convertRawToSensorMsgsImage(rawFiltrageImage, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_MONO);

	/* if (&f!=(-1)) {
		rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0);
		image = turtleBot.convertRawToSensorMsgsImage(rawFiltrageImage, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_MONO);
        	turtleBot.displaySensorMsgsImage("TEST", image);
	}*/


	ROS_DEBUG("Hello %s", "World");
	ROS_DEBUG_STREAM("Hello " << "World");

	    graphicServerConvert.sendImageDisplay(image);
	    graphicServer.sendImageDisplay(turtleBot.getCameraRgbImageColor());
		
	    //Launching Callbacks and synchronizing with loop_rate
	    ros::spinOnce(); 
	    loop_rate.sleep();
	
		
	 }
    return 0;
}
