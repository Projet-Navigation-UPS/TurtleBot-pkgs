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
    
    unsigned char* rawSeuillage;
    unsigned char* rawOuverture;
    unsigned char* rawSeuillage_rapide;
    unsigned char* rawFermeture;
    unsigned char* rawErosion;
    unsigned char* rawDilatation;
    unsigned char** rawGet_chromatic_channels;
    unsigned char* rawGet_red_channel;
    unsigned char* rawGet_green_channel;
    unsigned char* rawGet_blue_channel;
    unsigned char* rawFiltrage_image;
    unsigned char* rawFiltrage_image_rapide;

    int canal = 0;
    int width = turtleBot.getCameraRgbImageColor().width; 
    int height = turtleBot.getCameraRgbImageColor().height;        
               
    
    
	while(ros::ok())
	{
		//Displays
		turtleBot.displaySensorMsgsImage("RAW", turtleBot.getCameraRgbImageRaw());
		turtleBot.displaySensorMsgsImage("COLOR", turtleBot.getCameraRgbImageColor());
		turtleBot.displaySensorMsgsImage("COLOR_RECT", turtleBot.getCameraRgbImageRectColor());
		turtleBot.displayMobileBaseCommandsVelocity();
		turtleBot.displayJointStates();
		turtleBot.displayMobileBaseCommandsSound();
        
        
        raw = turtleBot.convertSensor_msgsImageToRaw(turtleBot.getCameraRgbImageColor());
        
        rawOuverture = ouverture(raw, width, height);
        rawFermeture = fermeture(raw, width, height);
        rawErosion = erosion(raw, width, height);
        rawDilatation = dilatation(raw, width, height);
        rawGet_chromatic_channels = get_chromatic_channels(raw, width, height);
        rawGet_red_channel = get_red_channel(raw, width, height);
        rawGet_green_channel = get_green_channel(raw, width, height);
        rawGet_blue_channel = get_blue_channel(raw, width, height);
        rawSeuillage = seuillage(&raw, width, height, canal);
        rawSeuillage_rapide = seuillage_rapide(raw, width, height, canal);
        rawFiltrage_image = filtrage_image(raw, width, height, canal);
        rawFiltrage_image_rapide = filtrage_image_rapide(raw, width, height, canal);
        
                
        image = turtleBot.convertRawToSensorMsgsImage(rawSeuillage, turtleBot.getCameraRgbImageColor().height,turtleBot.getCameraRgbImageColor().width, turtleBot.getCameraRgbImageColor().encoding, turtleBot.getCameraRgbImageColor().is_bigendian, turtleBot.getCameraRgbImageColor().step);
         
        
        turtleBot.displaySensorMsgsImage("TEST", image);
        
        
        graphicServerConvert.sendImageDisplay(image);
	    graphicServer.sendImageDisplay(turtleBot.getCameraRgbImageColor());
		
		//Launching Callbacks and synchronizing with loop_rate
		ros::spinOnce(); 
		loop_rate.sleep();
	}
    return 0;
}
