/*
  ballSearch_node.cpp
  Bruno Dato

  ROS Node which supervises the research of a ball.
 
 */
#include "ros/ros.h"
#include "BallSearch.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "TurtleBotCamera.hpp"
#include "GraphicServer.hpp"

using namespace std;

int main(int argc, char **argv)
{
    // ROS node init
    ROS_INFO("Launching ballSearch_node ...\n");
    ros::init(argc, argv, "ballSearch_node");
    ros::NodeHandle n;
    // Rate of the node
    ros::Rate loop_rate(0.3); // 0.3Hz 

    // Object which provides image processing
    BallSearch ballSearch(n);
    // Object which provides image stream of the robot
    TurtleBotCamera turtleBotCamera(n);
    
    // Graphic servers which publish the RGB image and the threshold image
    GraphicServer graphicServer(n,"/nav/camera/color");
    GraphicServer graphicServerConvert(n,"/nav/traitement_image");
    
    // Image message for ROS
    sensor_msgs::Image image;
 
    // Raw data for images
    unsigned char* raw;    
    unsigned char* rawFiltrageImage = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_MONO)];
    
    ROS_INFO("Initiating ...\n");
    ballSearch.attente(0,1); // Waiting 1 sec 0 nanosec
    bool command_sent = false;

   while (ros::ok()) 
   {
         // Launches callbacks which received messages
         ros::spinOnce();
         
         // Get robot RGB image raw     	    	    
	     raw = turtleBotCamera.getCameraRgbImageColorRaw();
	     // Get threshold image raw for a red ball
	     rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0);
	     // Convert raw into an image message for ROS
	     image = turtleBotCamera.convertRawToSensorMsgsImage(rawFiltrageImage, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_MONO);
	     
         // Publish video streams
	     graphicServerConvert.sendImageDisplay(image);
	     graphicServer.sendImageDisplay(turtleBotCamera.getCameraRgbImageColor());
	     
	        ROS_INFO("Seek ball...\n");
	        // Seeking the red ball in the RGB image
            Objet * obj = ballSearch.Recherche_balle(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0) ;
         
            if ( obj == NULL ) 
            {
               ROS_INFO("No ball found");
            }
            else if(!command_sent)
            {
               ROS_INFO("Distance estimated to the ball : %lf m",  (obj->Dist));
               ROS_INFO("Angle estimated to the ball: %lf degrees", obj->Theta);
               ROS_INFO("Center of the ball : (%d,%d) ", obj->Ucg, obj->Vcg);
            
               // Send distance and angle command to reach the ball
	           if (obj->Theta<0) 
	           {
	                ballSearch.sendBallReference(0.2, -1.5, (obj->Dist)-0.7,-(obj->Theta)*PI/180);
	                command_sent = true;
	           }
	           else 
	           {
	                ballSearch.sendBallReference(0.2, 1.5, (obj->Dist)-0.7,(obj->Theta)*PI/180);
	                command_sent = true;
               }
               
            }

         // Synchronize with the rate of the node
         loop_rate.sleep();
    }
    
    return 0;
}
