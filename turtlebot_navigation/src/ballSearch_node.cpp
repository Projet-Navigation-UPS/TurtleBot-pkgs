#include "ros/ros.h"
#include "BallSearch.hpp"
#include "command.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "TurtleBotCamera.hpp"
#include "GraphicServer.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Launching ballSearch_node ...\n");
    ros::init(argc, argv, "ballSearch_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.3); // 0.3Hz 

    BallSearch ballSearch(n);
    TurtleBotCamera turtleBotCamera(n);
    GraphicServer graphicServer(n,"/nav/camera/color");
    GraphicServer graphicServerConvert(n,"/nav/traitement_image");
    
    
    sensor_msgs::Image image;
    unsigned char* raw;    
    unsigned char* rawFiltrageImage = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_MONO)];
    
    
    
    ROS_INFO("Initiating ...\n");
    ballSearch.attente(0,1); // Waiting 1 sec 0 nanosec

   while (ros::ok()) 
   {
         ros::spinOnce();
         
         
              	    	    
	     raw = turtleBotCamera.getCameraRgbImageColorRaw();
	     rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0);
	     image = turtleBotCamera.convertRawToSensorMsgsImage(rawFiltrageImage, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_MONO);
	     

	     graphicServerConvert.sendImageDisplay(image);
	     graphicServer.sendImageDisplay(turtleBotCamera.getCameraRgbImageColor());
	     
	    ROS_INFO("Recherche de la balle...\n");
            Objet * obj = ballSearch.Recherche_balle(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0) ;
         
            if ( obj == NULL ) 
            {
               ROS_INFO("Pas de balle trouvée. \n");
            }
            else 
            {
               ROS_INFO("distance estimée à la balle en m : %lf \n",  (obj->Dist));
               ROS_INFO("angle estimé par rapport à la balle (degrés) : %lf \n", obj->Theta);
               ROS_INFO("centre de la balle : (%d,%d) \n", obj->Ucg, obj->Vcg);
               ROS_INFO("=> on tourne de %lf degrés\n", (obj->Theta));
            
	           if (obj->Theta<0) ballSearch.sendBallReference(0.2, -1.5, (obj->Dist)-0.5,-(obj->Theta)*PI/180);
	           else ballSearch.sendBallReference(0.2, 1.5, (obj->Dist)-0.5,(obj->Theta)*PI/180);
             
               
            }

         
         loop_rate.sleep();
    }
    
    return 0;
}
