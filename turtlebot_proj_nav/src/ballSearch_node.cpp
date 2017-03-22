#include "ros/ros.h"
#include "BallSearch.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "TurtleBotCamera.hpp"
#include "GraphicServer.hpp"

#define BALLE_ROUGE 0
#define BALLE_VERTE 1
#define BALLE_BLEUE 2

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Launching ballSearch_node ...\n");
    ros::init(argc, argv, "ballSearch_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1); // 0.3Hz 

    BallSearch ballSearch(n);
    TurtleBotCamera turtleBotCamera(n);
    GraphicServer graphicServer(n,"/nav/camera/color");
    GraphicServer graphicServerConvert(n,"/nav/traitement_image");
    
    int currentState = 0;
    
    sensor_msgs::Image image;
    unsigned char* raw;    
    unsigned char* rawFiltrageImage = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_MONO)];
    
    
    
    ROS_INFO("Initiating ...\n");
    ballSearch.attente(0,0.4); // Waiting 1 sec 0 nanosec
    
    Objet * obj;

   while (ros::ok()) 
   {
         ros::spinOnce();
             	    	    
	     raw = turtleBotCamera.getCameraRgbImageColorRaw();
	     rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, BALLE_ROUGE);
	     image = turtleBotCamera.convertRawToSensorMsgsImage(rawFiltrageImage, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_MONO);
	     

	     graphicServerConvert.sendImageDisplay(image);
	     graphicServer.sendImageDisplay(turtleBotCamera.getCameraRgbImageColor());
	     
	     
            
            switch (currentState)
            {
            case 0:
                ROS_INFO("Recherche de la balle...");
       	        obj = ballSearch.Recherche_balle(raw, CAMERA_WIDTH, CAMERA_HEIGHT, BALLE_ROUGE) ;
                if(obj == NULL) 
                {
                    ROS_INFO("Pas de balle trouvée...");
                }
                else 
                {
                  ROS_INFO("Distance estimée à la balle en m : %lf", (obj->Dist));
                  ROS_INFO("Angle estimé par rapport à la balle (degrés) : %lf ", obj->Theta);
                  ROS_INFO("Centre de la balle : (%d,%d) ", obj->Ucg, obj->Vcg);
                  currentState=1;
                }   
                break;
            case 1:
                ROS_INFO("Recherche de la balle...");
       	        obj = ballSearch.Recherche_balle(raw, CAMERA_WIDTH, CAMERA_HEIGHT, BALLE_ROUGE) ;
                if ((obj->Theta<-5) && !ballSearch.getCommandState()) 
                {
                    ballSearch.sendBallReference(0.2, -1.5, 0, -(obj->Theta)*PI/180);
                    ROS_INFO("On tourne de %lf degrés", (obj->Theta));
                }
              	else if ((obj->Theta>5) && !ballSearch.getCommandState()) 
              	{
              	    ballSearch.sendBallReference(0.2, 1.5, 0, (obj->Theta)*PI/180);
              	    ROS_INFO("On tourne de %lf degrés", (obj->Theta));
              	}
              	else if (!ballSearch.getCommandState())
              	    currentState=2;
              	else 
              	    currentState=1;
                break;
            case 2:
                ROS_INFO("Recherche de la balle...");
       	        obj = ballSearch.Recherche_balle(raw, CAMERA_WIDTH, CAMERA_HEIGHT, BALLE_ROUGE) ;
                if (!ballSearch.getCommandState()) 
                {
                    ROS_INFO("On avance de %lf m", (obj->Dist)-0.5);
	                ballSearch.sendBallReference(0.2, 1.5, (obj->Dist)-0.5, 0);
		            currentState=3;
                }
                break;    
            case 3:
                if (!ballSearch.getCommandState()) 
                {
                    currentState=4;
                }
                else ROS_INFO("En approche de la balle...");
                break;  
            case 4:
                ROS_INFO("Balle atteinte !");
                break;       
            default:
                currentState = 0;
                break;
		      }
         
         loop_rate.sleep();
    }
    
    return 0;
}
