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
    std::cout<<"Launching ballSearch_node ..."<<std::endl;
    ros::init(argc, argv, "ballSearch");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5); // 0.5Hz 

    BallSearch ballSearch(n);
    TurtleBotCamera turtleBotCamera(n);
    GraphicServer graphicServer(n,"/image/display");
    GraphicServer graphicServerConvert(n,"/image/test");
    
    sensor_msgs::Image image;
    unsigned char* raw;    
    unsigned char* rawFiltrageImage = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*(CAMERA_STEP_MONO)];
    
    
    std::cout<<"Initiating ..."<<std::endl;
    ballSearch.attente(0,1); // O nano seconde, 5 secondes

   while ( ros::ok() ) {
         printf("\nRecherche de la balle...");
         
         ros::spinOnce();
	    	    
	     raw = turtleBotCamera.getCameraRgbImageColorRaw();
	     rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0);
	     image = turtleBotCamera.convertRawToSensorMsgsImage(rawFiltrageImage, CAMERA_HEIGHT, CAMERA_WIDTH, "mono8", ' ', CAMERA_STEP_MONO);
	     
	     //Displays
	     /*turtleBotCamera.displaySensorMsgsImage("COLOR", turtleBotCamera.getCameraRgbImageColor());
         turtleBotCamera.displaySensorMsgsImage("TEST", image);*/

	     graphicServerConvert.sendImageDisplay(image);
	     graphicServer.sendImageDisplay(turtleBotCamera.getCameraRgbImageColor());
	     
         Objet * obj = ballSearch.Recherche_balle(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 0) ;
         if ( obj == NULL ) { // balle non detectee
            printf("\nPas de balle trouvée. \n");
            ballSearch.sendBallReference(0, 1.5, 0, PI/4);
         }
         else {
            printf("distance estimée à la balle en m : %lf \n",  (obj->Dist));
            printf("angle estimé par rapport à la balle (C) : %lf \n", obj->Theta);
            printf("centre de la balle : (%d,%d) \n", obj->Ucg, obj->Vcg);
            printf("=> on tourne de %lf degrés\n", (obj->Theta));
            ballSearch.sendBallReference(0.2, 1, (obj->Dist)-0.5,(obj->Theta)*PI/180);
	        //ballSearch.attente(0,10);
            /*if ( obj->Dist > 90.0 ) { // 90 cm 
               float force_repulsive;
               float angle_force_repulsive;
               float distance_influence = 50.0; // en cm
               //int exist = existence_obstacle(pekee, &force_repulsive,
               //                               &angle_force_repulsive, distance_influence);
               //if ( exist == 1 ) 
               { 
                  //printf("\nattention obstacle\n");
                  //printf(" force_repulsive : %f\n",force_repulsive);
                  //printf(" angle_force_repulsive : %f\n",angle_force_repulsive); 
                  //float t;
		          //commandMotorSetSpeedAndSteering(100, angle_force_repulsive , 2000); // 100 mm /s pendant 2s
		          ballSearch.attente(0,1); // pour que le robot ait le temps de tourner
                }
                else {
		   printf("=> on avance de 20 cm en tournant de %d°\n",(int)obj->Theta);
                   //commandMotorSetSpeedAndSteering(100, ((int)obj->Theta)/2, 2000); // 100 mm /s pendant 2s
		   //commandSensorCameraMove(0,-200,0,1);// reinit de la camera 
		   ballSearch.attente(0,2);
               }
            }
            else {
		  // printf("=> on avance de 10 cm en tournant de %d°\n",(int)obj->Theta);
                  // commandMotorSetSpeedAndSteering(50, ((int)obj->Theta)/2, 2000); // 100 mm /s pendant 2s
		  // commandSensorCameraMove(0,-200,0,1); // on baisse la camera 
		   ballSearch.attente(0,2);
            }*/
	    
	    /*if ( arret ) {
	      std::cout << "Entrez un car pour continuer le processus " << std::endl;
	      getline(cin,blabla);
	    }*/
         }
         
         std::cout<<"Ending ..."<<std::endl;
         
         loop_rate.sleep();
    }
    
    //ros::spin();
    
    /*while(ros::ok())
	{
	    ballSearch.sendBallReference(linearVelocity, angularVelocity, distance, angle);      
	    //Launching Callbacks and synchronizing with loop_rate
	    ros::spinOnce(); 
	    loop_rate.sleep();
	}*/
    return 0;
}
