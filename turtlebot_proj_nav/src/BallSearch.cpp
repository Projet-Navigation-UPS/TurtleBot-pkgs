/*
  BallSearch.cpp
  Bruno Dato, Marine Bouchet & Thibaut Aghnatios 

  Class which provide the search for a ball and communication with the command_node 
 
 */
#include "BallSearch.hpp"
#include <math.h>

// Constructor
BallSearch::BallSearch(ros::NodeHandle& node):
    //Subsribers
    subscriberCommandBusy(node.subscribe("/nav/command_busy", 1, &BallSearch::callbackCommandBusy,this)),
    //Publishers
    publisherBallReference(node.advertise<turtlebot_proj_nav::command>("/nav/open_loop_command", 1))
{
    command_busy.data = true;
    etat_recherche = 0;
}

// Destructor
BallSearch::~BallSearch(){}

// Callback that updates command busyness
void BallSearch::callbackCommandBusy(const std_msgs::Bool& msg)
{
    command_busy = msg;
}

// Function that sends angle and distance command to the command_node
void BallSearch::sendBallReference(const float linearVelocity, const float angularVelocity, const float distance, const float angle)
{
    if (!command_busy.data)
    {
        turtlebot_proj_nav::command msgBallReference;
        msgBallReference.linearVelocity = linearVelocity;
        msgBallReference.angularVelocity = angularVelocity;
        msgBallReference.distance = distance;
        msgBallReference.angle = angle;
        publisherBallReference.publish(msgBallReference);
        ROS_INFO("Command sent");
    }
    else ROS_INFO("Command busy");
}


// Fonction that asleeps the program 
void BallSearch::attente(int nsec, int sec) 
{
  struct timespec delai_vie_appli;
  delai_vie_appli.tv_nsec = nsec;  
  delai_vie_appli.tv_sec = sec;
  nanosleep(&delai_vie_appli,NULL);
}

// Function that detects a ball in a raw data image 
Objet * BallSearch::Recherche_balle(unsigned char* raw, int  width, int height, int couleur) 
{ // color of the ball :
                                        // 0: red
                                        // 1: gree
                                        // 2 : blue

	 int h,w;
	 Objet * listeObj; 
	 Objet * obj = NULL;
	 unsigned char* binRVB;


      binRVB = filtrage_image(raw,width,height,couleur); // Thresholding
         
      // Analysis of the image: division into regions      
      int nbRegions = Etiqueter_Region(binRVB,width,height);
      ROS_INFO("Number of segmented regions : %d ", nbRegions);
      listeObj = new Objet[nbRegions];

      // Extract attributes from each segmented region
      Extract_attributs(listeObj, binRVB, width, height, nbRegions, 0); 

      // Search for the largest area 
      int num_obj = -1;
      int surface_max = -1;
      
      // Look for a box of almost square footprint
      for ( int l = 0; l < nbRegions; l++ ) {
            //ROS_INFO("Object surface : %d",listeObj[l].Surface);
            //ROS_INFO("Ucg objet : %d",listeObj[l].Ucg);
            //ROS_INFO("Vcg objet : %d",listeObj[l].Vcg);

         if ( listeObj[l].Surface > SURF_MIN  ) {
            float h = listeObj[l].Hmax - listeObj[l].Hmin;
            float w = listeObj[l].Wmax - listeObj[l].Wmin;
            //ROS_INFO("h objet : %f",);
            //ROS_INFO("w objet : %f",);
            float ratio = h/w;
            //ROS_INFO("ratio objet : %f",rati);
            if ( ratio >= 0.8 && ratio <= 1.2 ) { // almost square
               if ( listeObj[l].Surface > surface_max  ) {
                  num_obj = l;
               }
            }
         }
      }
	
      if ( num_obj == -1 ) 
      {
         ROS_INFO("No visible ball");
         
         if (!command_busy.data)
         {
         
         switch (etat_recherche)
            {
                case 0:
                    ROS_INFO("Rotate left by Pi/3...");
                    this->sendBallReference(0, 1.5, 0, PI/3);
                    etat_recherche = 1;
                    break;
                case 1:
                    ROS_INFO("Rotate right by 2Pi/3...");
                    this->sendBallReference(0, -1.5, 0, 2*PI/3);
                    etat_recherche = 2;
                    break;
                case 2:
                    ROS_INFO("Rotate left by Pi...");
                    this->sendBallReference(0, 1.5, 0, PI);
                    etat_recherche = 3;
                    break;
                case 3:
                    ROS_INFO("Rotate right by 4Pi/3...");
                    this->sendBallReference(0, -1.5, 0, 4*PI/3);
                    etat_recherche = 4;
                    break;
                case 4:
                    ROS_INFO("Rotate left by 5Pi/3...");
                    this->sendBallReference(0, 1.5, 0, 5*PI/3);
                    etat_recherche = 5;
                    break;
                case 5:
                    ROS_INFO("Rotate right by 2Pi...");
                    this->sendBallReference(0, -1.5, 0, 2*PI);
                    etat_recherche = 6;
                    break;
                case 6:
                    ROS_INFO("Abort seeking");
                    break;
		        case 7:
                    ROS_INFO("Seeking over");
                    break;
                default:
                    ROS_INFO("Abort seeking");
                    break;

            }
         }
      }
      else {         
         ROS_INFO("Number of interecting objects : %d ", num_obj);

         obj = new Objet;
         obj->Ucg = listeObj[num_obj].Ucg;
         obj->Vcg = listeObj[num_obj].Vcg;
         obj->Perimetre = listeObj[num_obj].Perimetre;
         obj->Surface = listeObj[num_obj].Surface;
         obj->Color = listeObj[num_obj].Color;
         obj->Compactness = listeObj[num_obj].Compactness;
         obj->Mbr_Fill = listeObj[num_obj].Mbr_Fill;
         obj->Hmin = listeObj[num_obj].Hmin;
         obj->Hmax = listeObj[num_obj].Hmax;
         obj->Wmin = listeObj[num_obj].Wmin;
         obj->Wmax = listeObj[num_obj].Wmax;
	    double focale = 8.6; // Found empirically
	    //ROS_INFO("focale: %lf mm",focale);
	    double diametre_balle = 10.5;
	    double diametre_balle_image = obj->Wmax - obj->Wmin ;
	    double z = 55 / diametre_balle_image  ;
         double d = obj->Vcg - (double) (width)/2.0;
         double x = (d * diametre_balle) / diametre_balle_image;
         //ROS_INFO("d : %.2lf",d);
         //ROS_INFO("x : %.2lf\n",x);
         //ROS_INFO("z : %.2lf\n",z);
         double theta = asin(x/(z*100));
         //ROS_INFO("Barycentre : (%d, %d), Wmin, Wmax : (%d,%d), Hmin, Hmax : (%d,%d), Bounding box : (%d,%d), distance : %.2lf, Surface : %d", obj->Vcg, obj->Ucg,obj->Wmin,obj->Wmax, obj->Hmin, obj->Hmax, obj->Hmax-obj->Hmin, obj->Wmax-obj->Wmin, z, obj->Surface); 
         obj->Dist = z;
         obj->Theta = -theta*180.0/PI;
	    etat_recherche = 7;
         
      }
   
   return obj;
} 

bool BallSearch::getCommandState(){return command_busy.data;}
