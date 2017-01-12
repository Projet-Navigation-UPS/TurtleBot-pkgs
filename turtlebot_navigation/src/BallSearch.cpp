#include "BallSearch.hpp"
#include "command.h"
#include <math.h>


BallSearch::BallSearch(ros::NodeHandle& node):
    //Subsribers
    subscriberCommandBusy(node.subscribe("/nav/command_busy", 1, &BallSearch::callbackCommandBusy,this)),
    //Publishers
    publisherBallReference(node.advertise<recherche_balle_tp1::command>("/nav/ball_reference", 1))
{
    command_busy.data = true;
    etat_recherche = 0;
}

BallSearch::~BallSearch(){}

void BallSearch::sendBallReference(const float linearVelocity, const float angularVelocity, const float distance, const float angle)
{
    if (!command_busy.data)
    {
        recherche_balle_tp1::command msgBallReference;
        msgBallReference.linearVelocity = linearVelocity;
        msgBallReference.angularVelocity = angularVelocity;
        msgBallReference.distance = distance;
        msgBallReference.angle = angle;
        publisherBallReference.publish(msgBallReference);
        ROS_INFO("COMMAND SENT !");
    }
    else ROS_INFO("COMMAND BUSY !");
}


void BallSearch::callbackCommandBusy(const std_msgs::Bool& msg)
{
    command_busy = msg;
}

//*********************************************************************************
// Fonction qui permet d'endormir le process pendant qq secondes
// Le temps que le robot commence à envoyer des données, comme par exemple des images
//*********************************************************************************
void BallSearch::attente(int nsec, int sec) 
{
  struct timespec delai_vie_appli;
  delai_vie_appli.tv_nsec = nsec;  
  delai_vie_appli.tv_sec = sec;
  nanosleep(&delai_vie_appli,NULL);
}

//******************************************************************************
// Recherche_fleche : Fonction qui permet de trouver la fleche verte sur fond rouge
//******************************************************************************
Objet * BallSearch::Recherche_balle(unsigned char* raw, int  width, int height, int couleur) 
{ // couleur de la balle :
                                        // 0: rouge
                                        // 1: verte
                                        // 2 : bleue

	 int h,w;
	 Objet * listeObj; 
	 Objet * obj = NULL;
	 unsigned char* binRVB;


      binRVB = filtrage_image(raw,width,height,couleur); // Filtrage de l'image par des seuils rouges 
         
      // Analyse de l'image  : decoupage en regions      
      int nbRegions = Etiqueter_Region(binRVB,width,height);
      ROS_INFO("Nombre de regions etiquetees : %d \n", nbRegions);
      listeObj = new Objet[nbRegions];

      // Extraction des attributs de chaque région segmentée
      Extract_attributs(listeObj, binRVB, width, height, nbRegions, 0); 

      // recherche de la plus grosse surface 
      int num_obj = -1;
      int surface_max = -1;
      
      // Il faut rechercher une boite d'encombrement presque carree
      for ( int l = 0; l < nbRegions; l++ ) {
        //  printf("\nSurface objet : %d",listeObj[l].Surface);
        //  printf("\nUcg objet : %d",listeObj[l].Ucg);
        //  printf("\nVcg objet : %d",listeObj[l].Vcg);
         if ( listeObj[l].Surface > SURF_MIN  ) {
            float h = listeObj[l].Hmax - listeObj[l].Hmin;
            float w = listeObj[l].Wmax - listeObj[l].Wmin;
	    // printf("\nh objet : %f",h);
	    // printf("\nw objet : %f",w);
            float ratio = h/w;
	    // printf("\nratio objet : %f",ratio);
            if ( ratio >= 0.8 && ratio <= 1.2 ) { // presque carrée
               if ( listeObj[l].Surface > surface_max  ) {
                  num_obj = l;
               }
            }
         }
      }
	
	
      if ( num_obj == -1 ) 
      {
         ROS_INFO("\nPas de balle visible");
         
         if (!command_busy.data)
         {
         
         switch (etat_recherche)
            {
                case 0:
                    ROS_INFO("On tourne à gauche de Pi/3 \n");
                    this->sendBallReference(0, 1.5, 0, PI/3);
                    etat_recherche = 1;
                    break;
                case 1:
                    ROS_INFO("On tourne à droite de 2Pi/3 \n");
                    this->sendBallReference(0, -1.5, 0, 2*PI/3);
                    etat_recherche = 2;
                    break;
                case 2:
                    ROS_INFO("On tourne à gauche de Pi \n");
                    this->sendBallReference(0, 1.5, 0, PI);
                    etat_recherche = 3;
                    break;
                case 3:
                    ROS_INFO("On tourne à droite de 4Pi/3 \n");
                    this->sendBallReference(0, -1.5, 0, 4*PI/3);
                    etat_recherche = 4;
                    break;
                case 4:
                    ROS_INFO("On tourne à droite de 5Pi/3 \n");
                    this->sendBallReference(0, 1.5, 0, 5*PI/3);
                    etat_recherche = 5;
                    break;
                case 5:
                    ROS_INFO("On tourne à droite de 2Pi \n");
                    this->sendBallReference(0, -1.5, 0, 2*PI);
                    etat_recherche = 6;
                    break;
                case 6:
                    ROS_INFO("Abandon recherche... \n");
                    break;
                default:
                    ROS_INFO("Abandon recherche... \n");
                    break;

            }
         }
      }
      else {         
         ROS_INFO("Numéro objet intéressant : %d \n", num_obj);

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
	 double focale = 8.6; // trouvée de facon empirique
         //std::cout << "focale:" << focale<<endl;
	 double diametre_balle = 10.5;
	 double diametre_balle_image = obj->Wmax - obj->Wmin ;
	 double z = 55 / diametre_balle_image  ;

         double d = obj->Vcg - (double) (width)/2.0;
         double x = (d * diametre_balle) / diametre_balle_image;//(obj->Wmax - obj->Wmin);
         //printf("d : %.2lf\n",d);
         //printf("x : %.2lf\n",x);
         //printf("z : %.2lf\n",z);
         double theta = asin(x/(z*100));
         //printf("Barycentre : (%d, %d), Wmin, Wmax : (%d,%d), Hmin, Hmax : (%d,%d), Bounding box : (%d,%d), distance : %.2lf, Surface : %d \n", obj->Vcg, obj->Ucg,obj->Wmin,obj->Wmax, obj->Hmin, obj->Hmax, obj->Hmax-obj->Hmin, obj->Wmax-obj->Wmin, z, obj->Surface); 
         obj->Dist = z;
         obj->Theta = -theta*180.0/PI;
         
      }
   
   return obj;
} 
