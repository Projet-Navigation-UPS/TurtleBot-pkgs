#include <stdlib.h>
#include <iostream>

#include "ros/ros.h"
#include <fstream>

#include <math.h>

//#include "serveurGraphique.h"
//#include "serveurGraphique.h"

#include "TurtleBot.hpp"
//#include "pekeeIIConnexion.h"

//#include "image.h"
//#include "champs_potentiels.h"

#define EXTERN

// #include "objet.h"
#include "traitement.hpp"
#include "analyse.hpp"

using namespace std;

//*********************************************************************************
// Fonction qui permet d'endormir le process pendant qq secondes
// Le temps que le robot commence à envoyer des données, comme par exemple des images
//*********************************************************************************
void attente(int nsec, int sec) {

  struct timespec delai_vie_appli;
  delai_vie_appli.tv_nsec = nsec;  
  delai_vie_appli.tv_sec = sec;
  nanosleep(&delai_vie_appli,NULL);
}


//******************************************************************************
// Recherche_fleche : Fonction qui permet de trouver la fleche verte sur fond rouge
//******************************************************************************

Objet * Recherche_balle(unsigned char* raw, int  width, int height, int couleur) { // couleur de la balle :
                                        // 0: rouge
                                        // 1: verte
                                        // 2 : bleue

 int h,w;
   Objet * listeObj; 
   Objet * obj = NULL;
  unsigned char* binRVB;
   int nb_rotates = 0;
  // unsigned char* raw;
   bool recherche_finie = false; // on recherche la balle autour du robot
   int current_rotate = 0; // rotation de la camera / axe robot 

/*   while ( ! recherche_finie ) {
	    rawFiltrageImage = filtrage_image(raw, CAMERA_WIDTH, CAMERA_HEIGHT, 1);
      // acquisition d'une image
	  //   attente(0,1);
	  
      unsigned char* raw = turtlebot.convertSensor_msgsImageToRaw(turtlebot.getCameraRgbImageRaw());
      int width = turtlebot.getCameraRgbImageRaw().width;
      int height = turtlebot.getCameraRgbImageRaw().height;

      // Si serveur graphique actif, envoi de l'image au serveur graphique
      if ( serveur_graphique && debug ) {
	//afficher_raw_image ((char*)raw, 1, width, height); //////////////////////////////////////////// 
      }  



      // Si serveur graphique actif, envoi de l'image filtree
      if ( serveur_graphique ) {
         // Allocation d'une image pour le bleu*/

  	

      binRVB = filtrage_image(raw,width,height,couleur); // Filtrage de l'image par des seuils rouges 

/*
         unsigned char* raw_1 = new unsigned char[width*height*3];
         for ( h = 0; h < (int)height; h++) {
            for ( w = 0; w < (int)width; w++) {
               raw_1[h*width*3 + w*3] = 0;
               raw_1[h*width*3 + w*3 + 1] = 0;
               raw_1[h*width*3 + w*3 + 2]= binRVB[h*width+w];
            }
         }
	return raw_1 ; 

       // afficher_raw_image ((char *)raw_1, 2, width, height);  */


         
      // Analyse de l'image  : decoupage en regions      
      int nbRegions = Etiqueter_Region(binRVB,width,height);
      printf("\nNombre de regions etiquetees : %d \n", nbRegions);
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
	


	// return num_obj  ;



      if ( num_obj == -1 ) {
         printf("\nPas de balle visible");
	  //exit(0);
         nb_rotates++;

         if ( nb_rotates <= 12 ) {
            if ( nb_rotates == 1 ) {
               printf(" => rotate à gauche\n"); // on regarde à gauche de la camera
	       //commandSensorCameraMove(-300,-200,0,1); //turtlebot.turnLeft()
	       current_rotate = 30;
	       attente(0,1);
            }
            else if ( nb_rotates == 2 ) {
               printf(" => rotate à droite\n"); // on regarde à droite de la camera
	       //commandSensorCameraMove(300,-200,0,1); //turtlebot.turnRight()
	       current_rotate = -30;
	       attente(0,2);
            }	
	    else if (nb_rotates <= 7) {
               printf(" => rotate à droite\n"); // on regarde à droite de la camera
	       //commandSensorCameraMove(300,0,0,0); //turtlebot.turnRight()
	       current_rotate = current_rotate - 30;
	       attente(0,1);
            }	
	    else if (nb_rotates == 8) {
               printf(" => rotate à gauche\n"); // on regarde à droite de la camera
	       // commandSensorCameraMove(-600,-200,0,1); //turtlebot.turnLeft()
	       current_rotate = 60;
	       attente(0,8);
            }
	    else if (nb_rotates > 8) {
               printf(" => rotate à gauche\n"); // on regarde à droite de la camera
	       //commandSensorCameraMove(-300,0,0,0); //turtlebot.turnLeft()
	       current_rotate = current_rotate + 30;
	       attente(0,1);
            }	
         }
         else {
            printf("\n");
            recherche_finie = true;
         } 
      }
      else {         
         printf("\nNuméro objet intéressant : %d \n", num_obj);
         recherche_finie = true;

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
std::cout << "focale:" << focale<<endl;
	 double diametre_balle = 10.5;
	 double k = 640.0/3.2; // 1/4" 3.2*2.4
         // valeur initiale de z à commenter une fois on calcule la vraie profondeur
	 double z = 80.0;
         // A calculer la profondeur z en tentant compte que d (figurant dans la formule de la profondeur) est le diamètre de la balle dans l'image


         double d = obj->Vcg - (double) (width)/2.0;
         double x = (d * diametre_balle) / (obj->Wmax - obj->Wmin);
         printf("d : %.2lf\n",d);
         printf("x : %.2lf\n",x);
         double theta = atan(x/z);
         printf("Barycentre : (%d, %d), Wmin, Wmax : (%d,%d), Hmin, Hmax : (%d,%d), Bounding box : (%d,%d), distance : %.2lf, Surface : %d \n", obj->Vcg, obj->Ucg,obj->Wmin,
obj->Wmax, obj->Hmin, obj->Hmax, obj->Hmax-obj->Hmin, obj->Wmax-obj->Wmin, z, obj->Surface); 
         obj->Dist = z;
         obj->Theta = theta*180.0/PI-current_rotate;
      }
   
   return obj;
}   


