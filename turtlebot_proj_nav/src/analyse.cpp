#include <stdlib.h>
#include <iostream>
#include <math.h>

#define EXTERN extern
#include "objet.h"

/* Fonction récursive AVANCER_ETIQUETAGE  */
/* image : image entrée                   */
/* h, w : indice du pixel                 */
/* etiquette : étiquette du pixel         */
int Avancer_Etiquetage(unsigned char* image, int width, int height, int h, int w, int etiquette) {

   int dx, dy; /* indices de balayage des 8 voisins */
   
 //  if (etiquette > 255 ) etiquette = 255;
  //          printf("ds Avancer_Etiquetage : %d\n",etiquette);
  //          printf("ds Avancer_Etiquetage h : %d\n",h);
  //          printf("ds Avancer_Etiquetage w : %d\n",w);
    image[h*width+w] = etiquette;

   // on boucle sur les 8 voisins
	for (dx=-1; dx <= 1; dx++) {
      for (dy = -1; dy <= 1; dy++) {
         // on teste si non hors image
         if ((h+dx >= 0) && ((h+dx) < (int) height) && ((w+dy)>=0) && ((w+dy)< (int) width)) {
            // on teste si le voisin est 255
            if (image[(h+dx)*width+(w+dy)]==255) {
               Avancer_Etiquetage(image, width, height, h+dx, w+dy, etiquette);
            }
         }
      }
   }
   return 0;
}

/* Fonction ETIQUETTER_REGION           */
/* image : image entrée et mise  à jour */
int Etiqueter_Region(unsigned char* image, int width, int height) {

   bool etiquettage_fini = false; 

   int etiquette = 1; /* indice de label pour les régions */
   int h, w; /* indices de lignes et colonnes image */

   h = 0;
   while ( h < (int) height && ! etiquettage_fini ) {
      w = 0;
      while ( w < (int) width && ! etiquettage_fini ) {
         if (image[h*width+w] == 255) {
            Avancer_Etiquetage(image, width, height, h, w, etiquette);
            etiquette++;    
            if (etiquette > 254 ) etiquettage_fini = true;
           // printf("Etiquette : %d\n",etiquette);
         }
         w++;
      }
      h++;
   } 
  // printf("Etiquette en sortie de Etiqueter_region: %d\n",etiquette-1);
   return etiquette-1; 
}

/* Fonction EXTRACT_ATTRIBUTS     */
/* listeObjet : liste des régions */
/* image : image entrée           */
/* nbObj : nbre objets total      */
/* c : indice du canal couleur    */
int Extract_attributs(Objet * listeObjet, unsigned char * image, int width, int height, int nbObj, int c) {

   int i; /* indice de région */
   int h, w; /* indices de ligne et colonnes image */
   int dx, dy; /* indices pour balayer les 8 voisins */

   // Initialisation de la structure
   for (i = 0; i < nbObj; i++) {
      listeObjet[i].Ucg = 0.0;
      listeObjet[i].Vcg = 0.0;
      listeObjet[i].Perimetre = 0;
      listeObjet[i].Surface = 0;
      listeObjet[i].Color = c;
      listeObjet[i].Compactness = 0.0;
      listeObjet[i].Mbr_Fill = 0.0;
      listeObjet[i].Hmin = (int) height;
      listeObjet[i].Hmax = 0;
      listeObjet[i].Wmin = (int) width;
      listeObjet[i].Wmax = 0;
      listeObjet[i].Bord = 0;
      listeObjet[i].Dist = 0;
      listeObjet[i].Theta = 0;
      listeObjet[i].Sens = 0;
      listeObjet[i].Angle_de_vue = 0;
      listeObjet[i].Reussite = 0;
   }
   for (h = 0; h < (int) height; h++) {
      for (w = 0; w < (int) width; w++) {
         if (image[h*width+w] != 0 && image[h*width+w] != 255) { // car les labels vont de 1 à 254
            listeObjet[image[h*width+w]-1].Ucg += (double) h;
            listeObjet[image[h*width+w]-1].Vcg += (double) w;
            listeObjet[image[h*width+w]-1].Surface ++;
            if (h < listeObjet[image[h*width+w]-1].Hmin) listeObjet[image[h*width+w]-1].Hmin = h;
            if (h > listeObjet[image[h*width+w]-1].Hmax) listeObjet[image[h*width+w]-1].Hmax = h;
            if (w < listeObjet[image[h*width+w]-1].Wmin) listeObjet[image[h*width+w]-1].Wmin = w;                       
            if (w > listeObjet[image[h*width+w]-1].Wmax) listeObjet[image[h*width+w]-1].Wmax = w;
            if (h == 0 || h == (int) height-1 || w == 0 || w == (int) width-1) {
               listeObjet[image[h*width+w]-1].Bord = 1;
            }
            int flag = 0;
            for (dx = -1; dx <= 1 && !flag; dx++) {
               for (dy=-1; dy<= 1 && !flag; dy++) {
                  if ((h+dx >= 0) && ((h+dx) < (int) height) && ((w+dy) >= 0) && ((w+dy) < (int) width)) {
                     if (image[(h+dx)*width+(w+dy)] != image[h*width+w]) {
                        listeObjet[image[h*width+w]-1].Perimetre ++;
                        flag = 1;
                     }
                  }
               }
            }
         }
      }
   }
   // Calcul du barycentre et facteur de compacité
   for (int i = 0;i < nbObj;i++) {
      listeObjet[i].Ucg = listeObjet[i].Ucg / listeObjet[i].Surface;
      listeObjet[i].Vcg = listeObjet[i].Vcg / listeObjet[i].Surface;
      // A calculer le facteur de compacité de l'objet


   //   printf("Objet[%d]: Position : (%lf, %lf), Perimetre: %d, Surface : %d, Compacité: %lf\n", i, listeObjet[i].Ucg, listeObjet[i].Vcg, listeObjet[i].Perimetre, listeObjet[i].Surface, listeObjet[i].Compactness);
   }
   
   return 0;
}

bool est_carre(Objet region) {

  bool result = false;
  float largeur = region.Wmax - region.Wmin; 
  float hauteur = region.Hmax - region.Hmin;
  if ( hauteur == 0 ) {
    result = false;
  }
  else {
   // printf("largeur : %d\n",largeur);
   // printf("hauteur : %d\n",hauteur);
   // printf("largeur/hauteur : %f\n",largeur / hauteur);

    if ( ((largeur / hauteur) >= 0.3) && ((largeur / hauteur) <= 1.7 ) ) result = true;
  }
  return result;
}

bool est_inclus(Objet region_verte, Objet region_rouge) {

  bool result = false;
  if ( region_verte.Ucg > region_rouge.Hmin && region_verte.Ucg < region_rouge.Hmax &&
       region_verte.Vcg > region_rouge.Wmin && region_verte.Vcg < region_rouge.Wmax ) {
      result = true;
  }
  return result;
}
