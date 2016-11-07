#include <stdlib.h>
#include <iostream>
#include <fstream>

#define EXTERN extern
#include "objet.h"

using namespace std;

//*********************************************************************************
// Fonction qui permet de passer d'un espace colorimétrique Rouge,Vert,Bleu à
// Hue, Saturation, Value. 
// Hue == teinte, comprise entre 0 et 360°
// Si 0 < teinte < 60 ou 300 < teinte < 360 => rouge
// Si 60 < teinte < 180  => vert
// Si 180 < teinte < 300  => bleu
// Saturation == intensité de la couleur, comprise entre 0 et 1. Proche de 0 => grise
// Value == brillance de la couleur, comprise entre 0 et 1.
// Les valeurs en entrée doivent être comprises entre 0 et 1. 
//*********************************************************************************
void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v ) {

   float min, max, delta;

   min = r;
   if ( g < min ) min = g;
   if ( b < min ) min = b;

   max = r;
   if ( g > max ) max = g;
   if ( b > max ) max = b;

   *v = max;				

   delta = max - min;

   if ( max != 0 ) {
      *s = delta / max;		
   }
   else {
      // r = g = b = 0		// s = 0, v is undefined
      *s = 0;
      *h = -1;
      return;
   }

   if ( r == max )
      *h = ( g - b ) / delta;		// between yellow & magenta
   else if ( g == max )
      *h = 2 + ( b - r ) / delta;	// between cyan & yellow
   else
      *h = 4 + ( r - g ) / delta;	// between magenta & cyan

   *h *= 60;				// degrees
   if ( *h < 0 ) *h += 360;

}


//*********************************************************************************
// Fonction qui renvoie "vrai" quand les paramètres "RGB" caractèrisent une
// couleur de teinte rouge, faux sinon.
// Les valeurs en entrée doivent être comprises entre 0 et 255. 
//*********************************************************************************
bool estRouge(int rouge, int vert, int bleu) {
   
   float r = rouge / 255.0;
   float v = vert / 255.0;
   float b = bleu / 255.0;

   float hue, saturation, value;
   bool result = false;

   RGBtoHSV( r, v, b, &hue, &saturation, &value );
   if ( saturation > 0.38 ) { // intensité suffisante pour détermininer la couleur
      if ( (hue >= 0.0 && hue <= 30.0) || (hue >= 330.0 && hue <= 360.0) ) {
        result = true;
      }
   }
   return result;
}

//*********************************************************************************
// Fonction qui renvoie "vrai" quand les paramètres "RGB" caractèrisent une
// couleur de teinte verte, faux sinon.
// Les valeurs en entrée doivent être comprises entre 0 et 255. 
//*********************************************************************************
bool estVert(int rouge, int vert, int bleu) {
   
   float r = rouge / 255.0;
   float v = vert / 255.0;
   float b = bleu / 255.0;

   float hue, saturation, value;
   bool result = false;

   RGBtoHSV( r, v, b, &hue, &saturation, &value );
   if ( saturation > 0.2 ) { // intensité suffisante pour détermininer la couleur
      if ( hue >= 80.0 && hue <= 160.0 ) {
        result = true;
      }
   }
   return result;
}

//*********************************************************************************
// Fonction qui renvoie "vrai" quand les paramètres "RGB" caractèrisent une
// couleur de teinte bleue, faux sinon.
// Les valeurs en entrée doivent être comprises entre 0 et 255. 
//*********************************************************************************
bool estBleu(int rouge, int vert, int bleu) {
   
   float r = rouge / 255.0;
   float v = vert / 255.0;
   float b = bleu / 255.0;

   float hue, saturation, value;
   bool result = false;

   RGBtoHSV( r, v, b, &hue, &saturation, &value );
   if ( value > 0.3 && saturation > 0.2) { // intensité suffisante pour déterminer la couleur
      if ( hue > 170.0 && hue < 230.0 ) {
        result = true;
      }
   }
   return result;
}

//*********************************************************************************
// Fonction qui permet de charger dans "hist" l'histogramme decrit dans un fichier
// L'histogramme a 32 niveaux de rouge, 32 niveaux de vert, et 32 niveaux de bleu
//*********************************************************************************

void chargement_histogramme( const char* fic, float hist[32][32][32]) {   

   ifstream ficin;

   // Open the file where the reference histogram is stored in
   ficin.open( fic,ios::in );

   // Chargement de l'histogramme
   float valbin;

   for ( int i = 0 ; i < 32 ; i ++ ) {
       for ( int j = 0 ; j < 32 ; j ++ ) {
          for ( int k = 0 ; k < 32 ; k ++ ) {
             ficin >> valbin;
             hist[i][j][k] = valbin;
          }
       }
   }
   ficin.close();
}

//******************************************************************************
// seuillage_par_histogramme : Fonction qui permet de seuiller une image
// en utilisant un histogramme de probabilité
// L'histogramme a 32 niveaux de rouge, 32 niveaux de vert, et 32 niveaux de bleu
// image : image couleur en entree 
// en sortie : nouvelle image resultat de la binarisation
//******************************************************************************
unsigned char* seuillage_par_histogramme (unsigned char** image, int width, int height, float hist[32][32][32]) {

   int h, w;
   unsigned char* image2 = new unsigned char[width*height]; 
   
   for (h = 0; h < (int) height; h++) {
      for (w = 0; w < (int) width; w++) {
         int ind_rouge = (int)(image[0][h*width + w]/8);
         int ind_vert = (int)(image[1][h*width + w]/8);
         int ind_bleu = (int)(image[2][h*width + w]/8);
         if ( hist[ind_rouge][ind_vert][ind_bleu] > 0 ) {
            image2[h*width + w] = 255;
         }
         else {
            image2[h*width + w] = 0;
         }
      }
   }
   return image2;
}

//******************************************************************************
// seuillage : Fonction qui permet de seuiller une image
// image : image couleur en entree 
// en sortie : nouvelle image resultat de la binarisation
//******************************************************************************

unsigned char* seuillage (unsigned char** image, int width, int height, int canal) {

   int h, w;
   unsigned char* image2 = new unsigned char[width*height]; 
   
   if ( canal == 0 ) { // filtrage à prédominante rouge 
      for (h = 0; h < (int) height; h++) {
         for (w = 0; w < (int) width; w++) {
            int r = (int)(image[0][h*width + w]);
            int v = (int)(image[1][h*width + w]);
            int b = (int)(image[2][h*width + w]);
            if ( estRouge(r, v, b) ) {
                image2[h*width + w] = 255;
            }
            else {
               image2[h*width + w] = 0;
            }
         }
      }
   }
   else if ( canal == 1 ) { // filtrage à prédominante verte 
       for (h = 0; h < (int) height; h++) {
         for (w = 0; w < (int) width; w++) {
            int r = (int)(image[0][h*width + w]);
            int v = (int)(image[1][h*width + w]);
            int b = (int)(image[2][h*width + w]);
            if ( estVert(r,v,b) ) {
                image2[h*width + w] = 255;
            }
            else {
               image2[h*width + w] = 0;
            }
         }
      }
   }
   else if ( canal == 2 ) { // filtrage à prédominante bleue 
      for (h = 0; h < (int) height; h++) {
         for (w = 0; w < (int) width; w++) {
            int r = (int)(image[0][h*width + w]);
            int v = (int)(image[1][h*width + w]);
            int b = (int)(image[2][h*width + w]);
            if ( estBleu(r,v,b) ) {
                image2[h*width + w] = 255;
            }
            else {
               image2[h*width + w] = 0;
            }
         }
      }
   }
   return image2;
}

//******************************************************************************
// seuillage : Fonction qui permet de seuiller une image
// image : image couleur en entree 
// en sortie : nouvelle image resultat de la binarisation
//******************************************************************************

unsigned char* seuillage_rapide (unsigned char* image, int width, int height, int canal) {

   int h, w;
   unsigned char* image2 = new unsigned char[width*height]; 
   
   if ( canal == 0 ) { // filtrage à prédominante rouge 
      for (h = 0; h < (int) height; h++) {
	 int offset = h*width*3;
	 int offset2 = h*width;

         for (w = 0; w < (int) width; w++) {
	    int wf3 = w*3;
	    int decal = offset + wf3;
            int r = (int)(image[decal]);
            int v = (int)(image[decal + 1]);
            int b = (int)(image[decal + 2]);
            if ( estRouge(r, v, b) ) {
                image2[offset2 + w] = 255;
            }
            else {
               image2[offset2 + w] = 0;
            }
         }
      }
   }
   else if ( canal == 1 ) { // filtrage à prédominante verte 
       for (h = 0; h < (int) height; h++) {
         for (w = 0; w < (int) width; w++) {
            int r = (int)(image[h*width*3 + w*3]);
            int v = (int)(image[h*width*3 + w*3 + 1]);
            int b = (int)(image[h*width*3 + w*3 + 2]);
            if ( estVert(r,v,b) ) {
                image2[h*width + w] = 255;
            }
            else {
               image2[h*width + w] = 0;
            }
         }
      }
   }
   else if ( canal == 2 ) { // filtrage à prédominante bleue 
      for (h = 0; h < (int) height; h++) {
         for (w = 0; w < (int) width; w++) {
            int r = (int)(image[h*width*3 + w*3]);
            int v = (int)(image[h*width*3 + w*3 + 1]);
            int b = (int)(image[h*width*3 + w*3 + 2]);
            if ( estBleu(r,v,b) ) {
                image2[h*width + w] = 255;
            }
            else {
               image2[h*width + w] = 0;
            }
         }
      }
   }
   return image2;
}

//******************************************************************************
// erosion : Fonction qui permet d'éroder une image
// en entree : image à 3 canaux à éroder
// en sortie : nouvelle image resultat de l'érosion 
//******************************************************************************

unsigned char* erosion (unsigned char* image, int width, int height) {

   int size = 1; /* demi taille du noyau */
   int h, w; /* indices pour balayer lignes et colonnes image */
   int dx, dy; /* indices pour balayer le noyau */
   unsigned char* temp = new unsigned char[width*height]; /* image temporaire */
   bool erodee;
   
   for (h = 0; h < (int)height; h++) {
      for (w = 0; w < (int)width; w++) {
         erodee = false;
         if ( h < size || h > ((int)height - size - 1) || w < size || w > ((int)width - size - 1) ) {
           temp[h*width+w] = 0;
         } 
         else {
           temp[(h * width) + w]=255;
           for (dx =-size; dx <= size; dx++) {
              for (dy =-size; dy <= size; dy++) {
                 if (image[(h+dx)*width + (w+dy)]== 0) {
                    temp[(h*width) + w] = 0;
                    erodee = true;
                 }
                 if (erodee ) break;
              }
              if (erodee ) break;
           }
         }
      }
   }
   return temp;
}


//******************************************************************************
// dilation : Fonction qui permet de dilater une image
// en entree : image à 3 canaux à dilater
// en sortie : nouvelle image resultat de la dilatation 
//******************************************************************************

unsigned char* dilatation (unsigned char* image, int width, int height) {
// A compléter la fonction dilatation en se basant sur la fonction erosion

}

//******************************************************************************
// ouverture 
// en entree : image à 3 canaux sur laquelle on effectue une ouverture
// en sortie : nouvelle image resultat de l'ouverture
//******************************************************************************

unsigned char* ouverture (unsigned char* image, int width, int height) {
// A completer la fonction ouverture en utilisant les fonctions erosion et dilatation

 //  return image_dilatee;
}

//******************************************************************************
// fermeture 
// en entree : image à 3 canaux sur laquelle on effectue une fermeture
// en sortie : nouvelle image resultat de la fermeture
//******************************************************************************

unsigned char* fermeture (unsigned char* image, int width, int height) {
// A completer la fonction fermeture en utilisant les fonctions erosion et dilatation

 //  return image_erodee;
}


//******************************************************************************
// Fonction get_chromatic_channels
// en entree : image a 3 canaux (rouge, vert, bleu)
// en sortie : nouvelle image a 3 canaux separes
//******************************************************************************
unsigned char** get_chromatic_channels(unsigned char* raw, int width, int height) {

   int h,w;
   unsigned char* rawRVB[3]; 

   for (int i = 0 ;i <= 2; i++) {
      rawRVB[i] = new unsigned char[width*height];
   }

   // on recupere les plans de l'image d'origine
   for ( h = 0; h < (int)height; h++) {
     for ( w = 0; w < (int)width; w++) {
        rawRVB[0][h*width+w] = raw[h*width*3 + w*3];
        rawRVB[1][h*width+w] = raw[h*width*3 + w*3 + 1];	
        rawRVB[2][h*width+w] = raw[h*width*3 + w*3 + 2];
     }
   }
   return rawRVB;
}

//******************************************************************************
// Fonction get_red_channel 
// en entree : image a 3 canaux (rouge, vert, bleu)
// en sortie : nouvelle image a 1 canal, ici rouge
//******************************************************************************
unsigned char* get_red_channel(unsigned char* raw, int width, int height) {

   int h,w;
   unsigned char* rawRed = new unsigned char[width*height]; 

   // on recupere les plans de l'image d'origine
   for ( h = 0; h < (int)height; h++) {
     for ( w = 0; w < (int)width; w++) {
        rawRed[h*width+w] = raw[h*width*3 + w*3];
     }
   }
   return rawRed;
}

//******************************************************************************
// Fonction get_green_channel
// en entree : image a 3 canaux (rouge, vert, bleu)
// en sortie : nouvelle image a 1 canal, ici vert
//******************************************************************************
unsigned char* get_green_channel(unsigned char* raw, int width, int height) {

   int h,w;
   unsigned char* rawGreen = new unsigned char[width*height]; 

   // on recupere les plans de l'image d'origine
   for ( h = 0; h < (int)height; h++) {
     for ( w = 0; w < (int)width; w++) {
        rawGreen[h*width+w] = raw[h*width*3 + w*3 + 1];	
     }
   }
   return rawGreen;
}

//******************************************************************************
// Fonction get_blue_channel
// en entree : image a 3 canaux (rouge, vert, bleu)
// en sortie : nouvelle image a 1 canal, ici bleu
//******************************************************************************
unsigned char* get_blue_channel(unsigned char* raw, int width, int height) {

   int h,w;
   unsigned char* rawBlue = new unsigned char[width*height]; 

   // on recupere les plans de l'image d'origine
   for ( h = 0; h < (int)height; h++) {
     for ( w = 0; w < (int)width; w++) {
        rawBlue[h*width+w] = raw[h*width*3 + w*3 + 2];
     }
   }
   return rawBlue;
}

//******************************************************************************
// Fonction filtrage_image
// en entree : image a 3 canaux (rouge, vert, bleu)
// en sortie : nouvelle image seuillee en fonction du canal
// canal = 0 => filtrage de l'image par des seuils rouges
// canal = 1 => filtrage de l'image par des seuils verts
// canal = 1 => filtrage de l'image par des seuils bleus
//******************************************************************************
unsigned char* filtrage_image(unsigned char* image, int width, int height, int canal) { 

   unsigned char* rawRVB[3]; // Plans rouge, vert, bleu de l'image

   // on recupere les plans de l'image d'origine
   rawRVB[0] = get_red_channel(image,width,height);
   rawRVB[1] = get_green_channel(image,width,height);
   rawRVB[2] = get_blue_channel(image,width,height);

   unsigned char* image_seuillee = seuillage(rawRVB,width,height,canal);
   unsigned char* image_ouverte = ouverture(image_seuillee,width,height);
   unsigned char* image_finale = fermeture(image_ouverte,width,height);

   delete rawRVB[0];
   delete rawRVB[1];
   delete rawRVB[2];
   //return image_seuillee;
   return image_finale;
}

//******************************************************************************
// Fonction filtrage_image_rapide
// en entree : image a 3 canaux (rouge, vert, bleu)
// en sortie : nouvelle image seuillee en fonction du canal
// canal = 0 => filtrage de l'image par des seuils rouges
// canal = 1 => filtrage de l'image par des seuils verts
// canal = 1 => filtrage de l'image par des seuils bleus
//******************************************************************************
unsigned char* filtrage_image_rapide(unsigned char* image, int width, int height, int canal) { 

   // on recupere les plans de l'image d'origine

   unsigned char* image_seuillee = seuillage_rapide(image,width,height,canal);
   return image_seuillee;
}

//******************************************************************************
// Fonction filtrage_par_histogramme
// en entree : image a 3 canaux (rouge, vert, bleu)
// en sortie : nouvelle image seuillee en fonction de l'histogramme de probabilite.
// L'histogramme a 32 niveaux de rouge, 32 niveaux de vert, et 32 niveaux de bleu
//******************************************************************************
unsigned char* filtrage_par_histogramme(unsigned char* image, int width, int height, float histogramme[32][32][32]) { 

   unsigned char* rawRVB[3]; // Plans rouge, vert, bleu de l'image

   // on recupere les plans de l'image d'origine
   rawRVB[0] = get_red_channel(image,width,height);
   rawRVB[1] = get_green_channel(image,width,height);
   rawRVB[2] = get_blue_channel(image,width,height);

   unsigned char* image_seuillee = seuillage_par_histogramme(rawRVB,width,height,histogramme);

   delete rawRVB[0];
   delete rawRVB[1];
   delete rawRVB[2];
   return image_seuillee;
}

