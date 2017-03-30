/*
  traitement.hpp

  Header file 
 
 */
#ifndef _TRAITEMENT_H_
#define _TRAITEMENT_H



extern bool estRouge(int rouge, int vert, int bleu);
extern bool estVert(int rouge, int vert, int bleu);
extern bool estBleu(int rouge, int vert, int bleu);
extern void chargement_histogramme( const char* fic, float hist[32][32][32]);   
extern unsigned char* seuillage_par_histogramme (unsigned char** image, int width, int height, float hist[32][32][32]);
extern unsigned char* seuillage (unsigned char** image, int width, int height, int canal);
extern unsigned char* seuillage_rapide (unsigned char* image, int width, int height, int canal);
extern unsigned char* ouverture (unsigned char* image, int width, int height);
extern unsigned char* fermeture (unsigned char* image, int width, int height);
extern unsigned char* erosion (unsigned char* image, int width, int height);
extern unsigned char* dilatation (unsigned char* image, int width, int height);
extern unsigned char** get_chromatic_channels(unsigned char* raw, int width, int height);
extern unsigned char* get_red_channel(unsigned char* raw, int width, int height);
extern unsigned char* get_green_channel(unsigned char* raw, int width, int height);
extern unsigned char* get_blue_channel(unsigned char* raw, int width, int height);
extern unsigned char* filtrage_image(unsigned char* image, int width, int height, int canal);
extern unsigned char* filtrage_image_rapide(unsigned char* image, int width, int height, int canal);
extern unsigned char* filtrage_par_histogramme(unsigned char* image, int width, int height, float histogramme[32][32][32]); 

#endif
