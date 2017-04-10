/*
  analyse.hpp
  
  Header file 
 */
#ifndef _ANALYSE_H_
#define _ANALYSE_H_

#include "objet.h"

int Avancer_Etiquetage(unsigned char* image, int width, int height, int h, int w, int Etiquette);
int Etiqueter_Region(unsigned char* image, int width, int height);
int Extract_attributs(Objet * listeObjet, unsigned char * image, int width, int height, int nbObj, int c);
bool est_carre(Objet region);
bool est_inclus(Objet region_verte, Objet region_rouge);

#endif
      
