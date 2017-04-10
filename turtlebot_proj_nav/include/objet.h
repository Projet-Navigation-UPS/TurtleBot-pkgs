/*
  objet.hpp

  Header file 
  Definition of the class Objet used to define the ball caracteristics when it's found
 
 */
#define PI 3.14159
#define COMPACT_MIN 0.5 // Compacité minimum pour une région
#define SURF_MIN 300 // Surface minimum pour une région
#define MBR_FILL_MIN 0.65 // Rapport surface sur boite englobante


class Objet { // Structure pour stocker les attributs des régions segmentées

public :

	int Ucg, Vcg; // Coordonnées du centre gravité
	int Perimetre; // Périmètre
	double Compactness; // Facteur de compacité
	int Surface; // Aire totale
	double Mbr_Fill; // Taux de remplissage
	int Hmin, Hmax, Wmin, Wmax; // Rectangle englobant
	int Color; // Indice du plan couleur
	int Bord; // Indicateur de bord image
	double Rho;   // coordonnees cylindriques de l'objet vu par la camera
	double Theta; // angle entre axe des x du robot et position sur l'image de l'objet (en degres)
	double Dist; // distance à l'objet en cm et la caméra
	int Sens; // sens de la fleche // 1 : droite, -1 : gauche, 0 : non determine
	double Angle_de_vue; // 0° => robot perpendiculaire au panneau
	int Reussite; // plus cette valeur est grande, plus cet objet nous interesse


};

