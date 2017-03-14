#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <math.h> 
#include <std_msgs/Int8.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#define pi 3.14159265358979323846

#include <fstream>

#include "tinyxml.h"
#include <ros/package.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <vector>
#include <string>
#include "graph.hpp"
#include "graph.cpp"


void displayGraphVisib(Graph g, float x1[], float y1[])
{
	int a=0;
    // Display vertices
    std::cout << "Display vertices" <<  std::endl;
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vertexPair;
    for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
        std::cout << g[*vertexPair.first].id << " " << g[*vertexPair.first].label << " " << g[*vertexPair.first].x << " " << g[*vertexPair.first].y << " " << g[*vertexPair.first].orientation << std::endl;

	x1[a]=g[*vertexPair.first].x;
	y1[a]=g[*vertexPair.first].y;
	a+=1;
    }
    
    // Display edges weights
    std::cout << "Display edges weights" <<  std::endl;
    boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);
    typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> edgePair;
    for(edgePair = edges(g); edgePair.first != edgePair.second; ++edgePair.first)
    {
        std::cout << boost::source(*edgePair.first, g) << " --" << EdgeWeightMap[*edgePair.first] << "--> "<< boost::target(*edgePair.first, g) << std::endl;
    }
}


using namespace std;


int main(int argc, char **argv)
{
    ROS_INFO("Launching visibility ...");
    ros::init(argc, argv, "visib");
    ros::NodeHandle node;

	int n1=920;
	int n2=900;
	int m=6;
	int x[]={0,0,0,0,0,0};//colonne
    int y[]={0,0,0,0,0,0};//ligne     
	float t[]={45,-20,-20,-20,-20,-20}; 
	
	int i,j,k,l,o,p=0,q=0,r=0,s=0,u=0,w=0,a;
	float dist=1.0 ;
	int pix[n1][n2];
	int v[m][m];
	float yn[m];
	float x1[5],y1[5];
    float angle[]={0.0,0.0,0.0,0.0,0.0,0.0};
	float alphamax[]={45,45,45,45,45,45};
	int distancemax=3000;
	int distancemin=10;

	float distancecm=200;
	//1px = 0.494134897 cm
	// x px = 337cm

	Graph Graph_test = xmlToGraph("graph.xml");
    displayGraphVisib(Graph_test,x1,y1);

	
	for(a=0;a<m;a++)
	{	
		//printf("*************************** x1 = %lf \t y1 = %lf *********************\n",x1[a],y1[a]);
		x[a]=x1[a]*100/0.494134897;
		y[a]=y1[a]*100/0.494134897;
		//printf("*************************** x1 = %d \t y1 = %d *********************\n",x[a],y[a]);
	}


	ros::Publisher pubVisib(node.advertise<std_msgs::Int8>("/nav/Visib", 1));

	std_msgs::Int8 visib;
	visib.data=0;

	distancemax=distancecm/0.494134897;
	printf("Dmax = %d\n",distancemax);
	distancemin=20/0.494134897;	
	printf("Dmin = %d\n",distancemin);

    	ros::Rate loop_rate(5); // 2Hz 

	ofstream fichier("src/TurtleBot-pkgs/turtlebot_proj_nav/maps/visib.pgm", ios::out | ios::trunc);  
		// ouverture en écriture avec effacement du fichier ouvert

    while (ros::ok()) //&& r<1) 
    {
		ros::spinOnce();
    	
		if(fichier && (r<1))
        		{	
				printf("Création de la carte \n");
				fichier << "P2" << endl;
				fichier << "#Thibaut" << endl;
				fichier << "#Carte de visibilite" << endl;
				fichier << n1 << " " << n2 << endl;
				fichier << "15" << endl;
			}
			//else
                		//cerr << "Impossible d'ouvrir le fichier !" << endl;

    
		for (i=0;i<n2;i++) // line
		{
			for (j=0;j<n1;j++) // colonne
			{
				pix[i][j]=15;
				for(k=0;k<m;k++) // amers
				{
					
					if(t[k]>-90 && t[k]<90) //si theta compris entre -90 degre et 90 degre
						v[k][1]=x[k];
					else if (t[k]<-90 && t[k]>90)
						v[k][1]=-x[k];
						else v[k][1]=0;
				
					yn[k]=sin(t[k]);
					angle[k]=acos(((i-x[k])+((j-y[k])*yn[k]))/(sqrt(pow((i-x[k]),2)+pow((j-y[k]),2))*(sqrt(1+pow(yn[k],2)))))*180/pi;					
					//printf("Angle : %f \n", angle[k]);
					dist = sqrt(pow(i-x[k],2)+pow(j-y[k],2));

					if((dist<distancemax && dist>distancemin) && (angle[k]<alphamax[k] && angle[k]>-alphamax[k]))
						pix[i][j]-=3;
					else 
						if(dist == 0.0)
						{	pix[i][j]=0;
							//printf("Amers :\ni=%d\tj=%d\n",i,j);
							
						}
				}

				//if(i<n&&j<n)
				//printf("%d ", pix[i][j]);
				
				q+=1;
				if(r<1)	//flag pour savoir si l'ecriture s'est deja faite 1 fois
				{	fichier << pix[i][j] << " " ;
					p+=1;
				}
				if(p>=70 && (r<1))// condition de retour a la ligne
				{
					fichier << endl;
					p=0;		
				}
			}
		}

		//printf("i=%d\tj=%d\n",i,j); //Verification iterations ligne et colonne
		//printf("q=%d\n",q); //Verification nb totale d'iterations

		if(r<1)      		        
        	{	
			fichier.close();
			printf("Carte generee correctement, fermeture du fichier, veuillez patienter environ 10 sec... \n");
			r=1;
		}
		
		// --------------- Publish the number of markers ---------

		/*for (i=0;i<10;i++) // line
		{
			for (j=0;j<10;j++) // colonne
			{
				for(k=1;k<m;k++) // amers
				{
					if(pix[i][j]==0)
					{
						visib.data=k;
						//ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						printf("Nombre d'amers visibles : %d \n",k);
						pubVisib.publish(visib);
						printf("Numéro de l amer détectable : %d \n",k-1);						
					}
					else if(pix[i][j]==9)
					{
						visib.data=k;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						printf("Numéros des amers détectables : %d et %d \n",k-1,k);
					}
					else if(pix[i][j]==6)
					{
						visib.data=k;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						printf("Numéros des amers détectables : %d, %d et %d \n",k-1,k,k+1);
					}
					else if(pix[i][j]==3)
					{
						visib.data=k;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);
						printf("Numéros des amers détectables : %d, %d, %d et %d \n",k-1,k,k+1,k+2);
					}
					else
					{
						visib.data=0;
						ROS_INFO("Pas d amer visible : %d", visib.data);
						pubVisib.publish(visib);
					}
				}
			}
		}*/

		cout << "Ending visibility " << endl;
		
		loop_rate.sleep();
    }
	return 0;
}
