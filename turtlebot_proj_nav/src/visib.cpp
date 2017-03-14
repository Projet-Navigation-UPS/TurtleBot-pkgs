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
 
using namespace std;


int main(int argc, char **argv)
{
    ROS_INFO("Launching visibility ...");
    ros::init(argc, argv, "visib");
    ros::NodeHandle node;

	/*

    ros::Publisher pubPath(node.advertise<nav_msgs::Path>("/nav/PathToFollow", 1));
    ros::Publisher pubLocation(node.advertise<nav_msgs::Odometry>("/nav/Location", 1));
    ros::Publisher pubCmdFinished(node.advertise<std_msgs::Bool>("/nav/CommandFinished", 1));
    ros::Publisher pubGoal(node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1));

    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose1, pose2, pose3, goal;
    nav_msgs::Odometry location;
    std_msgs::Bool commandFinished;
    
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 0;
    pose1.pose.orientation.x = 0;
    pose1.pose.orientation.y = 0;
    pose1.pose.orientation.z = 0;
    pose1.pose.orientation.w = 0;
    
    pose2.pose.position.x = 1;
    pose2.pose.position.y = 1;
    pose2.pose.position.z = 1;
    pose2.pose.orientation.x = 1;
    pose2.pose.orientation.y = 1;
    pose2.pose.orientation.z = 1;
    pose2.pose.orientation.w = 1;
    
    pose3.pose.position.x = 2;
    pose3.pose.position.y = 2;
    pose3.pose.position.z = 2;
    pose3.pose.orientation.x = 2;
    pose3.pose.orientation.y = 2;
    pose3.pose.orientation.z = 2;
    pose3.pose.orientation.w = 2;
    
    path.poses.push_back(pose1);
    path.poses.push_back(pose2);
    path.poses.push_back(pose3);
    
    location.pose.pose.position.x = 2.05;
    location.pose.pose.position.y = 2.05;
    location.pose.pose.position.z = 2.05;
    location.pose.pose.orientation.x = 2.05;
    location.pose.pose.orientation.y = 2.05;
    location.pose.pose.orientation.z = 2.05;
    location.pose.pose.orientation.w = 2.05;
    
    
    
    goal.header.seq = 1;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = 3;
    goal.pose.position.y = 2;
    goal.pose.position.z = 0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    commandFinished.data = true;
    float time = 0;
    while (ros::ok()) 
    {
        ros::spinOnce();
        
        //pubLocation.publish(location);
        //std::cout<<location<<std::endl;
        
        if(time>=5 && time<5.5)
        {
            //pubPath.publish(path);
            //std::cout<<path<<std::endl;
            pubGoal.publish(goal);
            std::cout<<goal<<std::endl;
        }
        
        if(time>10)
        {
            //pubCmdFinished.publish(commandFinished);
        }
        
        
        //std::cout<<time<<std::endl;
        time = time + 0.5;
        loop_rate.sleep();
    }*/


	int n1=720;
	int n2=700;
	int m=3;
	int x[]={100,200,150};//colonne
    int y[]={100,600,300};//ligne     
	float t[]={45,-20,-20}; 
	
	int i,j,k,l,o,p=0,q=0,r=0,s=0,u=0;
	float dist=1.0 ;
	int pix[n1][n2];
	int v[m][m];
	float yn[m];
    float angle[]={0.0,0.0,0.0};
	float alphamax[]={30,30,30};
	int distancemax=3000;
	int distancemin=10;

	float distancecm=130;
	//1px = 0.494134897 cm
	// x px = 337cm

	ros::Publisher pubVisib(node.advertise<std_msgs::Int8>("/nav/Visib", 1));

	std_msgs::Int8 visib;
	visib.data=0;

	distancemax=distancecm/0.494134897;
	printf("Dmax = %d\n",distancemax);
	distancemin=35/0.494134897;	
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
				fichier << "720" << " " << "700" << endl;
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
				//printf(".");
				
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

			
			//printf("\n");
		}

		//printf("i=%d\tj=%d\n",i,j); //Verification iterations ligne et colonne
		//printf("q=%d\n",q); //Verification nb totale d'iterations
		if(r<1)      		        
        	{	
			fichier.close();
			printf("Carte genere correctement, fermeture du fichier, veuillez patienter environ 10 sec... \n");
			r=1;
		}
		
		// --------------- Publish the number of markers ---------

		for (i=0;i<n2;i++) // line
		{
			for (j=0;j<n1;j++) // colonne
			{
				for(k=0;k<m;k++) // amers
				{
					pix[i][j]=15-3*k;
					if(pix[i][j]<15)
					{
						//printf("Nombre d'amers visibles : %d\n",k);
						visib.data=k;
						ROS_INFO("Nombre d'amers visibles : %d", visib.data);
						pubVisib.publish(visib);						
					}
				}
			}
		}

		cout << "Ending visibility " << endl;
		
		loop_rate.sleep();
    }
	return 0;
}
