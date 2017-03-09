#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <math.h> 

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#define pi 3.14159265358979323846

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


	int n=20;
	int m=2;
	int x[]={10, 5};
    int y[]={10, 5};     
	int t[]={45, 90}; 
	
	int i,j,k,l,o;
	float dist ;
	int pix[n][n];
    float angle[]={0.0,0.0};
    ros::Rate loop_rate(5); // 2Hz 

    while (ros::ok()) 
    {
		ros::spinOnce();
		for (i=0;i<=n;i++) // line
		{
			for (j=0;j<=n;j++) // colonne
			{
				pix[i][j]=0;
				for(k=0;k<m;k++) // amers
				{
					
					
					
				// if((theta==pi/4 && i>=x[k] && j<=y[k]) || (theta==pi/2) || (theta==3*pi/4 && i<=x[k] && j<=y[k]) || (theta==pi && i<=x[k] && j<=y[k]) || (theta==5*pi/4 && i<=x[k] && j>=y[k]) || (theta==-pi/2 && i<=x[k] && j>=y[k]) || (theta==-pi/4 && i<=x[k] && j>=y[k]))
				
					dist = sqrt(pow(i-x[k],2)+pow(j-y[k],2));
					//hypv = 1/cos(t[k]) ;
					//v = (1 tan(t[k])) 
					//distangle = arctan((u*v)/(
					if(i<x[k] && j<y[k])
						angle[k]=180-(asin((y[k]-j)/dist)*180/pi);
					else if (i>x[k] && j<y[k])
						angle[k]=(asin((y[k]-j)/dist))*180/pi;
						else angle[k]=90;

					if(i<x[k] && j>y[k])
						angle[k]=-180+(asin((j-y[k])/dist)*180/pi);
					else if (i>x[k] && j>y[k])
						angle[k]=-(asin((j-y[k])/dist))*180/pi;
						else angle[k]=90;

					//printf("Angle : %f", angle);
					//printf("\n theta : %d", t[k]);
					if(dist<4.0 && dist>1.0 && (angle[k]<=t[k] && angle[k]>=0)) 
						pix[i][j]+=30;
					else 
						if(dist == 0.0)
							pix[i][j]= 255;
					
				}
				if(i<n&&j<n)
				printf("%d\t", pix[i][j]);
			}
			printf("\n");
		}

		printf("\n");
				
		loop_rate.sleep();
    }
	return 0;
}
