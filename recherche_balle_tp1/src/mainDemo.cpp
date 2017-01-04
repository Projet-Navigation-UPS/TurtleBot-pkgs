#include "ros/ros.h"
#include "TurtleBot.hpp"
#include "GraphicServer.hpp"
#include "traitement.hpp"
#include "analyse.hpp"

int main(int argc, char **argv)
{
    std::cout<<"Launching Odometrie ..."<<std::endl;
    ros::init(argc, argv, "recherche_balle_tp1_demo");
    ros::NodeHandle node;
    ros::Rate loop_rate(2); // 2Hz 


    TurtleBot turtleBot(node);
    
    float time = 0.0f;
    
    while(ros::ok())
	{
        
	    
        //Motion example
	    if(time>5 && time<10)  turtleBot.setMobileBaseCommandsVelocity(0.1, 0, 0, 0, 0, 0);           
        
	    if(time>10)  turtleBot.setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, 0);  
	    

		std::cout<<time<<std::endl;
	    time += 0.5f;
        //Launching Callbacks and synchronizing with loop_rate
        ros::spinOnce(); 
        loop_rate.sleep();
	}
    return 0;
}
