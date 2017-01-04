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
	    turtleBot.move(0.1f, 0.1f);  
	    

		std::cout<<time<<std::endl;
	    time += 0.5f;
        //Launching Callbacks and synchronizing with loop_rate
        ros::spinOnce(); 
        loop_rate.sleep();
	}
    return 0;
}
