#include "ros/ros.h"
#include "TurtleBot.hpp"
#include "GraphicServer.hpp"
#include "traitement.hpp"
#include "analyse.hpp"

#define FREQ 10

//TurtleBot* turtleBot;

/*void callbackStop(const ros::WallTimerEvent& event)
{
    turtleBot->stop();
    std::cout<<"STOP"<<std::endl;
} */




int main(int argc, char **argv)
{
    std::cout<<"Launching commande ..."<<std::endl;
    ros::init(argc, argv, "commande");
    ros::NodeHandle node;
    ros::Rate loop_rate(FREQ); // 2Hz 


    //turtleBot = new TurtleBot(node);
    
    TurtleBot turtleBot(node);
    
    float linearVelocity = -0.1f;
    float distance = 0.5f;
    bool stopSet = false;
    bool startTime = false;
    ros::WallTime time;
    ros::WallDuration duration(abs(distance/linearVelocity));
    std::cout<<"Duration : "<<duration<<std::endl;
    
    while(ros::ok())
	{
	    //std::cout<<ros::Time::now()<<std::endl;
	    /*turtleBot->move(linearVelocity);
	    if(!stopSet)
		{
		    float duration = abs(distance/linearVelocity);
		    node.createWallTimer(ros::WallDuration(duration), callbackStop, true);
		    stopSet = true;
		}*/
		if(!startTime) 
		{
		    time = ros::WallTime::now();
		    std::cout<<"Begin : "<<time<<std::endl;
		    startTime = true;
		}
		
		if(!stopSet && startTime)
		{
		    turtleBot.move(linearVelocity);
		    
		    if((ros::WallTime::now()-time) > duration) 
		    {
		        stopSet=true;
		        turtleBot.stop();
		        std::cout<<"STOP : "<<ros::WallTime::now()<<std::endl;
		    }
		}
		
		
		
		
       
	    //Launching Callbacks and synchronizing with loop_rate
	    ros::spinOnce(); 
	    loop_rate.sleep();
	}
    return 0;
}
