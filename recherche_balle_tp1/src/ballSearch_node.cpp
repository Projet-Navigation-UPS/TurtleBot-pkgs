#include "ros/ros.h"
#include "BallSearch.hpp"

int main(int argc, char **argv)
{
    std::cout<<"Launching ballSearch_node ..."<<std::endl;
    ros::init(argc, argv, "testSend");
    ros::NodeHandle n;
    ros::Rate loop_rate(2); // 2Hz 

    BallSearch ballSearch(n);
    
    float linearVelocity = 0.1f;
    float distance = 0.5f;
    float angularVelocity = 1.0f;
    float angle = 0.5f;
    
    while(ros::ok())
	{
	    
		ballSearch.sendBallReference(linearVelocity, angularVelocity, distance, angle);
				
       
	    //Launching Callbacks and synchronizing with loop_rate
	    ros::spinOnce(); 
	    loop_rate.sleep();
	}
    return 0;
}
