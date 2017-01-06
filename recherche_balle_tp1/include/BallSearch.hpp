#ifndef _BALLSEARCH_
#define _BALLSEARCH_

#include <ros/ros.h>


class BallSearch 
{
    
private:
    
    //Subscrbers
    

    //Publishers
    ros::Publisher publisherBallReference;

    //Messages
    
    
    
public:

    BallSearch(ros::NodeHandle& node);
    ~BallSearch();
    
    void sendBallReference(const float linearVelocity, const float angularVelocity, const float distance, const float angle); 



 

private:
   
   
    
};

#endif
