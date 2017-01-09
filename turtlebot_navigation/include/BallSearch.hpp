#ifndef _BALLSEARCH_
#define _BALLSEARCH_

#include <ros/ros.h>
#include "traitement.hpp"
#include "analyse.hpp"
#include "std_msgs/Bool.h"


class BallSearch 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subscriberCommandBusy;

    //Publishers
    ros::Publisher publisherBallReference;

    //Messages
    std_msgs::Bool command_busy;
    
    
public:

    BallSearch(ros::NodeHandle& node);
    ~BallSearch();
    
    void sendBallReference(const float linearVelocity, const float angularVelocity, const float distance, const float angle);
    void callbackCommandBusy(const std_msgs::Bool& msg); 
    Objet * Recherche_balle(unsigned char* raw, int  width, int height, int couleur);
    void attente(int nsec, int sec);

};

#endif
