/*
  BallSearch.hpp
  Bruno Dato, Marine Bouchet & Thibaut Aghnatios 

  Header file 
 
 */
#ifndef _BALLSEARCH_
#define _BALLSEARCH_

#include <ros/ros.h>
#include "traitement.hpp"
#include "analyse.hpp"
#include "std_msgs/Bool.h"
#include "turtlebot_proj_nav/command.h"


class BallSearch 
{
    
private:
    
    // Subscrbers
    ros::Subscriber subscriberCommandBusy;

    // Publishers
    ros::Publisher publisherBallReference;

    // Messages
    std_msgs::Bool command_busy;
 
    // Seekin ball state  
    int etat_recherche;
    
    // Callback
    void callbackCommandBusy(const std_msgs::Bool& msg);
    
public:

    BallSearch(ros::NodeHandle& node);
    ~BallSearch();
    
    // Members
    void sendBallReference(const float linearVelocity, const float angularVelocity, const float distance, const float angle);
    Objet * Recherche_balle(unsigned char* raw, int  width, int height, int couleur);
    void attente(int nsec, int sec);
    bool getCommandState();

};

#endif
