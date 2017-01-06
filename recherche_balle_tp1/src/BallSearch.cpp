#include "BallSearch.hpp"
#include "command.h"

BallSearch::BallSearch(ros::NodeHandle& node):
    //Publishers
    publisherBallReference(node.advertise<recherche_balle_tp1::command>("/nav/ballref", 1))
{
    
}

BallSearch::~BallSearch(){}

void BallSearch::sendBallReference(const float linearVelocity, const float angularVelocity, const float distance, const float angle)
{
    recherche_balle_tp1::command msgBallReference;
    msgBallReference.linearVelocity = linearVelocity;
    msgBallReference.angularVelocity = angularVelocity;
    msgBallReference.distance = distance;
    msgBallReference.angle = angle;
    publisherBallReference.publish(msgBallReference);
}
