#include "HighLevelCommand.hpp"

#include <iostream>
#include <vector>
#include <cmath> 

HighLevelCommand::HighLevelCommand(ros::NodeHandle& node):
    //Subsribers
    subLocationReady(node.subscribe("/odom", 1, &HighLevelCommand::callbackLocationReady,this)),
    //Publishers
    pubGoal(node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1))
{
    locationReady.data = false;
    pathFound.data = false; 
    commandFinished.data = false; 
    nearGoal.data = false;
    
    //planPath.data = false;
    //followPath.data = false;
}

HighLevelCommand::~HighLevelCommand(){}


//Callbacks
void HighLevelCommand::callbackLocationReady(const nav_msgs::Odometry& msg)
{
    locationReady.data = true;
    currentLocation.pose = msg.pose.pose;
    //std::cout<<currentLocation<<std::endl;
}






//States
bool HighLevelCommand::location_Ready()
{
    return locationReady.data;
}



//Hight Level Commands



//
void HighLevelCommand::publish()
{
    pubPlanPath.publish(planPath);
    pubFollowPath.publish(followPath);
}


