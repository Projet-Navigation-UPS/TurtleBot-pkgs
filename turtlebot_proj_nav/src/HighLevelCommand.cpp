#include "HighLevelCommand.hpp"

#include <iostream>
#include <vector>
#include <cmath> 

HighLevelCommand::HighLevelCommand(ros::NodeHandle& node):
    //Subsribers
    subLocation(node.subscribe("/odom", 1, &HighLevelCommand::callbackLocation,this)),
    subGoalStatus(node.subscribe("/move_base/status", 1, &HighLevelCommand::callbackGoalStatus,this)),
    subMoveBaseActionFeedback(node.subscribe("/move_base/feedback", 1, &HighLevelCommand::callbackMoveBaseActionFeedback,this)),
    subMoveBaseActionGoal(node.subscribe("/move_base/goal", 1, &HighLevelCommand::callbackMoveBaseActionGoal,this)),
    subMoveBaseActionResult(node.subscribe("/move_base/result", 1, &HighLevelCommand::callbackMoveBaseActionResult,this)),

    //Publishers
    pubGoal(node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1))
{
    locationAvailable.data = false;
    //tfListener.lookupTransform("map", "odom", ros::Time::now(), transform);
}

HighLevelCommand::~HighLevelCommand(){}


//Callbacks
void HighLevelCommand::callbackLocation(const nav_msgs::Odometry& msg)
{
    std::cout<<"MESSAGE"<<std::endl;
    std::cout<<msg.pose.pose.position<<std::endl;
    std::cout<<msg.pose.pose.orientation<<std::endl;
    currentLocation = msg;
           
    geometry_msgs::PoseStamped location;
    location.header = msg.header;
    location.pose = msg.pose.pose;
            
    geometry_msgs::PoseStamped transformed_location;
    tfListener.transformPose("map", location, transformed_location);      
    currentLocation.pose.pose = transformed_location.pose;
    std::cout<<"TRANSFORMED"<<std::endl;
    std::cout<<currentLocation.pose.pose.position<<std::endl;
    std::cout<<currentLocation.pose.pose.orientation<<std::endl;

    locationAvailable.data = true;
}


void HighLevelCommand::callbackGoalStatus(const actionlib_msgs::GoalStatusArray& msg)
{
    //std::cout<<"GoalStatusArray"<<std::endl;
    //std::cout<<msg<<std::endl;
    goalStatus = msg;
}

void HighLevelCommand::callbackMoveBaseActionResult(const move_base_msgs::MoveBaseActionResult& msg)
{
    //std::cout<<"MoveBaseActionResult"<<std::endl;
    //std::cout<<msg<<std::endl;
    moveBaseActionResult = msg;
}
void HighLevelCommand::callbackMoveBaseActionFeedback(const move_base_msgs::MoveBaseActionFeedback& msg)
{
    //std::cout<<"MoveBaseActionFeedback"<<std::endl;
    //std::cout<<msg<<std::endl;
    moveBaseActionFeedback = msg;
}
void HighLevelCommand::callbackMoveBaseActionGoal(const move_base_msgs::MoveBaseActionGoal& msg)
{
    //std::cout<<"MoveBaseActionGoal"<<std::endl;
    //std::cout<<msg<<std::endl;
    moveBaseActionGoal = msg;
}



//States
bool HighLevelCommand::location()
{
    return locationAvailable.data;
}



//Hight Level Commands
void HighLevelCommand::sendGoal()
{
    currentGoal.header.seq = 1;
    currentGoal.header.stamp = ros::Time::now();
    currentGoal.header.frame_id = "map";
    currentGoal.pose.position.x = currentLocation.pose.pose.position.x + 1;
    currentGoal.pose.position.y = currentLocation.pose.pose.position.y;
    currentGoal.pose.position.z = currentLocation.pose.pose.position.z;
    currentGoal.pose.orientation = currentLocation.pose.pose.orientation;
    
    pubGoal.publish(currentGoal);
}




