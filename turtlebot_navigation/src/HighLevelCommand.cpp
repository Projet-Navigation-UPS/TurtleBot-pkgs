#include "HighLevelCommand.hpp"

#include <iostream>
#include <vector>
#include <cmath> 

HighLevelCommand::HighLevelCommand(ros::NodeHandle& node):
    //Subsribers
    subLocationReady(node.subscribe("/nav/Location", 1, &HighLevelCommand::callbackLocationReady,this)),
    subPathFound(node.subscribe("/nav/PathToFollow", 1, &HighLevelCommand::callbackPathFound,this)),
    subCommandFinished(node.subscribe("/nav/CommandFinished", 1, &HighLevelCommand::callbackCommandFinished,this)),
    //Publishers
    pubPlanPath(node.advertise<actionlib_msgs::GoalStatus>("/nav/PlanPath", 1)),
    pubFollowPath(node.advertise<actionlib_msgs::GoalStatus>("/nav/FollowPath", 1))
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
    std::cout<<currentLocation<<std::endl;
}

void HighLevelCommand::callbackPathFound(const nav_msgs::Path& msg)
{
    pathFound.data = true;
    currentGoal = msg.poses.back();
    std::cout<<currentGoal<<std::endl;
}

void HighLevelCommand::callbackCommandFinished(const std_msgs::Bool& msg)
{
    commandFinished = msg;
    std::cout<<commandFinished<<std::endl;
}


//States
bool HighLevelCommand::location_Ready()
{
    return locationReady.data;
}
bool HighLevelCommand::path_Found()
{
    return pathFound.data;
}
bool HighLevelCommand::command_Finished()
{
    return commandFinished.data;
}
bool HighLevelCommand::near_Goal()
{
    if(abs(currentGoal.pose.position.x-currentLocation.pose.position.x)<0.1)
    {
        if(abs(currentGoal.pose.position.y-currentLocation.pose.position.y)<0.1)
        {
            if(abs(currentGoal.pose.position.z-currentLocation.pose.position.z)<0.1)
            {
                if(abs(currentGoal.pose.orientation.x-currentLocation.pose.orientation.x)<0.1)
                {
                    if(abs(currentGoal.pose.orientation.y-currentLocation.pose.orientation.y)<0.1)
                    {
                        if(abs(currentGoal.pose.orientation.z-currentLocation.pose.orientation.z)<0.1)
                        {
                            if(abs(currentGoal.pose.orientation.w-currentLocation.pose.orientation.w)<0.1)
                            {
                                nearGoal.data = true;
                            }
                            else nearGoal.data = false;
                        }
                        else nearGoal.data = false;
                    }
                    else nearGoal.data = false;
                }
                else nearGoal.data = false;
            }
            else nearGoal.data = false;
        }
        else nearGoal.data = false;
    }
    else nearGoal.data = false;

    return nearGoal.data;
}


//Hight Level Commands
void HighLevelCommand::plan_Path()
{
    planPath.goal_id.stamp = ros::Time::now();
    planPath.goal_id.id = "path_planning";
    planPath.status = 1;
    planPath.text = "";
    //std::cout<<planPath<<std::endl;
    pubPlanPath.publish(planPath);
}

void HighLevelCommand::follow_Path()
{
    followPath.goal_id.stamp = ros::Time::now();
    followPath.goal_id.id = "path_following";
    followPath.status = 1;
    followPath.text = "";
    //std::cout<<followPath<<std::endl;
    pubFollowPath.publish(followPath);
    pathFound.data = false; 
}


//
void HighLevelCommand::publish()
{
    pubPlanPath.publish(planPath);
    pubFollowPath.publish(followPath);
}


