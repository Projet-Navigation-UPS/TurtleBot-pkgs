#include "HighLevelCommand.hpp"


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
    
    //planPath.data = false;
    //followPath.data = false;
}

HighLevelCommand::~HighLevelCommand(){}


//Callbacks
void HighLevelCommand::callbackLocationReady(const nav_msgs::Odometry& msg)
{
    locationReady.data = true;
}

void HighLevelCommand::callbackPathFound(const nav_msgs::Path& msg)
{
    pathFound.data = true;
}

void HighLevelCommand::callbackCommandFinished(const std_msgs::Bool& msg)
{
    commandFinished = msg;
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
}


//
void HighLevelCommand::publish()
{
    pubPlanPath.publish(planPath);
    pubFollowPath.publish(followPath);
}


