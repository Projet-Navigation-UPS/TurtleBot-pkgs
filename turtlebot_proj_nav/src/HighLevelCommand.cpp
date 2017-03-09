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
    pubSound(node.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1)),
    pubGoal(node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1))
{
    locationAvailable.data = false;
    goalReached.data = false;
    //tfListener.lookupTransform("map", "odom", ros::Time::now(), transform);
}

HighLevelCommand::~HighLevelCommand(){}


//Callbacks
void HighLevelCommand::callbackLocation(const nav_msgs::Odometry& msg)
{
    //std::cout<<"MESSAGE"<<std::endl;
    //std::cout<<msg.pose.pose.position<<std::endl;
    //std::cout<<msg.pose.pose.orientation<<std::endl;
    currentLocation = msg;
           
    geometry_msgs::PoseStamped location;
    location.header = msg.header;
    location.pose = msg.pose.pose;
            
    geometry_msgs::PoseStamped transformed_location;
    tfListener.transformPose("map", location, transformed_location);      
    currentLocation.pose.pose = transformed_location.pose;
    //std::cout<<"TRANSFORMED"<<std::endl;
    //std::cout<<currentLocation.pose.pose.position<<std::endl;
    //std::cout<<currentLocation.pose.pose.orientation<<std::endl;

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
    
    if(moveBaseActionResult.status.status == 3) 
    {
      playSound(SOUND_ON);
      std::cout<<"MoveBaseActionResult"<<std::endl;
      std::cout<<msg<<std::endl;
      goalReached.data = true;
     }
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

bool HighLevelCommand::finalGoal()
{ 
    if(distance(X_GOAL3, Y_GOAL3, currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y) < 0.3) return true; 
}

bool HighLevelCommand::intermediateGoal()
{ 
    return goalReached.data; 
}


//Other
float HighLevelCommand::distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

int HighLevelCommand::nearestGoal(float x, float y)
{
    int goal;
    float crit = 100.0;
    
    if((distance(X_GOAL1, Y_GOAL1, x, y) < crit) && (distance(X_GOAL1, Y_GOAL1, x, y) > 0.5)) 
    {
      crit = distance(X_GOAL1, Y_GOAL1, x, y);
      goal = 1;
    }
    if((distance(X_GOAL2, Y_GOAL2, x, y) < crit) && (distance(X_GOAL2, Y_GOAL2, x, y) > 0.5))
    {
      crit = distance(X_GOAL2, Y_GOAL2, x, y);
      goal = 2;
    }
    if((distance(X_GOAL3, Y_GOAL3, x, y) < crit ) && (distance(X_GOAL3, Y_GOAL3, x, y) < 0.5) )
    {
      crit = distance(X_GOAL3, Y_GOAL3, x, y);
      goal = 3;
    }
    
    return goal;
}

//Sound Commands
void HighLevelCommand::playSound(int sound)
{
    mobileBaseCommandsSound.value = sound;
    pubSound.publish(mobileBaseCommandsSound);
}


//Hight Level Commands
void HighLevelCommand::sendGoal()
{
    goalReached.data = false;

    currentGoal.header.seq = 1;
    currentGoal.header.stamp = ros::Time::now();
    currentGoal.header.frame_id = "map";
    
     switch (nearestGoal(currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y))
        {
            case 1:
                ROS_INFO("Goal 1...");
                currentGoal.pose.position.x = X_GOAL1;
                currentGoal.pose.position.y = Y_GOAL1;
                break;
            case 2:
                ROS_INFO("Goal 2...");
                currentGoal.pose.position.x = X_GOAL2;
                currentGoal.pose.position.y = Y_GOAL2;
                break;
            case 3:
                ROS_INFO("Goal 3...");
                currentGoal.pose.position.x = X_GOAL3;
                currentGoal.pose.position.y = Y_GOAL3;
                break;
            default:
                ROS_INFO("Default...");
                break;

        }
        
    currentGoal.pose.position.z = currentLocation.pose.pose.position.z;
    currentGoal.pose.orientation = currentLocation.pose.pose.orientation;
    
    pubGoal.publish(currentGoal);
}




