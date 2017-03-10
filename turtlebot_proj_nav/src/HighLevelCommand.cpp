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
    closestMarkerId.data = 0;
    GlobalGoalMarkerId.data = 5;
}

HighLevelCommand::HighLevelCommand(ros::NodeHandle& node, int finalGoal):
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
    closestMarkerId.data = 0;
    GlobalGoalMarkerId.data = finalGoal;
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
      
        float dist = 100.0;
        Graph g = xmlToGraph("graph.xml");
        typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
        std::pair<vertex_iter, vertex_iter> vertexPair;
        for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
        {
            if(distance(g[*vertexPair.first].x, g[*vertexPair.first].y, currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y)<dist) 
            {
                closestMarkerId.data = g[*vertexPair.first].id;
                dist = distance(g[*vertexPair.first].x, g[*vertexPair.first].y, currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y);
            }
        }
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
    if(closestMarkerId.data == GlobalGoalMarkerId.data) return true; 
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
    
    NodeProperty marker = nextNode(closestMarkerId.data, GlobalGoalMarkerId.data, "graph.xml");

    currentGoal.header.seq = 1;
    currentGoal.header.stamp = ros::Time::now();
    currentGoal.header.frame_id = "map";
    
    currentGoal.pose.position.x = marker.x;    
    currentGoal.pose.position.y = marker.y;    
        
    currentGoal.pose.position.z = currentLocation.pose.pose.position.z;
    currentGoal.pose.orientation = currentLocation.pose.pose.orientation;
    
    pubGoal.publish(currentGoal);
}

void HighLevelCommand::findGlobalGoal()
{
    
}


