#include "HighLevelCommand.hpp"
#include "command.h"
#include <iostream>
#include <vector>
#include <cmath> 



HighLevelCommand::HighLevelCommand(ros::NodeHandle& node):
    //Services
    clientMarkersVisibility(node.serviceClient<turtlebot_proj_nav::MarkersVisibility>("/nav/markers_visibility")),

    //Subsribers
    subscriberCommandBusy(node.subscribe("/nav/command_busy", 1, &HighLevelCommand::callbackCommandBusy,this)),
    subLocation(node.subscribe("/odom", 1, &HighLevelCommand::callbackLocation,this)),
    subGoalStatus(node.subscribe("/move_base/status", 1, &HighLevelCommand::callbackGoalStatus,this)),
    subMoveBaseActionFeedback(node.subscribe("/move_base/feedback", 1, &HighLevelCommand::callbackMoveBaseActionFeedback,this)),
    subMoveBaseActionGoal(node.subscribe("/move_base/goal", 1, &HighLevelCommand::callbackMoveBaseActionGoal,this)),
    subMoveBaseActionResult(node.subscribe("/move_base/result", 1, &HighLevelCommand::callbackMoveBaseActionResult,this)),
    subMarkerSeen(node.subscribe("/nav/loca/markerSeen", 1, &HighLevelCommand::callbackMarkerSeen,this)),
    //subMarkersVisibility(node.subscribe("/nav/visibility_marker", 1, &HighLevelCommand::callbackMarkersVisibility,this)),

    //Publishers
    /*pubCommandState(node.advertise<std_msgs::Bool>("/nav/command/state", 1)),*/
    pubCommand(node.advertise<turtlebot_proj_nav::command>("/nav/open_loop_command", 1)),
    pubAskForMarker(node.advertise<std_msgs::Empty>("/nav/HLC/askForMarker", 1)),
    pubSound(node.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1)),
    pubGoal(node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1)),
    tfListener(node, ros::Duration(10), true)
{
    currentLocation.pose.pose.position.x = 0;
    currentLocation.pose.pose.position.y = 0;
    currentLocation.pose.pose.position.z = 0;
    currentLocation.pose.pose.orientation.x = 0;
    currentLocation.pose.pose.orientation.y = 0;
    currentLocation.pose.pose.orientation.z = 0;
    currentLocation.pose.pose.orientation.w = 1;
    seekingMarkerState = 0;
    markerSeen.data = -1;
    closestMarkerId.data = -1;
    GlobalGoalMarkerId.data = 5;
    commandBusy.data = true;
    locationAvailable.data = false;
    goalReached.data = false;
    responseMarker.data=false;
    askMarker.data=false;
}

HighLevelCommand::HighLevelCommand(ros::NodeHandle& node, int finalGoal):
    //Services
    clientMarkersVisibility(node.serviceClient<turtlebot_proj_nav::MarkersVisibility>("/nav/markers_visibility")),

    //Subsribers
    subscriberCommandBusy(node.subscribe("/nav/command_busy", 1, &HighLevelCommand::callbackCommandBusy,this)),
    subLocation(node.subscribe("/odom", 1, &HighLevelCommand::callbackLocation,this)),
    subGoalStatus(node.subscribe("/move_base/status", 1, &HighLevelCommand::callbackGoalStatus,this)),
    subMoveBaseActionFeedback(node.subscribe("/move_base/feedback", 1, &HighLevelCommand::callbackMoveBaseActionFeedback,this)),
    subMoveBaseActionGoal(node.subscribe("/move_base/goal", 1, &HighLevelCommand::callbackMoveBaseActionGoal,this)),
    subMoveBaseActionResult(node.subscribe("/move_base/result", 1, &HighLevelCommand::callbackMoveBaseActionResult,this)),
    subMarkerSeen(node.subscribe("/nav/loca/markerSeen", 1, &HighLevelCommand::callbackMarkerSeen,this)),
    //subMarkersVisibility(node.subscribe("/nav/markers_visibility", 1, &HighLevelCommand::callbackMarkersVisibility,this)),

    //Publishers
    /*pubCommandState(node.advertise<std_msgs::Bool>("/command/state", 1)),*/
    pubCommand(node.advertise<turtlebot_proj_nav::command>("/nav/open_loop_command", 1)),
    pubAskForMarker(node.advertise<std_msgs::Empty>("/nav/HLC/askForMarker", 1)),
    pubSound(node.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1)),
    pubGoal(node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1)),
    tfListener(node, ros::Duration(10), true)
{
    currentLocation.pose.pose.position.x = 0;
    currentLocation.pose.pose.position.y = 0;
    currentLocation.pose.pose.position.z = 0;
    currentLocation.pose.pose.orientation.x = 0;
    currentLocation.pose.pose.orientation.y = 0;
    currentLocation.pose.pose.orientation.z = 0;
    currentLocation.pose.pose.orientation.w = 1;
    seekingMarkerState = 0;
    markerSeen.data = -1;
    closestMarkerId.data = -1;
    GlobalGoalMarkerId.data = finalGoal;
    commandBusy.data = true;
    locationAvailable.data = false;
    goalReached.data = false;
    responseMarker.data=false;
    askMarker.data=false;
}

HighLevelCommand::~HighLevelCommand(){}


//Callbacks

void HighLevelCommand::callbackCommandBusy(const std_msgs::Bool& msg)
{
    commandBusy = msg;
}

void HighLevelCommand::callbackMarkerSeen(const std_msgs::Int16& msg)
{
    responseMarker.data=true;
    askMarker.data=false;
    markerSeen = msg;
    //std::cout<<msg<<std::endl;
}

void HighLevelCommand::callbackLocation(const nav_msgs::Odometry& msg)
{
    currentLocation = msg;
    currentLocation.header.frame_id = "map";       
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
        //std::cout<<"MoveBaseActionResult"<<std::endl;
        //std::cout<<msg<<std::endl;
        goalReached.data = true;
    }
    seekingMarkerState = 0;
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
int HighLevelCommand::marker()
{
    //std::cout<<markerSeen<<std::endl;
    if(markerSeen.data != -1) 
    {
        playSound(SOUND_OFF);
        seekingMarkerState=0;
        closestMarkerId = markerSeen; 
    }
    responseMarker.data=false;
    return markerSeen.data;   
}

int HighLevelCommand::markersVisibility()
{
    turtlebot_proj_nav::MarkersVisibility srv;
    transformLocationFromOdomToMap();
    srv.request.x = currentLocation.pose.pose.position.x;
    srv.request.y = currentLocation.pose.pose.position.y;
    if (clientMarkersVisibility.call(srv))
    {
        ROS_INFO("%d visible markers", (int)srv.response.markers);
        return srv.response.markers;
    }
    else
    {
        ROS_ERROR("Failed to call service markers_visibility");
        return -1;
    }
}

bool HighLevelCommand::location()
{
    return locationAvailable.data;
}

bool HighLevelCommand::markerResponse()
{
    return responseMarker.data;
}

bool HighLevelCommand::getAskMarker()
{
    return askMarker.data;
}

bool HighLevelCommand::finalGoal()
{ 
    if(closestMarkerId.data == GlobalGoalMarkerId.data) 
    {
        playSound(SOUND_CLEANINGEND);
        return true;
    } 
    else return false;
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


//Hight Level Commands
void HighLevelCommand::playSound(int sound)
{
    mobileBaseCommandsSound.value = sound;
    pubSound.publish(mobileBaseCommandsSound);
}

void HighLevelCommand::sendDistanceAndAngleCommand(const float linearVelocity, const float angularVelocity, const float distance, const float angle)
{
    if (!commandBusy.data)
    {
        turtlebot_proj_nav::command msg;
        msg.linearVelocity = linearVelocity;
        msg.angularVelocity = angularVelocity;
        msg.distance = distance;
        msg.angle = angle;
        pubCommand.publish(msg);
        ROS_INFO("Command sent...");
    }
    else ROS_INFO("Command busy...");
}

int HighLevelCommand::seekMarker()
{
    if (!commandBusy.data)
     {
     
     switch (seekingMarkerState)
        {
            case 0:
                ROS_INFO("Turning left at Pi/4... ");
                sendDistanceAndAngleCommand(0, 1.5, 0, PI/4);
                seekingMarkerState = 1;
                break;
            case 1:
                ROS_INFO("Turning right at Pi/2... ");
                sendDistanceAndAngleCommand(0, -1.5, 0, PI/2);
                seekingMarkerState = 2;
                break;
            case 2:
                ROS_INFO("Turning left at 3Pi/4... ");
                sendDistanceAndAngleCommand(0, 1.5, 0, 3*PI/4);
                seekingMarkerState = 3;
                break;
            case 3:
                ROS_INFO("Turning right at Pi... ");
                sendDistanceAndAngleCommand(0, -1.5, 0, PI);
                seekingMarkerState = 4;
                break;
            case 4:
                ROS_INFO("Turning left at de 5Pi/4... ");
                sendDistanceAndAngleCommand(0, 1.5, 0, 5*PI/4);
                seekingMarkerState = 5;
                break;
            case 5:
                ROS_INFO("Turning right at de 3Pi/2... ");
                sendDistanceAndAngleCommand(0, -1.5, 0, 3*PI/2);
                seekingMarkerState = 6;
                break;
            case 6:
                ROS_INFO("Turning left at de 7Pi/4... ");
                sendDistanceAndAngleCommand(0, 1.5, 0, 7*PI/4);
                seekingMarkerState = 7;
                break;
            case 7:
                ROS_INFO("Abort seeking... ");
                break;
            default:
                ROS_INFO("Abort seeking... ");
                break;

        }
        return seekingMarkerState;
     }
    else 
    {
        ROS_INFO("Command busy...");
        return -1;
    }
    
}

void HighLevelCommand::transformLocationFromOdomToMap()
{
    tf::StampedTransform transformMapOdom, transformMapRobot, transformOdomRobot;
    tfListener.lookupTransform("/map", "/odom", ros::Time(0), transformMapOdom);
	transformMapRobot = transformMapOdom;
	
    transformOdomRobot.setOrigin(tf::Vector3(currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y, currentLocation.pose.pose.position.y));
    transformOdomRobot.setRotation(tf::Quaternion( currentLocation.pose.pose.orientation.x, currentLocation.pose.pose.orientation.y, currentLocation.pose.pose.orientation.z, currentLocation.pose.pose.orientation.w));
    
    transformMapRobot *= transformOdomRobot; 
    tf::poseTFToMsg(transformMapRobot, currentLocation.pose.pose);
}


void HighLevelCommand::sendGoal()
{
    transformLocationFromOdomToMap();

    markerSeen.data =-1;
    goalReached.data = false;
    NodeProperty marker = nextNode(closestMarkerId.data, GlobalGoalMarkerId.data, "graph.xml");
    currentGoal.header.seq = 1;
    currentGoal.header.stamp = ros::Time::now();
    currentGoal.header.frame_id = "map";
    currentGoal.pose.position.x = marker.x + 0.1*cos(marker.orientation);    
    currentGoal.pose.position.y = marker.y + 0.1*sin(marker.orientation);    
    currentGoal.pose.position.z = currentLocation.pose.pose.position.z;
    currentGoal.pose.orientation.x = 0;
    currentGoal.pose.orientation.y = 0;
    currentGoal.pose.orientation.z = 0;
    currentGoal.pose.orientation.w = 1;
    ROS_INFO("Next goal (%lf,%lf)",currentGoal.pose.position.x,currentGoal.pose.position.y);
    pubGoal.publish(currentGoal);
}

void HighLevelCommand::findGlobalGoal()
{
    
}

void HighLevelCommand::askForMarker()
{
    askMarker.data=true;
    pubAskForMarker.publish(empty);
}





