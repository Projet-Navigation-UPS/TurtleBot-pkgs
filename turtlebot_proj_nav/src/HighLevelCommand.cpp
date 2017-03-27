#include "HighLevelCommand.hpp"
#include <iostream>
#include <vector>
#include <cmath> 


HighLevelCommand::HighLevelCommand(ros::NodeHandle& node, float x_finalGoal, float y_finalGoal):
    //Services
    clientMarkersVisibility(node.serviceClient<turtlebot_proj_nav::MarkersVisibility>("/nav/visibility_marker")),

    //Subsribers
    subscriberCommandBusy(node.subscribe("/nav/command_busy", 1, &HighLevelCommand::callbackCommandBusy,this)),
    subLocation(node.subscribe("/odom", 1, &HighLevelCommand::callbackLocation,this)),
    subMoveBaseActionResult(node.subscribe("/move_base/result", 1, &HighLevelCommand::callbackMoveBaseActionResult,this)),
    subMarkerSeen(node.subscribe("/nav/loca/markerSeen", 1, &HighLevelCommand::callbackMarkerSeen,this)),

    //Publishers
    pubCommand(node.advertise<turtlebot_proj_nav::command>("/nav/open_loop_command", 1)),
    pubAskForMarker(node.advertise<std_msgs::Empty>("/nav/HLC/askForMarker", 1)),
    pubSound(node.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1)),
    pubGoal(node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1)),
    tfListener(node, ros::Duration(10), true)
{
    seekingMarkerState = 0;
    markerSeen.data = -1;
    closestMarkerId.data = -1;
    globalGoalMarkerId.data = -1;
    commandBusy.data = true;
    locationAvailable.data = false;
    goalReached.data = false;
    responseMarker.data=false;
    askMarker.data=false;
    FinalGoalX.data= x_finalGoal;
    FinalGoalY.data= y_finalGoal;
    goalAborted.data=false;
}

HighLevelCommand::~HighLevelCommand(){}

//Callbacks
void HighLevelCommand::callbackCommandBusy(const std_msgs::Bool& msg)
{
    commandBusy = msg;
}

void HighLevelCommand::callbackMarkerSeen(const std_msgs::Int16& msg)
{
    responseMarker.data = true;
    askMarker.data = false;
    markerSeen = msg;
    //std::cout<<msg<<std::endl;
}

void HighLevelCommand::callbackLocation(const nav_msgs::Odometry& msg)
{
    currentLocation = msg;
    currentLocation.header.frame_id = "map";       
    locationAvailable.data = true;
    //std::cout<<msg<<std::endl;
}

void HighLevelCommand::callbackMoveBaseActionResult(const move_base_msgs::MoveBaseActionResult& msg)
{
    //std::cout<<"MoveBaseActionResult"<<std::endl;
    //std::cout<<msg<<std::endl;
    moveBaseActionResult = msg;
    
    if(moveBaseActionResult.status.status == 3) // Goal reached
    {
        playSound(SOUND_ON);
        goalReached.data = true;
    }
    else if(moveBaseActionResult.status.status == 4) // Goal aborted
    {
        playSound(SOUND_ERROR);
        goalAborted.data = true;
    }
    seekingMarkerState = 0;
}


//States
int HighLevelCommand::markerID()
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

bool HighLevelCommand::getCommandState()
{
    return commandBusy.data;
}

bool HighLevelCommand::getMarkerResponse()
{
    return responseMarker.data;
}

bool HighLevelCommand::getGoalAborted()
{
    return goalAborted.data;
}

bool HighLevelCommand::getMarkerAsked()
{
    return askMarker.data;
}

bool HighLevelCommand::finalGoal(float threshold)
{ 
    transformLocationFromOdomToMap();
    if(distance(currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y, FinalGoalX.data, FinalGoalY.data) < threshold) 
    {
        playSound(SOUND_CLEANINGEND);
        return true;
    } 
    else return false;
}

bool HighLevelCommand::finalMarkerGoal()
{ 
    if(closestMarkerId.data == globalGoalMarkerId.data) 
    {
        globalGoalMarkerId.data = -1;
        return true;
    }
    else return false;
}

bool HighLevelCommand::intermediateGoal()
{ 
    return goalReached.data; 
}


//Other useful functions
float HighLevelCommand::distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
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

int HighLevelCommand::getClosestMarkerToXYPosition(float x, float y)
{
    float dist = 100.0;
    int closestGoalMarkerId = -1;
    Graph g = xmlToGraph("graph.xml");
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vertexPair;
    for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
        if(distance(g[*vertexPair.first].x, g[*vertexPair.first].y, x, y)<dist) 
        {
            closestGoalMarkerId = g[*vertexPair.first].id;
            dist = distance(g[*vertexPair.first].x, g[*vertexPair.first].y, x, y);
        }
    }
    return closestGoalMarkerId;
}


//Commands
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


// High level commands

int HighLevelCommand::init(float threshold)
{
    transformLocationFromOdomToMap();
    if(distance(currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y, FinalGoalX.data, FinalGoalY.data) > threshold)
        globalGoalMarkerId.data = getClosestMarkerToXYPosition(FinalGoalX.data,FinalGoalY.data);
    ROS_INFO("Closest Marker to the goal -> %d",globalGoalMarkerId.data);
    return globalGoalMarkerId.data;
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
                seekingMarkerState = 0;
                break;
            default:
                seekingMarkerState = 0;
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


void HighLevelCommand::sendMarkerGoal(float distanceToMarker)
{
    goalAborted.data=false;
    markerSeen.data =-1;
    goalReached.data = false;
    transformLocationFromOdomToMap();
        
    NodeProperty marker = nextNode(closestMarkerId.data, globalGoalMarkerId.data, "graph.xml");
    currentGoal.header.seq = 1;
    currentGoal.header.stamp = ros::Time::now();
    currentGoal.header.frame_id = "map";
    currentGoal.pose.position.x = marker.x + distanceToMarker*cos(marker.orientation);    
    currentGoal.pose.position.y = marker.y + distanceToMarker*sin(marker.orientation);    
    currentGoal.pose.position.z = currentLocation.pose.pose.position.z;
    currentGoal.pose.orientation.x = 0;
    currentGoal.pose.orientation.y = 0;
    currentGoal.pose.orientation.z = 0;
    currentGoal.pose.orientation.w = 1;
    ROS_INFO("Next goal (%lf,%lf)",currentGoal.pose.position.x,currentGoal.pose.position.y);
    pubGoal.publish(currentGoal);

}

void HighLevelCommand::sendFinalGoal()
{
    goalAborted.data=false;
    markerSeen.data =-1;
    goalReached.data = false;
    transformLocationFromOdomToMap();
    
    currentGoal.header.seq = 1;
    currentGoal.header.stamp = ros::Time::now();
    currentGoal.header.frame_id = "map";
    currentGoal.pose.position.x = FinalGoalX.data;    
    currentGoal.pose.position.y = FinalGoalY.data;    
    currentGoal.pose.position.z = currentLocation.pose.pose.position.z;
    currentGoal.pose.orientation.x = 0;
    currentGoal.pose.orientation.y = 0;
    currentGoal.pose.orientation.z = 0;
    currentGoal.pose.orientation.w = 1;
    ROS_INFO("Next goal (%lf,%lf)",currentGoal.pose.position.x,currentGoal.pose.position.y);
    pubGoal.publish(currentGoal);
}

void HighLevelCommand::askForMarker()
{
    askMarker.data=true;
    pubAskForMarker.publish(empty);
}

