/*
  HightLevelCommand.cpp
  Bruno Dato

  Class which provides the high level commands for navigation and communication with all the navigation services.
 
 */
#include "HighLevelCommand.hpp"
#include <iostream>
#include <vector>
#include <cmath> 

// Constructor
HighLevelCommand::HighLevelCommand(ros::NodeHandle& node, float x_finalGoal, float y_finalGoal):
    //Services
    clientMarkersVisibility(node.serviceClient<turtlebot_proj_nav::MarkersVisibility>("/nav/visibility_marker")),

    //Subsribers
    subCommandBusy(node.subscribe("/nav/command_busy", 1, &HighLevelCommand::callbackCommandBusy,this)),
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
    // Initialization of all variables
    // Initialization of the seeking markers FSM
    seekingMarkerState = 0;
    // All markers variables are set to default values
    markerSeen.data = -1;
    closestMarkerId.data = -1;
    globalGoalMarkerId.data = -1;
    // We will wait for the command to update busyness
    commandBusy.data = true;
    // We will wait for the topic /odom to be published
    locationAvailable.data = false;
    // Initialization to false for other boolean variables
    goalReached.data = false;
    responseMarker.data=false;
    askMarker.data=false;
    goalAborted.data=false;
    // The final goal is define by the user
    FinalGoalX.data= x_finalGoal;
    FinalGoalY.data= y_finalGoal;
}

// Destructor
HighLevelCommand::~HighLevelCommand(){}

//Callbacks
// Updates command busyness
void HighLevelCommand::callbackCommandBusy(const std_msgs::Bool& msg)
{
    //std::cout<<msg<<std::endl;
    commandBusy = msg;
}

// Updates the ID of the marker seen, -1 if no marker seen
void HighLevelCommand::callbackMarkerSeen(const std_msgs::Int16& msg)
{
    //std::cout<<msg<<std::endl;
    responseMarker.data = true;
    askMarker.data = false;
    markerSeen = msg;
}

// Updates position of the robot 
void HighLevelCommand::callbackLocation(const nav_msgs::Odometry& msg)
{
    //std::cout<<msg<<std::endl;
    currentLocation = msg; 
    locationAvailable.data = true;
}

// Gives information on a goal result and plays sound depending on the cases
void HighLevelCommand::callbackMoveBaseActionResult(const move_base_msgs::MoveBaseActionResult& msg)
{
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
    
    // Initialise seekinf marker FSM each time a goal gives a result
    seekingMarkerState = 0;
}


//States
// Returns the detected marker's ID 
int HighLevelCommand::markerID()
{
    //std::cout<<markerSeen<<std::endl;
    if(markerSeen.data != -1) // A marker is seen
    {
        playSound(SOUND_OFF);
        // Initialise seekinf marker FSM each time a marker is seen
        seekingMarkerState=0;
        // Memorise the marker
        closestMarkerId = markerSeen;
    }
    responseMarker.data=false;
    return markerSeen.data;   
}

// Returns the quantity of visible markers
int HighLevelCommand::markersVisibility()
{
    turtlebot_proj_nav::MarkersVisibility srv;
    // Transform the current location in the map
    transformLocationFromOdomToMap();
    // Get position in the map
    srv.request.x = currentLocation.pose.pose.position.x;
    srv.request.y = currentLocation.pose.pose.position.y;
    // Call markers visibility service
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

// Returns location availability
bool HighLevelCommand::location()
{
    return locationAvailable.data;
}

// Returns the command busyness
bool HighLevelCommand::getCommandState()
{
    return commandBusy.data;
}

// Returns true if markers detection has given a reponse
bool HighLevelCommand::getMarkerResponse()
{
    return responseMarker.data;
}

// Returns true if the goal has been aborted
bool HighLevelCommand::getGoalAborted()
{
    return goalAborted.data;
}

// Returns true if a markers detection has been asked
bool HighLevelCommand::getMarkerAsked()
{
    return askMarker.data;
}

// Returns true and plays a sound if the final goal is reached wich a precision under a threshold
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

// Return true is true if the final marker goal is reached
bool HighLevelCommand::finalMarkerGoal()
{ 
    if(closestMarkerId.data == globalGoalMarkerId.data) 
    {
        // Set to default value
        globalGoalMarkerId.data = -1;
        return true;
    }
    else return false;
}

// Returns true if a move_base goal is reached
bool HighLevelCommand::intermediateGoal()
{ 
    return goalReached.data; 
}


//Other useful functions
// Return the distance between 2 XY points
float HighLevelCommand::distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// Transform the location from odom reference to map reference
void HighLevelCommand::transformLocationFromOdomToMap()
{
    tf::StampedTransform transformMapOdom, transformMapRobot, transformOdomRobot;
    
    // Get transformation from map to odom
    tfListener.lookupTransform("/map", "/odom", ros::Time(0), transformMapOdom);
	transformMapRobot = transformMapOdom;
	
	// Get transformation from odom to robot
    transformOdomRobot.setOrigin(tf::Vector3(currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y, currentLocation.pose.pose.position.y));
    transformOdomRobot.setRotation(tf::Quaternion( currentLocation.pose.pose.orientation.x, currentLocation.pose.pose.orientation.y, currentLocation.pose.pose.orientation.z, currentLocation.pose.pose.orientation.w));
    
    // Complete transformation -> transformMapRobot = transformMapOdom * transformOdomRobot
    transformMapRobot *= transformOdomRobot; 
    
    // get pose in the map corresponding to the transformation
    tf::poseTFToMsg(transformMapRobot, currentLocation.pose.pose);
    currentLocation.header.frame_id = "map"; 
}

// Returns the closest marker to an (x,y) position
int HighLevelCommand::getClosestMarkerToXYPosition(float x, float y)
{
    // Arbitrary criteria, big regarding to map dimensions
    float dist = 100.0;
    // Initialise with default value
    int closestGoalMarkerId = -1;
    // Parse the graph describing the markers informations
    Graph g = xmlToGraph("graph.xml");
    // Browse the graph to find the minimal distance between the position (x,y) and the markers positions
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
// Plays one of the avaibles sounds on the turtlebot
// SOUND_ON SOUND_OFF SOUND_RECHARGE SOUND_BUTTON SOUND_ERROR SOUND_CLEANINGSTART SOUND_CLEANINGEND 
void HighLevelCommand::playSound(int sound)
{
    mobileBaseCommandsSound.value = sound;
    pubSound.publish(mobileBaseCommandsSound);
}

// Sends a distance and angle command to command_node
void HighLevelCommand::sendDistanceAndAngleCommand(const float linearVelocity, const float angularVelocity, const float distance, const float angle)
{
    if (!commandBusy.data) // if the command_node is not already active
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


//High level commands
// Returns the global/final marker goal (ID) if the distance between the XY goal and the current position is less than a threshold
// Return -1 if there is no such marker
int HighLevelCommand::init(float threshold)
{
    // Transform the current location into the map
    transformLocationFromOdomToMap();
    if(distance(currentLocation.pose.pose.position.x, currentLocation.pose.pose.position.y, FinalGoalX.data, FinalGoalY.data) > threshold)
        globalGoalMarkerId.data = getClosestMarkerToXYPosition(FinalGoalX.data,FinalGoalY.data);
        
    ROS_INFO("Closest Marker to the goal -> %d",globalGoalMarkerId.data);
    return globalGoalMarkerId.data;
}

//FSM to seek a marker
// Returns the current state
int HighLevelCommand::seekMarker()
{
    if (!commandBusy.data) // If the command is not busy
     {
     switch (seekingMarkerState) // Rotational behaviors
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
                seekingMarkerState = 0; //Infinite loop if no marker seen
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
        return -1; //If enable to command a rotation
    }
    
}

// Sends a goal to move_base in front of a marker to a distance defined by distanceToMarker
void HighLevelCommand::sendMarkerGoal(float distanceToMarker)
{
    //Reinitialise variables
    goalAborted.data = false;
    goalReached.data = false;
    markerSeen.data = -1;
    // Transform the current location into the map
    transformLocationFromOdomToMap();
    // Get next marker goal to follow the shortest path to the final goal    
    NodeProperty marker = nextNode(closestMarkerId.data, globalGoalMarkerId.data, "graph.xml");
    // Definition of the goal message
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
    // Publish the goal
    pubGoal.publish(currentGoal);

}

// Sends the final goal 
void HighLevelCommand::sendFinalGoal()
{
    //Reinitialise variables
    goalAborted.data = false;
    goalReached.data = false;
    markerSeen.data = -1;
    // Transform the current location into the map
    transformLocationFromOdomToMap();
    // Definition of the final goal message
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
    // Publish the goal
    pubGoal.publish(currentGoal);
}

// Asks to the localisation_node for a global search for a marker
void HighLevelCommand::askForMarker()
{
    askMarker.data=true;
    pubAskForMarker.publish(empty);
}

