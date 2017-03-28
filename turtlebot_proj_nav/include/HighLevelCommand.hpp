/*
  HightLevelCommand.cpp
  Bruno Dato

  Header file for HightLevelCommand class.
 
 */
#ifndef _HIGHLEVELCOMMAND_
#define _HIGHLEVELCOMMAND_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "kobuki_msgs/Sound.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <string>
#include "graph.hpp"
#include "turtlebot_proj_nav/command.h"
#include "turtlebot_proj_nav/MarkersVisibility.h"
#include <unistd.h>

#define SOUND_ON 0
#define SOUND_OFF 1
#define SOUND_RECHARGE 2
#define SOUND_BUTTON 3
#define SOUND_ERROR 4
#define SOUND_CLEANINGSTART 5
#define SOUND_CLEANINGEND 6

#define PI 3.1416

class HighLevelCommand 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subLocation, subMoveBaseActionResult, subCommandBusy, subMarkerSeen;

    //Publishers
    ros::Publisher pubGoal, pubSound, pubCommand, pubAskForMarker;
    
    //Services
    ros::ServiceClient clientMarkersVisibility;
    
    //TF
    tf::TransformListener tfListener; 

    //Messages
    std_msgs::Bool locationAvailable, goalReached, commandBusy, responseMarker, askMarker, goalAborted;
    std_msgs::Int16 closestMarkerId, globalGoalMarkerId, markerSeen;
    std_msgs::Empty empty;
    std_msgs::Float64 FinalGoalX, FinalGoalY;
    nav_msgs::Odometry currentLocation;
    move_base_msgs::MoveBaseActionResult  moveBaseActionResult;
    kobuki_msgs::Sound mobileBaseCommandsSound;
    geometry_msgs::PoseStamped currentGoal;
    
    // FSM State
    int seekingMarkerState;
    
    // Callbacks
    void callbackMarkerSeen(const std_msgs::Int16& msg);
    void callbackCommandBusy(const std_msgs::Bool& msg);
    void callbackLocation(const nav_msgs::Odometry& msg);
    void callbackMoveBaseActionResult(const move_base_msgs::MoveBaseActionResult& msg);
    
    // Commands
    void playSound(int sound);
    void sendDistanceAndAngleCommand(const float linearVelocity, const float angularVelocity, const float distance, const float angle);
    
    // Other useful functions
    void transformLocationFromOdomToMap();
    float distance(float x1, float y1, float x2, float y2);
    int getClosestMarkerToXYPosition(float x, float y);
  
public:

    // Constructor and destructor
    HighLevelCommand(ros::NodeHandle& node, float x_finalGoal, float y_finalGoal);
    ~HighLevelCommand();
    
    // States
    int markerID();
    int markersVisibility();
    bool location();
    bool getMarkerAsked();
    bool getMarkerResponse();
    bool intermediateGoal();
    bool finalMarkerGoal();
    bool finalGoal(float threshold);
    bool getGoalAborted();
    bool getCommandState();
    
    // High level commands
    int init(float threshold);
    void sendMarkerGoal(float distanceToMarker);
    void sendFinalGoal();
    int seekMarker();
    void askForMarker();
    
};

#endif
