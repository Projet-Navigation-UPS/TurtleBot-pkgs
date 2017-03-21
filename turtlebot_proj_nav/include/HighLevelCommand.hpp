#ifndef _HIGHLEVELCOMMAND_
#define _HIGHLEVELCOMMAND_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "kobuki_msgs/Sound.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <string>
#include "graph.hpp"
#include "turtlebot_proj_nav/command.h"
#include "turtlebot_proj_nav/MarkersVisibility.h"
#include "std_msgs/Empty.h"
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
    ros::Subscriber subLocation, subGoalStatus, subMoveBaseActionFeedback, subMoveBaseActionGoal, subMoveBaseActionResult, subscriberCommandBusy, subMarkerSeen, subMarkersVisibility;

    //Publishers
    ros::Publisher pubGoal, pubSound, pubCommand, pubCommandState, pubAskForMarker;
    
    //Services
    ros::ServiceClient clientMarkersVisibility;

    //Messages
    std_msgs::Bool locationAvailable, goalReached, commandBusy, responseMarker, askMarker, goalAborted;
    std_msgs::Int16 closestMarkerId, GlobalGoalMarkerId, markerSeen, makersVisibility;
    std_msgs::Empty empty;
    std_msgs::Float64 FinalGoalX, FinalGoalY;
    
    //States
    int seekingMarkerState;
    
    //TF
    tf::TransformListener tfListener;
    //
    

    actionlib_msgs::GoalStatusArray goalStatus;
    nav_msgs::Odometry currentLocation;
    move_base_msgs::MoveBaseActionResult  moveBaseActionResult;
    move_base_msgs::MoveBaseActionFeedback moveBaseActionFeedback;
    move_base_msgs::MoveBaseActionGoal moveBaseActionGoal;
    kobuki_msgs::Sound mobileBaseCommandsSound;
    
    geometry_msgs::PoseStamped currentGoal;
    
    void callbackMarkersVisibility(const std_msgs::Int16& msg);
    void callbackMarkerSeen(const std_msgs::Int16& msg);
    void callbackCommandBusy(const std_msgs::Bool& msg);
    void callbackLocation(const nav_msgs::Odometry& msg);
    void callbackGoalStatus(const actionlib_msgs::GoalStatusArray& msg);
    void callbackMoveBaseActionResult(const move_base_msgs::MoveBaseActionResult& msg);
    void callbackMoveBaseActionFeedback(const move_base_msgs::MoveBaseActionFeedback& msg);
    void callbackMoveBaseActionGoal(const move_base_msgs::MoveBaseActionGoal& msg);
  
public:

    HighLevelCommand(ros::NodeHandle& node);
    HighLevelCommand(ros::NodeHandle& node, float x_finalGoal, float y_finalGoal);
    ~HighLevelCommand();
    
    void init(float threshold);
    int marker();
    int seekMarker();
    int markersVisibility();
    int getClosestMarkerToXYPosition(float x, float y);
    
    bool location();
    bool finalGoal(float threshold);
    bool intermediateGoal();
    bool markerResponse();
    bool getAskMarker();
    bool finalMarkerGoal();
    bool getGoalAborted();
    bool getCommandBusy();
    
    float distance(float x1, float y1, float x2, float y2);
    
    void sendGoal();
    void findGlobalGoal();
    void playSound(int sound);
    void sendDistanceAndAngleCommand(const float linearVelocity, const float angularVelocity, const float distance, const float angle);
    void askForMarker();
    
    void transformLocationFromOdomToMap();
    
    //void disableSimpleCommand();
    //void enableSimpleCommand();
    

};

#endif
