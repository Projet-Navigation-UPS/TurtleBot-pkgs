#ifndef _HIGHLEVELCOMMAND_
#define _HIGHLEVELCOMMAND_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "kobuki_msgs/Sound.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <tf/transform_listener.h>

#define SOUND_ON 0
#define SOUND_OFF 1
#define SOUND_RECHARGE 2
#define SOUND_BUTTON 3
#define SOUND_ERROR 4
#define SOUND_CLEANINGSTART 5
#define SOUND_CLEANINGEND 6

#define X_GOAL1 2
#define Y_GOAL1 2
#define X_GOAL2 2
#define Y_GOAL2 4
#define X_GOAL3 4
#define Y_GOAL3 4

class HighLevelCommand 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subLocation, subGoalStatus, subMoveBaseActionFeedback, subMoveBaseActionGoal, subMoveBaseActionResult;
    
    //Publishers
    ros::Publisher pubGoal, pubSound;

    //Messages
    std_msgs::Bool locationAvailable, goalReached;
    
    //TF
    tf::TransformListener tfListener;
    tf::StampedTransform transform;
    
    actionlib_msgs::GoalStatusArray goalStatus;
    nav_msgs::Odometry currentLocation;
    move_base_msgs::MoveBaseActionResult  moveBaseActionResult;
    move_base_msgs::MoveBaseActionFeedback moveBaseActionFeedback;
    move_base_msgs::MoveBaseActionGoal moveBaseActionGoal;
    kobuki_msgs::Sound mobileBaseCommandsSound;
    
    geometry_msgs::PoseStamped currentGoal;
    
    void callbackLocation(const nav_msgs::Odometry& msg);
    void callbackGoalStatus(const actionlib_msgs::GoalStatusArray& msg);
    void callbackMoveBaseActionResult(const move_base_msgs::MoveBaseActionResult& msg);
    void callbackMoveBaseActionFeedback(const move_base_msgs::MoveBaseActionFeedback& msg);
    void callbackMoveBaseActionGoal(const move_base_msgs::MoveBaseActionGoal& msg);
  
public:

    HighLevelCommand(ros::NodeHandle& node);
    ~HighLevelCommand();
    
    bool location();
    bool finalGoal();
    
    float distance(float x1, float y1, float x2, float y2);
    int nearestGoal(float x, float y);
    
    void sendGoal();
    void playSound(int sound);
    

};

#endif
