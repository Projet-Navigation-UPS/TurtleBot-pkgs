#ifndef _HIGHLEVELCOMMAND_
#define _HIGHLEVELCOMMAND_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <tf/transform_listener.h>


class HighLevelCommand 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subLocation, subGoalStatus, subMoveBaseActionFeedback, subMoveBaseActionGoal, subMoveBaseActionResult;
    
    //Publishers
    ros::Publisher pubGoal;

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
    
    
    void sendGoal();
    

};

#endif
