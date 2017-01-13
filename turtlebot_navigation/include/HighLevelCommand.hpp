#ifndef _HIGHLEVELCOMMAND_
#define _HIGHLEVELCOMMAND_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "actionlib_msgs/GoalStatus.h"
#include "geometry_msgs/PoseStamped.h"




class HighLevelCommand 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subPathFound, subCommandFinished , subLocationReady;

    //Publishers
    ros::Publisher pubPlanPath, pubFollowPath;

    //Messages
    std_msgs::Bool locationReady, commandFinished, pathFound, nearGoal;
    actionlib_msgs::GoalStatus followPath, planPath;
    geometry_msgs::PoseStamped currentGoal, currentLocation;
    
    void callbackLocationReady(const nav_msgs::Odometry& msg);
    void callbackPathFound(const nav_msgs::Path& msg);
    void callbackCommandFinished(const std_msgs::Bool& msg);
    
public:

    HighLevelCommand(ros::NodeHandle& node);
    ~HighLevelCommand();
    
    bool location_Ready();
    bool path_Found();
    bool command_Finished();
    bool near_Goal();
    
    void plan_Path();
    void follow_Path();
    
    void publish();
    

};

#endif
