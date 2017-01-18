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
    ros::Publisher pubPlanPath, pubFollowPath, pubGoal;

    //Messages
    std_msgs::Bool locationReady, commandFinished, pathFound, nearGoal;
    actionlib_msgs::GoalStatus followPath, planPath;
    geometry_msgs::PoseStamped currentGoal, currentLocation;
    
    void callbackLocationReady(const nav_msgs::Odometry& msg);
    
public:

    HighLevelCommand(ros::NodeHandle& node);
    ~HighLevelCommand();
    
    bool location_Ready();
    
    
    void publish();
    

};

#endif
