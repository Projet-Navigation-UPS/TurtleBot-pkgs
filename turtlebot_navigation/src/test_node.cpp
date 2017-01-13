#include "ros/ros.h"
#include "HighLevelCommand.hpp"
#include <stdlib.h>
#include <iostream>
#include <cmath> 

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
    ROS_INFO("Launching test_node ...");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(2); // 2Hz 


    ros::Publisher pubPath(node.advertise<nav_msgs::Path>("/nav/PathToFollow", 1));
    ros::Publisher pubLocation(node.advertise<nav_msgs::Odometry>("/nav/Location", 1));
    ros::Publisher pubCmdFinished(node.advertise<std_msgs::Bool>("/nav/CommandFinished", 1));

    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose1, pose2, pose3;
    nav_msgs::Odometry location;
    std_msgs::Bool commandFinished;
    
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 0;
    pose1.pose.orientation.x = 0;
    pose1.pose.orientation.y = 0;
    pose1.pose.orientation.z = 0;
    pose1.pose.orientation.w = 0;
    
    pose2.pose.position.x = 1;
    pose2.pose.position.y = 1;
    pose2.pose.position.z = 1;
    pose2.pose.orientation.x = 1;
    pose2.pose.orientation.y = 1;
    pose2.pose.orientation.z = 1;
    pose2.pose.orientation.w = 1;
    
    pose3.pose.position.x = 2;
    pose3.pose.position.y = 2;
    pose3.pose.position.z = 2;
    pose3.pose.orientation.x = 2;
    pose3.pose.orientation.y = 2;
    pose3.pose.orientation.z = 2;
    pose3.pose.orientation.w = 2;
    
    path.poses.push_back(pose1);
    path.poses.push_back(pose2);
    path.poses.push_back(pose3);
    
    location.pose.pose.position.x = 2.05;
    location.pose.pose.position.y = 2.05;
    location.pose.pose.position.z = 2.05;
    location.pose.pose.orientation.x = 2.05;
    location.pose.pose.orientation.y = 2.05;
    location.pose.pose.orientation.z = 2.05;
    location.pose.pose.orientation.w = 2.05;

    commandFinished.data = true;
    float time = 0;
    while (ros::ok()) 
    {
        ros::spinOnce();
        
        pubLocation.publish(location);
        //std::cout<<location<<std::endl;
        
        if(time>5)
        {
            pubPath.publish(path);
            //std::cout<<path<<std::endl;
        }
        
        if(time>10)
        {
            pubCmdFinished.publish(commandFinished);
        }
        
        
        //std::cout<<time<<std::endl;
        time = time + 0.5;
        loop_rate.sleep();
    }
    
    return 0;
}
