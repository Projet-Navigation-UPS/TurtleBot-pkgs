/*
  localisation_broadcaster.cpp
  Marine Bouchet & Tristan Klempka

  ROS Node which broadcast constantly the robot position.
  Initial pos can be set by running :
  "rostopic pub -r 10 /new_odom geometry_msgs/Transform '{translation:  {x: 0.1, y: 0.0, z: 0.0}, rotation: {x: 0.0,y: 0.0,z: 0.0, w: 1.0}}'"
  
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <iostream>

static const int NODE_FREQ = 60; // Must be > ~50 Hz

tf::Transform transform_mapOdom;
ros::Publisher pub_resetodom;

/* odomCallback : odom modification when a marker is seen */
void odomCallback(const geometry_msgs::Transform& new_odom) {
    transform_mapOdom.setOrigin(tf::Vector3(new_odom.translation.x, new_odom.translation.y, new_odom.translation.z));
    transform_mapOdom.setRotation(tf::Quaternion(new_odom.rotation.x, new_odom.rotation.y, new_odom.rotation.z, new_odom.rotation.w));
    pub_resetodom.publish(std_msgs::Empty());
}

int main(int argc, char** argv){
    // ROS node init
    ros::init(argc, argv, "localisation_broadcaster_node");
    ros::NodeHandle node; 
 
    // Transform broadcaster and listener
    tf::TransformBroadcaster br;
    tf::TransformListener li;

    //  Subscribers and Publishers
    pub_resetodom = node.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1000);   
    ros::Subscriber sub = node.subscribe("new_odom", 1000, odomCallback);
    
    // Empty TF
    tf::Transform transform_empty;
    transform_empty.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_empty.setRotation(q);

    // Localisation TF
    transform_mapOdom = transform_empty;

    ros::Rate loop_rate(NODE_FREQ);
    // Classic ROS loop
    while(ros::ok())
    {
        br.sendTransform(tf::StampedTransform(transform_mapOdom, ros::Time::now(), "map", "odom"));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};
