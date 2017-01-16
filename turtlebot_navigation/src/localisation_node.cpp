#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "localisation_node");
    tf::TransformBroadcaster br;
    
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    ros::NodeHandle node;
    ros::Rate loop_rate(5);
    while(ros::ok())
    {    
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));       
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};
