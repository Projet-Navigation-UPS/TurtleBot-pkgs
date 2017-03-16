#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <iostream>

/* rostopic pub -r 10 /odom_pub geometry_msgs/Transform '{translation:  {x: 0.1, y: 0.0, z: 0.0}, rotation: {x: 0.0,y: 0.0,z: 0.0, w: 1.0}}' */

tf::Transform transform_mapOdom;
ros::Publisher pub_resetodom;

void odomCallback(const geometry_msgs::Transform& new_odom) {
    transform_mapOdom.setOrigin(tf::Vector3(new_odom.translation.x, new_odom.translation.y, new_odom.translation.z));
    transform_mapOdom.setRotation(tf::Quaternion(new_odom.rotation.x, new_odom.rotation.y, new_odom.rotation.z, new_odom.rotation.w));
    pub_resetodom.publish(std_msgs::Empty());
    std::cout << " maj " << std::endl ;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "localisation_broadcaster_node");
    ros::NodeHandle node; 
 
    tf::TransformBroadcaster br;
    tf::TransformListener li;
    
    tf::Transform transform_empty;
    transform_empty.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    transform_empty.setRotation(q);

    transform_mapOdom = transform_empty;
    pub_resetodom = node.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1000);   


    // MESSAGE
    ros::Subscriber sub = node.subscribe("new_odom", 1000, odomCallback);
    // 

    ros::Rate loop_rate(5); 
    while(ros::ok())
    {
        
        /* tf::StampedTransform transform_mapNewOdom;
        try
	    {
            if(li.canTransform("/map", "/new_odom", ros::Time::now()))
            {  
                std::cout << " maj " << std::endl ;
                pub_resetodom.publish(std_msgs::Empty());
                li.lookupTransform("/map", "/new_odom", ros::Time::now(), transform_mapNewOdom);
                transform_mapOdom = transform_mapNewOdom;
            }
            
            br.sendTransform(tf::StampedTransform(transform_mapOdom, ros::Time::now(), "map", "odom"));
	    }
        catch (tf::TransformException ex)
	    {
            std::cout << " TransformException " << std::endl;
	    }*/

        br.sendTransform(tf::StampedTransform(transform_mapOdom, ros::Time::now(), "map", "odom"));
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};
