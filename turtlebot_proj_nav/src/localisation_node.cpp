#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <iostream>

static const int DEBUG = 0;
 
/* 1238B*/
int main(int argc, char** argv){
    ros::init(argc, argv, "localisation_node");
    ros::NodeHandle node; 
 
    tf::TransformBroadcaster br;
    tf::TransformListener li;
    
    tf::Transform transform_empty;
    transform_empty.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_empty.setRotation(q);

    tf::Transform transform_mapMarker;
    transform_mapMarker.setOrigin(tf::Vector3(2.0, 1.0, 0.0));        
    transform_mapMarker.setRotation(tf::Quaternion(0.0, -0.707, 0.0, 0.707));
    
    tf::Transform transform_robotCamera;
    transform_robotCamera.setOrigin(tf::Vector3(-0.087, -0.0125, 0.287));        
    transform_robotCamera.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    tf::Transform transform_cameraImage;
    transform_cameraImage.setOrigin(tf::Vector3(0.0, 0.0, 0.0));        
    transform_cameraImage.setRotation(tf::Quaternion(-0.5, 0.5, -0.5, 0.5));

    tf::Transform transform_markerRobot = transform_empty;
    tf::StampedTransform transform_imageMarker;
    tf::Transform transform_mapRobot = transform_empty;   
             
    ros::Publisher pub_resetodom = node.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1000);   

    // MESSAGE
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    geometry_msgs::Transform odom_msg;
    ros::Publisher odom_pub = node.advertise<geometry_msgs::Transform>("/new_odom", 50);
    // FIN MESSAGE

    int find = 0 ;
    while(ros::ok()) {
        try
	    {
            std::cout << " find : " << find << std::endl ;

	        if(li.waitForTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), ros::Duration(120)) && find == 0) // wait for tf during 1 min
            {
                li.lookupTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), transform_imageMarker);

	            tf::Transform transform_cameraMarker;    
	            transform_cameraMarker = transform_cameraImage ;    
	            transform_cameraMarker *= transform_imageMarker ;   

	            tf::Transform transform_robotMarker;    
	            transform_robotMarker = transform_robotCamera;
	            transform_robotMarker *= transform_cameraMarker ;
	            transform_robotMarker.setOrigin(tf::Vector3(transform_robotMarker.getOrigin().getX(), transform_robotMarker.getOrigin().getY(), 0)); 

	            tf::Transform transform_markerRobot;
	            transform_markerRobot = transform_robotMarker.inverse(); 

	            tf::Transform transform_mapRobot;
	            transform_mapRobot = transform_mapMarker;
	            transform_mapRobot *= transform_markerRobot ;
                
                // br.sendTransform(tf::StampedTransform(transform_mapRobot, ros::Time::now(), "map", "new_odom"));// send TF  transform_mapRobot  map odom

                std::cout << " echo " <<  std::endl ;

                odom_msg.translation.x = transform_mapRobot.getOrigin().getX();           
                odom_msg.translation.y = transform_mapRobot.getOrigin().getY();            
                odom_msg.translation.z = transform_mapRobot.getOrigin().getZ();
                odom_msg.rotation.x = transform_mapRobot.getRotation().getX();            
                odom_msg.rotation.y = transform_mapRobot.getRotation().getY();
                odom_msg.rotation.z = transform_mapRobot.getRotation().getZ();
                odom_msg.rotation.w = transform_mapRobot.getRotation().getW();
                odom_pub.publish(odom_msg);


                
                if(DEBUG)
                {
                    br.sendTransform(tf::StampedTransform(transform_imageMarker, ros::Time::now(), "/camera_rgb_optical_frame", "/ar_marker_0"));
                    br.sendTransform(tf::StampedTransform(transform_cameraMarker, ros::Time::now(), "/camera_rgb_frame", "/ar_tr"));
                    br.sendTransform(tf::StampedTransform(transform_robotMarker, ros::Time::now(), "/base_link", "/ar_tr_tr"));
                    br.sendTransform(tf::StampedTransform(transform_markerRobot, ros::Time::now(), "/marker_0", "/res"));
                    br.sendTransform(tf::StampedTransform(transform_mapRobot, ros::Time::now(), "map", "odom")); 
                }
                find = 1 ;
                ros::spinOnce();    
                return 0;
            }

	    }
        catch (tf::TransformException ex)
	    {
            std::cout << " TransformException " << std::endl;
	    }
        

    }
    return 0;
};
