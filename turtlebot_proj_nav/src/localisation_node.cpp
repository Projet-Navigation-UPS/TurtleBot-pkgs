#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <iostream>

static const int DEBUG = 0;
static const int TIMEOUT_AR_DETEC = 5;

int GLOBAL_SEARCH;

void askForMarkerCallback(const std_msgs::Empty& empty) {
    GLOBAL_SEARCH = 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "localisation_node");
    ros::NodeHandle node; 
 
    //  Broadcaster and Listener
    tf::TransformBroadcaster br;
    tf::TransformListener li;

    //  Subscribers and Publishers
    ros::Subscriber askForMarker = node.subscribe("/nav/HLC/askForMarker", 10, askForMarkerCallback);
    ros::Publisher idMarker_pub = node.advertise<std_msgs::Int16>("/nav/loca/markerSeen", 50); 
    ros::Publisher odom_pub = node.advertise<geometry_msgs::Transform>("/new_odom", 50);
    geometry_msgs::Transform odom_msg;

    // Statics TFs
    tf::StampedTransform transform_mapMarker;
    li.waitForTransform("/map", "/marker_0", ros::Time(0), ros::Duration(10));
    li.lookupTransform ("/map", "/marker_0", ros::Time(0), transform_mapMarker);
    /* transform_mapMarker.setOrigin(tf::Vector3(2.0, 1.0, 0.0));        
       transform_mapMarker.setRotation(tf::Quaternion(0.0, -0.707, 0.0, 0.707)); */

    tf::Transform transform_robotCamera;
        transform_robotCamera.setOrigin(tf::Vector3(-0.087, -0.0125, 0.287));        
        transform_robotCamera.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    tf::Transform transform_cameraImage;
            transform_cameraImage.setOrigin(tf::Vector3(0.0, 0.0, 0.0));        
            transform_cameraImage.setRotation(tf::Quaternion(-0.5, 0.5, -0.5, 0.5));

    // Empty TF
    tf::Transform transform_empty;
        transform_empty.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_empty.setRotation(q);

    // Dynamic TFs
    tf::Transform transform_markerRobot = transform_empty;
    tf::Transform transform_mapRobot = transform_empty;  
    tf::StampedTransform transform_imageMarker; 


    GLOBAL_SEARCH = 0;
    std_msgs::Int16 id ;
    
    while(ros::ok()) 
    {
        try
	    {

		if(GLOBAL_SEARCH == 1)
		{
		        std::cout << " GLOBAL_SEARCH " << std::endl ;
			if(li.waitForTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), ros::Duration(TIMEOUT_AR_DETEC))) 
			{
				GLOBAL_SEARCH = 0 ; // search once

                tf::Transform transform_cameraMarker;    
             	tf::Transform transform_robotMarker;  
	            tf::Transform transform_markerRobot;   
	            tf::Transform transform_mapRobot;    

                // image to marker
				li.lookupTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), transform_imageMarker);

                // camera to marker
				transform_cameraMarker = transform_cameraImage ;    
				transform_cameraMarker *= transform_imageMarker ;   

                // robot to marker
				transform_robotMarker = transform_robotCamera;
				transform_robotMarker *= transform_cameraMarker ;
				transform_robotMarker.setOrigin(tf::Vector3(transform_robotMarker.getOrigin().getX(), transform_robotMarker.getOrigin().getY(), 0)); 

                // marker to robot
				transform_markerRobot = transform_robotMarker.inverse(); 

                // map to robot
				transform_mapRobot = transform_mapMarker;
				transform_mapRobot *= transform_markerRobot ;
                
                // message creation
				odom_msg.translation.x = transform_mapRobot.getOrigin().getX();           
				odom_msg.translation.y = transform_mapRobot.getOrigin().getY();            
				odom_msg.translation.z = transform_mapRobot.getOrigin().getZ();
				odom_msg.rotation.x = transform_mapRobot.getRotation().getX();            
				odom_msg.rotation.y = transform_mapRobot.getRotation().getY();
				odom_msg.rotation.z = transform_mapRobot.getRotation().getZ();
				odom_msg.rotation.w = transform_mapRobot.getRotation().getW();
				odom_pub.publish(odom_msg);

                // id publication
                // TODO generalisation 
				id.data = 0; 
				idMarker_pub.publish(id);
  
				    if(DEBUG)
				    {
					    br.sendTransform(tf::StampedTransform(transform_imageMarker, ros::Time::now(), "/camera_rgb_optical_frame", "/ar_marker_0"));
					    br.sendTransform(tf::StampedTransform(transform_cameraMarker, ros::Time::now(), "/camera_rgb_frame", "/ar_tr"));
					    br.sendTransform(tf::StampedTransform(transform_robotMarker, ros::Time::now(), "/base_link", "/ar_tr_tr"));
					    br.sendTransform(tf::StampedTransform(transform_markerRobot, ros::Time::now(), "/marker_0", "/res"));
					    br.sendTransform(tf::StampedTransform(transform_mapRobot, ros::Time::now(), "/map", "/odom"));
				    }
			    }
			    else
			    {   
                    // marker not detected, end of the search
				    GLOBAL_SEARCH = 0;
				    id.data = -1;
				    idMarker_pub.publish(id); 
			    }
		    }
	    }
        catch (tf::TransformException ex)
	    {
		std::cout << " TransformException " << std::endl;
	    }
        ros::spinOnce(); 
    }
    return 0;
};
