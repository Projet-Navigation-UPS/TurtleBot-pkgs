#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <iostream>

<<<<<<< HEAD
=======




>>>>>>> facd987d281947ef8265a5d2042d88dc2a5f655e
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

<<<<<<< HEAD
=======


    tf::Quaternion quad;    

    tf::Quaternion rot_y;
    rot_y.setEuler(0, 3.14, 0); 
    tf::Quaternion rot_x;
    rot_x.setRPY(1, 0, 0); 
    tf::Quaternion rot_z;
    rot_z.setRPY(0, 0, 1);
>>>>>>> facd987d281947ef8265a5d2042d88dc2a5f655e
    bool reset_odom = false;

    tf::Transform transform_mapMarker;
    transform_mapMarker.setOrigin(tf::Vector3(2.0, 1.0, 0.0));        
    transform_mapMarker.setRotation(tf::Quaternion(0.0, -0.707, 0.0, 0.707));//0.5, -0.5, -0.5, 0.5));//0.5, 0.5, -0.5, 0.5));
    
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

    ros::Rate loop_rate(5); 
    int i = 0 ;
    while(ros::ok())
	{

	    br.sendTransform(tf::StampedTransform(transform_empty, ros::Time::now(), "base_footprint", "base_link"));     

	    try
		{

<<<<<<< HEAD
		    if(li.waitForTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), ros::Duration(15)))
            {
                li.lookupTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), transform_imageMarker);
		       // br.sendTransform(tf::StampedTransform(transform_imageMarker, ros::Time::now(), "/camera_rgb_optical_frame", "/ar_marker_0"));

		        tf::Transform transform_cameraMarker;    
		        transform_cameraMarker = transform_cameraImage ;    
		        transform_cameraMarker *= transform_imageMarker ;   
		        //br.sendTransform(tf::StampedTransform(transform_cameraMarker, ros::Time::now(), "/camera_rgb_frame", "/ar_tr"));

		        tf::Transform transform_robotMarker;    
		        transform_robotMarker = transform_robotCamera;
		        transform_robotMarker *= transform_cameraMarker ;
		        transform_robotMarker.setOrigin(tf::Vector3(transform_robotMarker.getOrigin().getX(), transform_robotMarker.getOrigin().getY(), 0)); 
		        //br.sendTransform(tf::StampedTransform(transform_robotMarker, ros::Time::now(), "/base_link", "/ar_tr_tr"));

		        tf::Transform transform_markerRobot;
		        transform_markerRobot = transform_robotMarker.inverse(); 
		        //br.sendTransform(tf::StampedTransform(transform_markerRobot, ros::Time::now(), "/marker_0", "/res")); 

                pub_resetodom.publish(std_msgs::Empty());
		        tf::Transform transform_mapRobot;
		        transform_mapRobot = transform_mapMarker;
		        transform_mapRobot *= transform_markerRobot ;
		        br.sendTransform(tf::StampedTransform(transform_mapRobot, ros::Time::now(), "map", "odom"));
            }
            else 
            {
                br.sendTransform(tf::StampedTransform(transform_mapRobot, ros::Time::now(), "map", "odom"));
            }
=======
			li.lookupTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), transform_imageMarker);
            
            br.sendTransform(tf::StampedTransform(transform_imageMarker, ros::Time::now(), "/camera_rgb_optical_frame", "/ar_marker_0"));

            tf::Transform transform_cameraMarker;    
            transform_cameraMarker = transform_cameraImage ;    
            transform_cameraMarker *= transform_imageMarker ;   
	        br.sendTransform(tf::StampedTransform(transform_cameraMarker, ros::Time::now(), "/camera_rgb_frame", "/ar_tr"));

            tf::Transform transform_robotMarker;    
            transform_robotMarker = transform_robotCamera;
            transform_robotMarker *= transform_cameraMarker ;
            transform_robotMarker.setOrigin(tf::Vector3(transform_robotMarker.getOrigin().getX(), transform_robotMarker.getOrigin().getY(), 0)); 
            br.sendTransform(tf::StampedTransform(transform_robotMarker, ros::Time::now(), "/base_link", "/ar_tr_tr"));

            tf::Transform transform_markerRobot;
            transform_markerRobot = transform_robotMarker.inverse();
            br.sendTransform(tf::StampedTransform(transform_markerRobot, ros::Time::now(), "/marker_0", "/res")); 


		    if(reset_odom)
			{ 
			    pub_resetodom.publish(std_msgs::Empty());
			    reset_odom = false;
			}

            tf::Transform transform_mapRobot;
            transform_mapRobot = transform_mapMarker;
            transform_mapRobot *= transform_markerRobot ;

		  br.sendTransform(tf::StampedTransform(transform_empty, ros::Time::now(), "map", "odom"));

>>>>>>> facd987d281947ef8265a5d2042d88dc2a5f655e
		}
	    catch (tf::TransformException ex)
		{
		    br.sendTransform(tf::StampedTransform(transform_empty, ros::Time::now(), "map", "odom"));
		    reset_odom = true;
		}


	    ros::spinOnce();
	    loop_rate.sleep();
	}
    return 0;
};
