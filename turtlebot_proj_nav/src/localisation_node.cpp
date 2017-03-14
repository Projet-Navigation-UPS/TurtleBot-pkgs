#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <iostream>





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



    tf::Quaternion quad;    

    tf::Quaternion rot_y;
    rot_y.setEuler(0, 3.14, 0); 
    tf::Quaternion rot_x;
    rot_x.setRPY(1, 0, 0); 
    tf::Quaternion rot_z;
    rot_z.setRPY(0, 0, 1);
    bool reset_odom = false;

    tf::Transform transform_mapMarker;
    transform_mapMarker.setOrigin(tf::Vector3(2.0, 1.0, 0.0));        
    transform_mapMarker.setRotation(tf::Quaternion(0.0, -0.707, 0.0, 0.707));//0.5, 0.5, -0.5, 0.5));

    tf::Transform transform_robotCamera;
    transform_robotCamera.setOrigin(tf::Vector3(-0.087, -0.0125, 0.287));        
    transform_robotCamera.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));


    tf::Transform transform_cameraImage;
    transform_cameraImage.setOrigin(tf::Vector3(0.0, 0.0, 0.0));        
    transform_cameraImage.setRotation(tf::Quaternion(-0.5, 0.5, -0.5, 0.5));

    tf::Transform transform_markerRobot = transform_empty;
    tf::StampedTransform transform_imageMarker;   
             
   // transform_imageMarker.setOrigin(tf::Vector3(0.0579880018591, -0.00732792333958, 0.407226392688)); 
//-0.660010266417, -0.050182851828, 1.21713546119));
//0.0329890632742,     -0.00733393898488, 0.613363666968));
// 0.0579880018591, -0.00732792333958, 0.407226392688));        
   // transform_imageMarker.setRotation(tf::Quaternion(0.999481774828, 0.0155962407668, -0.0206495090784, 0.0191451517316));
//0.691354140005, 0.673177654754, -0.178080932861, -0.192739408501));
//0.708020089844, -0.683205901406, -0.0938495097937, 0.152083918192));
//0.999481774828, 0.0155962407668, -0.0206495090784, 0.0191451517316));



    ros::Publisher pub_resetodom = node.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1000);   

    ros::Rate loop_rate(5); 
    int i = 0 ;
    while(ros::ok())
	{

        br.sendTransform(tf::StampedTransform(transform_empty, ros::Time::now(), "base_footprint", "base_link"));     

	    try
		{

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
