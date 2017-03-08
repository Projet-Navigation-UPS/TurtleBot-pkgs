#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>

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

	tf::Transform transform_y;
    transform_empty.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 1);
    transform_empty.setRotation(q);

    bool reset_odom = false;

    tf::Transform transform_markermap;    
	tf::Transform transform_markercamera ;	
	tf::StampedTransform stamped_transform_markercamera ;
    tf::Transform transform_mapodom = transform_empty;
    
    ros::Publisher pub_resetodom = node.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1000);   

    ros::Rate loop_rate(5);
    while(ros::ok())
	{
	    br.sendTransform(tf::StampedTransform(transform_empty, ros::Time::now(), "base_footprint", "base_link"));     

	    try
		{
			li.lookupTransform("/camera_rgb_optical_frame", "/ar_marker_0", ros::Time(0), stamped_transform_markercamera);
			transform_markercamera = stamped_transform_markercamera.getData();
			transform_markercamera *= 
		    if(reset_odom)
			{ 
			    pub_resetodom.publish(std_msgs::Empty());
			    reset_odom = false;
			}
		    transform_markermap.setOrigin(transform_markercamera.getOrigin() + tf::Vector3(2.0,1.0,0.0));
			transform_markermap.setRotation(transform_markercamera.getRotation());
		    br.sendTransform(tf::StampedTransform(transform_markermap, ros::Time::now(), "map", "odom"));
		    transform_mapodom = transform_markermap;
		}
        catch (tf::TransformException ex)
		{
		    br.sendTransform(tf::StampedTransform(transform_mapodom, ros::Time::now(), "map", "odom"));
		    reset_odom = true;
		}
	    
	    ros::spinOnce();
	    loop_rate.sleep();
	}
    return 0;
};
