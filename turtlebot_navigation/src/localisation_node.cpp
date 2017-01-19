#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>

void qrPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, tf::Transform& transform_markercamera, tf::TransformBroadcaster& br)
{ 
    
    transform_markercamera.setOrigin(tf::Vector3(0.0, msg->pose.position.z, 0.0));
    tf::Quaternion q;
    q.setRPY(0,0,0);

    transform_markercamera.setRotation(q); 
     
}

void qrStatusCB(const std_msgs::Int8::ConstPtr& msg, bool* qr_tracked)
{ 
    static const int QR_TRACKED_CODE = 3;
    if(msg->data == QR_TRACKED_CODE)
        *qr_tracked = true; 
    else
        *qr_tracked = false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "localisation_node");
    ros::NodeHandle node; 
 
    tf::TransformBroadcaster br;
    tf::Transform transform_empty;
    transform_empty.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_empty.setRotation(q);
    
    tf::Transform transform_markercamera;
    transform_markercamera = transform_empty;
    ros::Subscriber sub1 = node.subscribe<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position", 100, boost::bind(qrPoseCB, _1, transform_markercamera, br));
    
    bool qr_tracked = false;
    bool reset_odom = false;
    ros::Subscriber sub2 = node.subscribe<std_msgs::Int8>("/visp_auto_tracker/status", 100, boost::bind(qrStatusCB, _1, &qr_tracked));

    tf::Transform transform_markermap;
    tf::Transform transform_mapodom = transform_empty;
    
    ros::Publisher pub_resetodom = node.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1000);   

    ros::Rate loop_rate(5);
    while(ros::ok())
	{
	    
	    br.sendTransform(tf::StampedTransform(transform_empty, ros::Time::now(), "base_footprint", "base_link"));     

	    if(qr_tracked) 
		{
		    if(reset_odom)
			{ 
			    pub_resetodom.publish(std_msgs::Empty());
			    reset_odom = false;
			}
		    transform_markermap = transform_markercamera;
		    transform_markermap.setOrigin (transform_markercamera.getOrigin() + tf::Vector3(2.0,1.0,0.0)); // emplacement marker1
		    br.sendTransform(tf::StampedTransform(transform_markermap, ros::Time::now(), "map", "odom"));
		    transform_mapodom = transform_markermap;
		}
	    else
		{
		    br.sendTransform(tf::StampedTransform(transform_mapodom, ros::Time::now(), "map", "odom"));
		    reset_odom = true;
		}
	    
	    ros::spinOnce();
	    loop_rate.sleep();
	}
    return 0;
};
