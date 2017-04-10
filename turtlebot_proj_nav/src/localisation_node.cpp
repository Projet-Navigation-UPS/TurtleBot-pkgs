/*
  localisation_node.cpp
  Marine Bouchet & Tristan Klempka

  ROS Node which update the robot position according to its observation.
 
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <sstream>
#include <string>

// (De)Activate Debug mode
static const bool DEBUG = false;
// Timeout (s) for tf detection 
static const int TIMEOUT_AR_DETEC = 15;
// Number of markers in the scene
static const int NB_MARKER = 2;

// Global vars
// (De)Activate marker search 
bool GLOBAL_SEARCH;
// ID marker detected 
int ID_MARKER_DETECTED;

// Callback used by High Level Command to ask for marker detection
void askForMarkerCallback(const std_msgs::Empty& empty) {
    GLOBAL_SEARCH = true;
}

// Callback with AR_track_alvar node
// Read msg from alvar to know which marker is being detected 
void readMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    std::vector<ar_track_alvar_msgs::AlvarMarker> markers = msg.markers;
    if(markers.empty())
	ID_MARKER_DETECTED = -1;
    else
	ID_MARKER_DETECTED = msg.markers[0].id; // handle only one marker at once
}

int main(int argc, char** argv){
    // ROS node init
    ros::init(argc, argv, "localisation_node");
    ros::NodeHandle node; 
 
    // Transforms broadcaster and listener
    tf::TransformBroadcaster br;
    tf::TransformListener li;

    // Subscribers and publishers
    ros::Subscriber askForMarker = node.subscribe("/nav/HLC/askForMarker", 10, askForMarkerCallback);
    ros::Subscriber readMarker = node.subscribe<const ar_track_alvar_msgs::AlvarMarkers&> ("/ar_pose_marker", 10, readMarkerCallback);
    ros::Publisher idMarker_pub = node.advertise<std_msgs::Int16>("/nav/loca/markerSeen", 50); 
    ros::Publisher odom_pub = node.advertise<geometry_msgs::Transform>("/new_odom", 50);
    
    // Markers static transform init
    tf::StampedTransform transforms_mapMarker[NB_MARKER];
    for(int i = 0; i < NB_MARKER; i++)
    	{
    	    std::stringstream sstf;
    	    sstf << "/marker_" << i;
    	    std::string stf;
    	    stf = sstf.str();
    	    li.waitForTransform("/map", stf, ros::Time(0), ros::Duration(5));
    	    li.lookupTransform ("/map", stf, ros::Time(0), transforms_mapMarker[i]);
    	}
    
    // Robot to camera transform
    tf::Transform transform_robotCamera;
    transform_robotCamera.setOrigin(tf::Vector3(-0.087, -0.0125, 0.287));        
    transform_robotCamera.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    
    // Camera to image transform
    tf::Transform transform_cameraImage;
    transform_cameraImage.setOrigin(tf::Vector3(0.0, 0.0, 0.0));        
    transform_cameraImage.setRotation(tf::Quaternion(-0.5, 0.5, -0.5, 0.5));

    // Identity transform init
    tf::Transform transform_identity;
    transform_identity.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_identity.setRotation(q);    
    
    // Global vars init
    GLOBAL_SEARCH = false;
    ID_MARKER_DETECTED = -1;
    
    // Classic ROS loop
    while(ros::ok()) 
	{
	    try
	    	{
		    // Search has been asked by High Level Command
	    	    if(GLOBAL_SEARCH)
	    		{
	    		    std::cout << " GLOBAL_SEARCH " << std::endl ;
			    // Marker detected
			    // Publish /new_odom transform to update the robot position
			    // This transform is built through several transforms
			    if(ID_MARKER_DETECTED != -1)  
	    			{
				    GLOBAL_SEARCH = false ; // search once
	    			    std::cout << " MARKER DETECTED: " << ID_MARKER_DETECTED << std::endl ;
				    
	    			    // Find the right transform according to the marker detected by Alvar 
				    // Image to marker transform
	    			    std::stringstream sstf;
	    			    sstf << "/ar_marker_" << ID_MARKER_DETECTED;
	    			    std::string stf;
	    			    stf = sstf.str();
				    std::cout << " LOOKING FOR TF: " << stf << std::endl ;
				    tf::StampedTransform transform_imageMarker; 
	    			    li.waitForTransform("/camera_rgb_optical_frame", stf, ros::Time(0), ros::Duration(TIMEOUT_AR_DETEC));
	    			    li.lookupTransform("/camera_rgb_optical_frame", stf, ros::Time(0), transform_imageMarker);

	    			    // Camera to marker transform
	    			    tf::Transform transform_cameraMarker;    
				    transform_cameraMarker = transform_cameraImage ;    
	    			    transform_cameraMarker *= transform_imageMarker ;   

	    			    // Robot to marker transform
				    tf::Transform transform_robotMarker;
	    			    transform_robotMarker = transform_robotCamera;
	    			    transform_robotMarker *= transform_cameraMarker ;
	    			    transform_robotMarker.setOrigin(tf::Vector3(transform_robotMarker.getOrigin().getX(), transform_robotMarker.getOrigin().getY(), 0)); 

	    			    // Marker to robot transform
				    tf::Transform transform_markerRobot;
	    			    transform_markerRobot = transform_robotMarker.inverse(); 

	    			    // Map to robot transform
				    tf::Transform transform_mapRobot;
	    			    transform_mapRobot = transforms_mapMarker[ID_MARKER_DETECTED];
	    			    transform_mapRobot *= transform_markerRobot ;
                
	    			    // New robot position publication
				    geometry_msgs::Transform odom_msg;
	    			    odom_msg.translation.x = transform_mapRobot.getOrigin().getX();           
	    			    odom_msg.translation.y = transform_mapRobot.getOrigin().getY();            
	    			    odom_msg.translation.z = transform_mapRobot.getOrigin().getZ();
	    			    odom_msg.rotation.x = transform_mapRobot.getRotation().getX();            
	    			    odom_msg.rotation.y = transform_mapRobot.getRotation().getY();
	    			    odom_msg.rotation.z = transform_mapRobot.getRotation().getZ();
	    			    odom_msg.rotation.w = transform_mapRobot.getRotation().getW();
	    			    odom_pub.publish(odom_msg);

	    			    // Marker id publication
	    			    std_msgs::Int16 id ;
				    id.data = ID_MARKER_DETECTED; 
	    			    idMarker_pub.publish(id);
				    
				    // Activate tf in rviz to help debuging
	    			    if(DEBUG)
	    				{
	    				    br.sendTransform(tf::StampedTransform(transform_imageMarker, ros::Time::now(), "/camera_rgb_optical_frame", stf));
	    				    br.sendTransform(tf::StampedTransform(transform_cameraMarker, ros::Time::now(), "/camera_rgb_frame", "/ar_tr"));
	    				    br.sendTransform(tf::StampedTransform(transform_robotMarker, ros::Time::now(), "/base_link", "/ar_tr_tr"));
	    				    
	    				    br.sendTransform(tf::StampedTransform(transform_mapRobot, ros::Time::now(), "/map", "/odom"));
	    				}
	    			}
			    // Marker not detected
			    // End of the search
			    // Send not detected code to High Level Command topic
	    		    else 
	    			{   
	    			    GLOBAL_SEARCH = false;
	    			    std_msgs::Int16 id ;
				    id.data = ID_MARKER_DETECTED; // Here -1
	    			    idMarker_pub.publish(id); 
	    			}
	    		}
	    	}
	    // Handle tf exception
	    catch (tf::TransformException ex)
	    	{
	    	    std::cout << " TransformException " << std::endl;
	    	}
	    ros::spinOnce(); 
	}
    return 0;
};
