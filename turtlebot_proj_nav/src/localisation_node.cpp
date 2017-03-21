#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
static const int TIMEOUT_AR_DETEC = 15;
void readMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
    std::vector<ar_track_alvar_msgs::AlvarMarker> markers = msg.markers;
    if(markers.empty())
    else
	ID_MARKER_DETECTED = msg.markers[0].id; // handle only one marker at once

int main(int argc, char** argv){
    ros::init(argc, argv, "localisation_node");
    ros::NodeHandle node; 
 
    tf::TransformBroadcaster br;
	tf::TransformListener li;
    
    ros::Subscriber readMarker = node.subscribe<const ar_track_alvar_msgs::AlvarMarkers&> ("/ar_pose_marker", 10, readMarkerCallback);
    tf::StampedTransform transforms_mapMarker[NB_MARKER];
    	{
    	    std::stringstream sstf;
    	    sstf << "/marker_" << i;
    	    std::string stf;
    	    stf = sstf.str();
    	    li.waitForTransform("/map", stf, ros::Time(0), ros::Duration(5));
    	    li.lookupTransform ("/map", stf, ros::Time(0), transforms_mapMarker[i]);
    	}
	tf::Transform transform_empty;
    transform_empty.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_empty.setRotation(q);


    tf::Quaternion rot_y(1.0, 0.0, 0.0); 
    bool reset_odom = false;

    tf::Transform transform_markermap;    
	tf::StampedTransform transform_markercamera ;	;
    tf::Transform transform_mapodom = transform_empty;
	    	{
    
	    	    if(GLOBAL_SEARCH == 1)
	    		{
	    		    std::cout << " GLOBAL_SEARCH " << std::endl ;

	    		    if(ID_MARKER_DETECTED != -1) 
    while(ros::ok())
	    			{
	    			    GLOBAL_SEARCH = 0 ; // search once
	    			    std::cout << " MARKER DETECTED: " << ID_MARKER_DETECTED << std::endl ;
	    			    tf::Transform transform_cameraMarker;    
	    			    tf::Transform transform_robotMarker;  
	    			    tf::Transform transform_markerRobot;   
	    			    tf::Transform transform_mapRobot;    
	    			    // image to marker
	    			    std::stringstream sstf;
	    			    sstf << "/ar_marker_" << ID_MARKER_DETECTED;
	    			    std::string stf;
	    			    stf = sstf.str();
				    std::cout << " LOOKING FOR TF: " << stf << std::endl ;
	    			    li.waitForTransform("/camera_rgb_optical_frame", stf, ros::Time(0), ros::Duration(TIMEOUT_AR_DETEC));
	    			    li.lookupTransform("/camera_rgb_optical_frame", stf, ros::Time(0), transform_imageMarker);
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
	    			    transform_mapRobot = transforms_mapMarker[ID_MARKER_DETECTED];
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
	    			    id.data = ID_MARKER_DETECTED; 
	    			    idMarker_pub.publish(id);

	    			    if(DEBUG)
	    				{
	    				    br.sendTransform(tf::StampedTransform(transform_imageMarker, ros::Time::now(), "/camera_rgb_optical_frame", stf));
	    				    br.sendTransform(tf::StampedTransform(transform_cameraMarker, ros::Time::now(), "/camera_rgb_frame", "/ar_tr"));
	    				    br.sendTransform(tf::StampedTransform(transform_robotMarker, ros::Time::now(), "/base_link", "/ar_tr_tr"));
	    				    // TODO br.sendTransform(tf::StampedTransform(transform_markerRobot, ros::Time::now(), "/marker_0", "/res"));
	    				    br.sendTransform(tf::StampedTransform(transform_mapRobot, ros::Time::now(), "/map", "/odom"));
	    				}
	    			}
	    		    else
	    			{   
	    			    // marker not detected, end of the search
	    			    GLOBAL_SEARCH = 0;
	    			    id.data = ID_MARKER_DETECTED;
	    			    idMarker_pub.publish(id); 
	    			}
	    		}
		    transform_markermap.setOrigin(transform_markercamera.getOrigin() + tf::Vector3(2.0,1.0,0.0));
			transform_markermap.setRotation(transform_markercamera.getRotation());
		    br.sendTransform(tf::StampedTransform(transform_markermap, ros::Time::now(), "map", "odom"));
		    transform_mapodom = transform_markermap;
	    	}
        catch (tf::TransformException ex)
	    	{
	    	    std::cout << " TransformException " << std::endl;
		    reset_odom = true;
	    	}
	    
	    ros::spinOnce();
	    loop_rate.sleep();
	}
    return 0;
};
