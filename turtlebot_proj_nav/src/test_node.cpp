#include "ros/ros.h"
#include <stdlib.h>
#include "std_msgs/Bool.h"

void callbackExample(const std_msgs::Bool& msg)
{
    ROS_INFO("Callback called, message : %d",msg.data);
}


int main(int argc, char **argv)
{
    ROS_INFO("Launching test_node ...");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(1); // 1Hz 

    
    ros::Publisher publisherExample(node.advertise<std_msgs::Bool>("/topic/example", 1));
    ros::Subscriber subscriberExample(node.subscribe("/topic/example", 1, &callbackExample));
    
    std_msgs::Bool msg;

    while (ros::ok()) 
    {
        msg.data = true;
        publisherExample.publish(msg);  
        ROS_INFO("Published message : %d",msg.data);  
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
