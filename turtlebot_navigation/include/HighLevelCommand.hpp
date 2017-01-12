#ifndef _HIGHLEVELCOMMAND_
#define _HIGHLEVELCOMMAND_

#include <ros/ros.h>
#include "std_msgs/Bool.h"


class HighLevelCommand 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subscriberTest;

    //Publishers
    ros::Publisher publisherTest;

    //Messages
    std_msgs::Bool test;
    
    void callbackTest(const std_msgs::Bool& msg);
    
public:

    HighLevelCommand(ros::NodeHandle& node);
    ~HighLevelCommand();
    

};

#endif
