#ifndef _HIGHLEVELCOMMAND_
#define _HIGHLEVELCOMMAND_

#include <ros/ros.h>
#include "std_msgs/Bool.h"




class HighLevelCommand 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subPathFound, subCommandFinished ;

    //Publishers
    ros::Publisher pubPathAsked, pubCommandAsked;

    //Messages
    std_msgs::Bool pathFound, pathAsked;
    std_msgs::Bool commandFinished, commandAsked;
    
    void callbackPathFound(const std_msgs::Bool& msg);
    void callbackCommandFinished(const std_msgs::Bool& msg);
    
public:

    HighLevelCommand(ros::NodeHandle& node);
    ~HighLevelCommand();
    
    bool path_Found();
    bool command_Finished();
    
    void ask_Path();
    void ask_Command();
    
    void publish();
    

};

#endif
