#include "HighLevelCommand.hpp"


HighLevelCommand::HighLevelCommand(ros::NodeHandle& node):
    //Subsribers
    subPathFound(node.subscribe("/nav/PathFound", 1, &HighLevelCommand::callbackPathFound,this)),
    subCommandFinished(node.subscribe("/nav/CommandFinished", 1, &HighLevelCommand::callbackCommandFinished,this)),
    //Publishers
    pubPathAsked(node.advertise<std_msgs::Bool>("/nav/PathAsked", 1)),
    pubCommandAsked(node.advertise<std_msgs::Bool>("/nav/CommandAsked", 1))
{
    pathFound.data = false; 
    pathAsked.data = false;
    commandFinished.data = false; 
    commandAsked.data = false;
}

HighLevelCommand::~HighLevelCommand(){}

void HighLevelCommand::callbackPathFound(const std_msgs::Bool& msg)
{
    pathFound = msg;
}


void HighLevelCommand::callbackCommandFinished(const std_msgs::Bool& msg)
{
    commandFinished = msg;
}
