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
    if(pathFound.data) pathAsked.data = false;
}


void HighLevelCommand::callbackCommandFinished(const std_msgs::Bool& msg)
{
    commandFinished = msg;
    if(commandFinished.data) commandAsked.data = false;
}

bool HighLevelCommand::path_Found()
{
    return pathFound.data;
}
bool HighLevelCommand::command_Finished()
{
    return commandFinished.data;
}

void HighLevelCommand::ask_Path(){pathAsked.data = true;}
void HighLevelCommand::ask_Command(){commandAsked.data = true;}

void HighLevelCommand::publish()
{
    pubPathAsked.publish(pathAsked);
    pubCommandAsked.publish(commandAsked);
}
