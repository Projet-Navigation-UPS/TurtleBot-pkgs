#include "HighLevelCommand.hpp"


HighLevelCommand::HighLevelCommand(ros::NodeHandle& node):
    //Subsribers
    subscriberTest(node.subscribe("/nav/sub_test", 1, &HighLevelCommand::callbackTest,this)),
    //Publishers
    publisherTest(node.advertise<std_msgs::Bool>("/nav/pub_test", 1))
{
    test.data = true;
}

HighLevelCommand::~HighLevelCommand(){}

void HighLevelCommand::callbackTest(const std_msgs::Bool& msg)
{
    test = msg;
}
