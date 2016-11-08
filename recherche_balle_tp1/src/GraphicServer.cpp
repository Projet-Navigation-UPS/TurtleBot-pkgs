#include "GraphicServer.hpp"

GraphicServer::GraphicServer(ros::NodeHandle node,const std::string& topic) :

    publisherImageDisplay(node.advertise<sensor_msgs::Image>(topic, 1))
{
}

void GraphicServer::sendImageDisplay(sensor_msgs::Image imageMsg)
{
    publisherImageDisplay.publish(imageMsg);
}

