/*
  GraphicServer.cpp
  Bruno Dato and Tristan Klempka

  Class which provides the service to publish a ROS image on a chosen topic
 
 */
#include "GraphicServer.hpp"

// Constructor of the "server" with a defined topic
GraphicServer::GraphicServer(ros::NodeHandle node,const std::string& topic) :

    publisherImageDisplay(node.advertise<sensor_msgs::Image>(topic, 1))
{}

// Publication of the image on the topic
void GraphicServer::sendImageDisplay(sensor_msgs::Image imageMsg)
{
    publisherImageDisplay.publish(imageMsg);
}

