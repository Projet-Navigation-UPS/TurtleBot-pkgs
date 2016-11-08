#ifndef _GRAPHICSERVER_
#define _GRAPHICSERVER_

#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Image.h"

class GraphicServer
{

public:
    GraphicServer(const string& topic);
    void sendImageDisplay(sensor_msgs::Image imageMsg);

private:    
    ros::Publisher publisherImageDisplay;

};

#endif
