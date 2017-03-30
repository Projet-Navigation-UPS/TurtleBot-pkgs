#ifndef _TURTLEBOTCAMERA_
#define _TURTLEBOTCAMERA_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "sensor_msgs/Image.h"
#include "kobuki_msgs/Sound.h"

//Constantes
const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
const int CAMERA_CHANNELS = 3;
const int CAMERA_STEP_RGB = CAMERA_WIDTH * CAMERA_CHANNELS;
const int CAMERA_STEP_MONO = CAMERA_WIDTH;


class TurtleBotCamera
{
    
private:
    
    //Subscrbers
    ros::Subscriber subscriberCameraRgbImageColor;

    //Messages
    sensor_msgs::Image cameraRgbImageColor;
    unsigned char* cameraRgbImageColorRaw;
    std::vector<unsigned char> cameraRgbImageColorVec;
    
    // simple callback to receive image from camera 
    void callbackCameraRgbImageColor(const sensor_msgs::Image& msg);
    
public:

    TurtleBotCamera(ros::NodeHandle& node);
    ~TurtleBotCamera();
    
    //Getters
    sensor_msgs::Image getCameraRgbImageColor();
    unsigned char* getCameraRgbImageColorRaw();
 

    //Image conversion
    sensor_msgs::Image convertRawToSensorMsgsImage(const unsigned char* raw, const int height, const int width, const std::string&encoding, const char is_bigendian, const int step);

    //Display
    void displaySensorMsgsImage(const std::string& type, const sensor_msgs::Image& sensorMsgsImage);
    
};

#endif
