#ifndef _TURTLEBOTCAMERA_
#define _TURTLEBOTCAMERA_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "sensor_msgs/Image.h"
#include "kobuki_msgs/Sound.h"

//Constantes
const int SOUND_ON = 0;
const int SOUND_OFF = 1;
const int SOUND_RECHARGE = 2;
const int SOUND_BUTTON = 3;
const int SOUND_ERROR = 4;
const int SOUND_CLEANINGSTART = 5;
const int SOUND_CLEANINGEND = 6;


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

    //Publishers
    ros::Publisher publisherMobileBaseCommandsSound;

    //Messages
    sensor_msgs::Image cameraRgbImageColor;
    unsigned char* cameraRgbImageColorRaw;
    std::vector<unsigned char> cameraRgbImageColorVec;
    
    kobuki_msgs::Sound mobileBaseCommandsSound;
    
    ros::NodeHandle& m_node;
public:

    TurtleBotCamera(ros::NodeHandle& node);
    ~TurtleBotCamera();
    
    //Getters
    sensor_msgs::Image getCameraRgbImageColor();
    unsigned char* getCameraRgbImageColorRaw();
 
    //Setters    
    void setMobileBaseCommandsSound(const int sound);

    //Image conversion
    unsigned char* convertSensor_msgsImageToRaw(const sensor_msgs::Image& sensorMsgsImage);
    sensor_msgs::Image convertRawToSensorMsgsImage(const unsigned char* raw, const int height, const int width, const std::string&encoding, const char is_bigendian, const int step);

    //Displays
    void displaySensorMsgsImage(const std::string& type, const sensor_msgs::Image& sensorMsgsImage);
    void displayMobileBaseCommandsSound();


private:

    // simple callback to receive image from camera 
    void callbackCameraRgbImageColor(const sensor_msgs::Image& msg);
    
};

#endif
