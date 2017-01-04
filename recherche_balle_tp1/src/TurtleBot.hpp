#ifndef _TURTLEBOT_
#define _TURTLEBOT_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/Sound.h"

//Constantes
const int SOUND_ON = 0;
const int SOUND_OFF = 1;
const int SOUND_RECHARGE = 2;
const int SOUND_BUTTON = 3;
const int SOUND_ERROR = 4;
const int SOUND_CLEANINGSTART = 5;
const int SOUND_CLEANINGEND = 6;

const float LINEAR_MAX_VELOCITY = 0.25;
const float ANGULAR_MAX_VELOCITY = 2;

const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
const int CAMERA_CHANNELS = 3;
const int CAMERA_STEP_RGB = CAMERA_WIDTH * CAMERA_CHANNELS;
const int CAMERA_STEP_MONO = CAMERA_WIDTH;

const float ROBOT_MAX_LINEAR_VELOCITY = 0.5f;
const float ROBOT_MAX_ANGULAR_VELOCITY = 3.14f;

class TurtleBot 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subscriberCameraRgbImageColor;

    //Publishers
    ros::Publisher publisherMobileBaseCommandsVelocity;
    ros::Publisher publisherMobileBaseCommandsSound;

    //Messages
    sensor_msgs::Image cameraRgbImageColor;
    unsigned char* cameraRgbImageColorRaw;
    std::vector<unsigned char> cameraRgbImageColorVec;
    
    kobuki_msgs::Sound mobileBaseCommandsSound;
    geometry_msgs::Twist mobileBaseCommandsVelocity;
    
    ros::NodeHandle& m_node;
public:

    TurtleBot(ros::NodeHandle& node);
    ~TurtleBot();
    
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
    void displayMobileBaseCommandsVelocity();
    void displayMobileBaseCommandsSound();

    //Motion
    void stop();
    void move(const float linearVelocity);
    void turn(const float angularVelocity);

    void moveAndTurn(const float linearVelocity, const float angularVelocity);

private:
    geometry_msgs::Twist getMobileBaseCommandsVelocity();
    void setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ);

    // simple callback to receive image from camera 
    void callbackCameraRgbImageColor(const sensor_msgs::Image& msg);
    
};

#endif
