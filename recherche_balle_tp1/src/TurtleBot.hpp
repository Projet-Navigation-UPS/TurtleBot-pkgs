#ifndef _TURTLEBOT_
#define _TURTLEBOT_

#include <ros/ros.h>
#include <string>
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

class TurtleBot 
{
    
private:
    
    //Subscrbers
    ros::Subscriber subscriberCameraRgbImageRaw;
    ros::Subscriber subscriberCameraRgbImageColor;
    ros::Subscriber subscriberCameraRgbImageRectColor;
    ros::Subscriber subscriberJointStates;
    //Publishers
    ros::Publisher publisherMobileBaseCommandsVelocity;
    ros::Publisher publisherMobileBaseCommandsSound;


    //Messages
    sensor_msgs::Image cameraRgbImageRaw;
    sensor_msgs::Image cameraRgbImageColor;
    sensor_msgs::Image cameraRgbImageRectColor;
    sensor_msgs::JointState jointStates;

    kobuki_msgs::Sound mobileBaseCommandsSound;
    geometry_msgs::Twist mobileBaseCommandsVelocity;

public:

    TurtleBot(ros::NodeHandle node);

    //Getters
    sensor_msgs::Image getCameraRgbImageRaw();
    sensor_msgs::Image getCameraRgbImageColor();
    sensor_msgs::Image getCameraRgbImageRectColor();
    geometry_msgs::Twist getMobileBaseCommandsVelocity();
    sensor_msgs::JointState getJointStates();

    //Setters
    void setMobileBaseCommandsVelocity(float linearX, float linearY, float linearZ, float angularX, float angularY, float angularZ);
    void setMobileBaseCommandsSound(int sound);


    //Callbacks
    void callbackCameraRgbImageRaw(const sensor_msgs::Image& msg);
    void callbackCameraRgbImageColor(const sensor_msgs::Image& msg);
    void callbackCameraRgbImageRectColor(const sensor_msgs::Image& msg);
    void callbackJointStates(const sensor_msgs::JointState& msg);

    //Publications
    void sendMobileBaseCommandsVelocity();
    void sendMobileBaseCommandsSound();

    //Image convertion
    unsigned char* convertSensor_msgsImageToRaw(sensor_msgs::Image sensorMsgsImage);

    //Displays
    void displaySensorMsgsImage(std::string type, sensor_msgs::Image sensorMsgsImage);
    void displayMobileBaseCommandsVelocity();
    void displayJointStates();
    void displayMobileBaseCommandsSound();

    //Motions
    void stop();
    void moveForward();
    void moveBackward();
    void turnRight();
    void turnLeft();
    void moveForwardTurningRight();
    void moveForwardTurningLeft();
    void moveBackwardTurningRight();
    void moveBackwardTurningLeft();

};

#endif
