#include "TurtleBot.hpp"

TurtleBot::TurtleBot(ros::NodeHandle& node):
    m_node(node),
    //Publishers
    publisherMobileBaseCommandsVelocity(node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)),
    publisherMobileBaseCommandsSound(node.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1)),
    //Subcribers
    subscriberCameraRgbImageColor(node.subscribe("/image_converter/output_video", 1, &TurtleBot::callbackCameraRgbImageColor,this)),
    cameraRgbImageColorVec(CAMERA_HEIGHT*CAMERA_STEP_RGB)
{
    cameraRgbImageColorRaw = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*CAMERA_STEP_RGB];
    TurtleBot::stop();
}
TurtleBot::~TurtleBot()
{
    delete cameraRgbImageColorRaw;
}

//Getters
sensor_msgs::Image TurtleBot::getCameraRgbImageColor()
{
    return cameraRgbImageColor;
}

unsigned char* TurtleBot::getCameraRgbImageColorRaw()
{
    return cameraRgbImageColorRaw;
}

geometry_msgs::Twist TurtleBot::getMobileBaseCommandsVelocity() 
{
    return mobileBaseCommandsVelocity;
}

void TurtleBot::setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ)
{
    mobileBaseCommandsVelocity.linear.x=linearX;
    mobileBaseCommandsVelocity.linear.y=linearY;
    mobileBaseCommandsVelocity.linear.z=linearZ;
    mobileBaseCommandsVelocity.angular.x=angularX;
    mobileBaseCommandsVelocity.angular.y=angularY;
    mobileBaseCommandsVelocity.angular.z=angularZ;
    publisherMobileBaseCommandsVelocity.publish(mobileBaseCommandsVelocity);
}

void TurtleBot::setMobileBaseCommandsSound(const int sound)
{
    mobileBaseCommandsSound.value = sound;
    publisherMobileBaseCommandsSound.publish(mobileBaseCommandsSound);
}

//Callbacks
// We use a pre allocated vector to facilitate img raw conversion
void TurtleBot::callbackCameraRgbImageColor(const sensor_msgs::Image& msg)
{
    cameraRgbImageColor = msg;
    cameraRgbImageColorVec = msg.data;
    cameraRgbImageColorRaw = &cameraRgbImageColorVec[0];
}
    
sensor_msgs::Image TurtleBot::convertRawToSensorMsgsImage(const unsigned char* raw, const int height, const int width, const std::string& encoding, const char is_bigendian, const int step)
{	
    sensor_msgs::Image sensorMsgsImage;
    std::vector<unsigned char> vectorRaw(raw, raw + height*step);
    sensorMsgsImage.data = vectorRaw;
    sensorMsgsImage.height = height;
    sensorMsgsImage.width = width;
    sensorMsgsImage.encoding = encoding;
    sensorMsgsImage.is_bigendian = is_bigendian;
    sensorMsgsImage.step = step;
    return sensorMsgsImage;
}

//Displays
void TurtleBot::displaySensorMsgsImage(const std::string& type, const sensor_msgs::Image& sensorMsgsImage)
{
    std::cout<<"--------"<<type<<"---------"<<std::endl;
    std::cout<<sensorMsgsImage.height<<std::endl;
    std::cout<<sensorMsgsImage.width<<std::endl;
    std::cout<<sensorMsgsImage.encoding<<std::endl;
    std::cout<<sensorMsgsImage.is_bigendian<<std::endl;
    std::cout<<sensorMsgsImage.step<<std::endl;
}

void TurtleBot::displayMobileBaseCommandsVelocity()
{
    std::cout<<mobileBaseCommandsVelocity<<std::endl;
}

void TurtleBot::displayMobileBaseCommandsSound()
{
    std::cout<<mobileBaseCommandsSound<<std::endl;
}

//Motions
void TurtleBot::stop()
{
    TurtleBot::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, 0);
}

void TurtleBot::move(const float linearVelocity)
{
    TurtleBot::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, 0);
}

void TurtleBot::turn(const float angularVelocity)
{
    TurtleBot::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, angularVelocity);
}

void TurtleBot::moveAndTurn(const float linearVelocity, const float angularVelocity)
{
    TurtleBot::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, angularVelocity);
}
