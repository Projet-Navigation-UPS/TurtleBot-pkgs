#include "TurtleBotCamera.hpp"

TurtleBotCamera::TurtleBotCamera(ros::NodeHandle& node):
    m_node(node),
    //Publishers
    publisherMobileBaseCommandsSound(node.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1)),
    //Subcribers
    subscriberCameraRgbImageColor(node.subscribe("/nav/image_converter/output_video", 1, &TurtleBotCamera::callbackCameraRgbImageColor,this)),
    cameraRgbImageColorVec(CAMERA_HEIGHT*CAMERA_STEP_RGB)
{
    cameraRgbImageColorRaw = new unsigned char[sizeof(unsigned char) * CAMERA_HEIGHT*CAMERA_STEP_RGB];
}
TurtleBotCamera::~TurtleBotCamera()
{
    delete cameraRgbImageColorRaw;
}

//Getters
sensor_msgs::Image TurtleBotCamera::getCameraRgbImageColor()
{
    return cameraRgbImageColor;
}

unsigned char* TurtleBotCamera::getCameraRgbImageColorRaw()
{
    return cameraRgbImageColorRaw;
}


void TurtleBotCamera::setMobileBaseCommandsSound(const int sound)
{
    mobileBaseCommandsSound.value = sound;
    publisherMobileBaseCommandsSound.publish(mobileBaseCommandsSound);
}

//Callbacks
// We use a pre allocated vector to facilitate img raw conversion
void TurtleBotCamera::callbackCameraRgbImageColor(const sensor_msgs::Image& msg)
{
    cameraRgbImageColor = msg;
    cameraRgbImageColorVec = msg.data;
    cameraRgbImageColorRaw = &cameraRgbImageColorVec[0];
}

    
sensor_msgs::Image TurtleBotCamera::convertRawToSensorMsgsImage(const unsigned char* raw, const int height, const int width, const std::string& encoding, const char is_bigendian, const int step)
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
void TurtleBotCamera::displaySensorMsgsImage(const std::string& type, const sensor_msgs::Image& sensorMsgsImage)
{
    std::cout<<"--------"<<type<<"---------"<<std::endl;
    std::cout<<sensorMsgsImage.height<<std::endl;
    std::cout<<sensorMsgsImage.width<<std::endl;
    std::cout<<sensorMsgsImage.encoding<<std::endl;
    std::cout<<sensorMsgsImage.is_bigendian<<std::endl;
    std::cout<<sensorMsgsImage.step<<std::endl;
}


void TurtleBotCamera::displayMobileBaseCommandsSound()
{
    std::cout<<mobileBaseCommandsSound<<std::endl;
}
