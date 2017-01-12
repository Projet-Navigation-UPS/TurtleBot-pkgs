#include "TurtleBotCommand.hpp"

TurtleBotCommand::TurtleBotCommand(ros::NodeHandle& node):
    m_node(node),
    publisherMobileBaseCommandsVelocity(node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1))
{
    TurtleBotCommand::stop();
}

TurtleBotCommand::~TurtleBotCommand()
{}


geometry_msgs::Twist TurtleBotCommand::getMobileBaseCommandsVelocity() 
{
    return mobileBaseCommandsVelocity;
}


void TurtleBotCommand::displayOdom()
{
    std::cout<<odom<<std::endl;
}

//Motions
void TurtleBotCommand::stop()
{
    TurtleBotCommand::setOdom(0, 0, 0, 0, 0, 0);
}

void TurtleBotCommand::move(const float linearVelocity)
{
    TurtleBotCommand::setodom(linearVelocity, 0, 0, 0, 0, 0);
}

void TurtleBotCommand::turn(const float angularVelocity)
{
    TurtleBotCommand::setOdom(0, 0, 0, 0, 0, angularVelocity);
}

void TurtleBotCommand::moveAndTurn(const float linearVelocity, const float angularVelocity)
{
    TurtleBotCommand::setOdom(linearVelocity, 0, 0, 0, 0, angularVelocity);
}

/************************Follow Command************************/
sensor_msgs::Imu TurtleCommand::getOdom()
{
	return odom;
}

sensor_msgs::JointState TurtleCommand::getWheel()
{
	return wheel;
}

void TurtleBotCommand::setOdom(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ)
{
	odom.twist.linear.x=linearX;	 // speed linear
	odom.twist.linear.y=linearY;
	odom.twist.linear.z=linearZ;
	odom.twist.angular.x=angularX;
	odom.twist.angular.y=angularY;
	odom.twist.angular.z=angularZ;	 // speed angular
	publisherOdom.publish(odom);
}

void TurtleBotCommabd::setWheel(const float whp, const float whv, const float whef)
{
    wheel.position=whp;
    wheel.velocity=whv;
    wheel.effort=whef;
    wheel.publish(wheel);
}

// For each point, calculate difference between p and p+1
void TurtleBotCommand::folcom(float tabp[][], const float linearVelocity, const float angularVelocity) 
{
	int T=max(size(tabp);
	float distx=0, disty=0, alpha=0;
	float px0=0, py0=0, theta0=0, px1=0, py1=0, theta1=0;
	float pxe=0, pye=0, thetae=0;
	for(int i=1;i<T-1; i++)
	{
		distx=tabp[1][i]-tabp[1][i-1];
		disty=tabp[2][i]-tabp[2][i-1];

		alpha= acos(distx/sqrt(distx^2+disty^2));
		
		if(alpha<0) alpha=-alpha;
		
		if(distx>=0 && disty>=0) alpha=alpha-pi/2;
		else if(distx<=0 && disty>=0) alpha=pi/2-alpha; 
		else if(distx<=0 && disty<=0) alpha=alpha+pi/2;
		else if(distx>=0 && disty<=0) alpha=-alpha-pi/2;

		px0=odom.pose.position.x;
		py0=odom.pose.position.y;
		theta0=odom.pose.orientation.z;

		TurtleBotCommand::setodom(linearVelocity, 0, 0, 0, 0, angularVelocity);

		px1=odom.pose.position.x-px0;
		py1=odom.pose.position.y-py0;
		theta1=odom.pose.orientation.z-alpha0;
	
		pxe=px1-distx;
		pye=py1-disty;
		thetae=theta1-alpha;

		if(pxe!=0)
		if(pye!=0)
		if(thetae!=0)
	}
}




