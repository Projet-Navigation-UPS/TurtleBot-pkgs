#include "TurtleBotCommand.hpp"

TurtleBotCommand::TurtleBotCommand(ros::NodeHandle& node):
    m_node(node),
    publisherOdom(node.advertise<nav_msgs::Odometry>("/odom", 50))
{
    TurtleBotCommand::stop();
}

TurtleBotCommand::~TurtleBotCommand()
{}


void TurtleBotCommand::displayMobileBaseCommandsVelocity()
{
    std::cout<<odom<<std::endl;
}

//Motions
void TurtleBotCommand::stop()
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, 0);
}

void TurtleBotCommand::move(const float linearVelocity, const float mile)
{
    float px0=0, px1=0;
    px0=odom.pose.pose.position.x;

    if((px1=odom.pose.pose.position.x-px0) < mile) TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, 0);
}

void TurtleBotCommand::turn(const float angularVelocity, const float angu)
{
    float theta0=0, theta1=0;
    theta0=odom.pose.pose.orientation.z;

    if((theta1=odom.pose.pose.orientation.z-theta0) < angu) TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, angularVelocity);
}

void TurtleBotCommand::moveAndTurn(const float linearVelocity, const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, angularVelocity);
}

/************************Follow Command************************/
nav_msgs::Odometry TurtleBotCommand::getOdom()
{
	return odom;
}

sensor_msgs::JointState TurtleBotCommand::getWheel()
{
	return wheel;
}

geometry_msgs::Twist TurtleBotCommand::getMobileBaseCommandsVelocity()
{
	return mobileBaseCommandsVelocity;
}

void TurtleBotCommand::setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ)
{
	mobileBaseCommandsVelocity.linear.x=linearX;	 // speed linear
	mobileBaseCommandsVelocity.linear.y=linearY;
	mobileBaseCommandsVelocity.linear.z=linearZ;
	mobileBaseCommandsVelocity.angular.x=angularX;
	mobileBaseCommandsVelocity.angular.y=angularY;
	mobileBaseCommandsVelocity.angular.z=angularZ;	 // speed angular
	publisherMobileBaseCommandsVelocity.publish(mobileBaseCommandsVelocity);
}


/*void TurtleBotCommand::setOdom(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ)
{
	odom.twist.twist.linear.x=linearX;	 // speed linear
	odom.twist.twist.linear.y=linearY;
	odom.twist.twist.linear.z=linearZ;
	odom.twist.twist.angular.x=angularX;
	odom.twist.twist.angular.y=angularY;
	odom.twist.twist.angular.z=angularZ;	 // speed angular
	publisherOdom.publish(odom);
}*/

/*void TurtleBotCommand::setWheel(const float whv, const float whef)
{
    wheel.position=whp;
    wheel.velocity=whv;
    wheel.effort=whef;
    wheel.publish(wheel);
}*/

// For each point, calculate difference between p and p+1
/*void TurtleBotCommand::folcom(std::vector<std::vector<const float> > tabp, const float linearVelocity, const float angularVelocity) 
{
	int T=tabp.max_size();

	float distx=0, disty=0, alpha=0;
	float px0=0, py0=0, theta0=0, px1=0, py1=0, theta1=0;
	float pxe=0, pye=0, thetae=0;
	for(int i=1;i<T-1; i++)
	{
		distx=tabp[1][i]-tabp[1][i-1];
		disty=tabp[2][i]-tabp[2][i-1];

		alpha= acos(distx/sqrt(exp2(distx)+exp2(disty)));
		
		if(alpha<0) alpha=-alpha;
		
		if(distx>=0 && disty>=0) alpha=alpha-pi/2;
		else if(distx<=0 && disty>=0) alpha=pi/2-alpha; 
		else if(distx<=0 && disty<=0) alpha=alpha+pi/2;
		else if(distx>=0 && disty<=0) alpha=-alpha-pi/2;

		px0=odom.pose.pose.position.x;
		py0=odom.pose.pose.position.y;
		theta0=odom.pose.pose.orientation.z;

		TurtleBotCommand::setOdom(linearVelocity, 0, 0, 0, 0, angularVelocity);

		px1=odom.pose.pose.position.x-px0;
		py1=odom.pose.pose.position.y-py0;
		theta1=odom.pose.pose.orientation.z-theta0;
	
		pxe=px1-distx;
		pye=py1-disty;
		thetae=theta1-alpha;

		if(pxe!=0)
		if(pye!=0)
		if(thetae!=0)
	}
}
*/



