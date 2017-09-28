/*
  TurtleBot_command.cpp
  Bruno Dato and Tristan Klempka

  Class to command the turtlebot in speed and distance.
 
 */
#include "TurtleBotCommand.hpp"

// Constructor
TurtleBotCommand::TurtleBotCommand(ros::NodeHandle& node):
    //subscribers
    subCommandReceived(node.subscribe("/nav/open_loop_command", 1, &TurtleBotCommand::callBackCommandReceived,this)),

    //publishers
    pubCommandState(node.advertise<std_msgs::Bool>("/nav/command_busy", 1)),
    publisherMobileBaseCommandsVelocity(node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1))
{
    // Initialization of the command custom message
    commandAsked.linearVelocity = 0;
    commandAsked.angularVelocity = 0;
    commandAsked.distance = 0;
    commandAsked.angle = 0;
    
    // Initialization of the boolean values used for the FSM
    // The command is not busy
    busy.data = false;
    // The robot is not turning
    turning = false;
    // The robot is not moving forward
    moving = false;
    // A mouvement hasen't been asked yet
    startMouvement = false;
    
    // Initialy the robot doesn't move
    stop();
    
    // Indicates on the associated topic that the commande is not busy
    pubCommandState.publish(busy);
}

// Destructor
TurtleBotCommand::~TurtleBotCommand()
{}


//Callbacks
// Updates the custom command message when a command is asked
// Sets the boolean values to start a mouvement on the FSM of the command_node
void TurtleBotCommand::callBackCommandReceived(const turtlebot_proj_nav::command& msg)
{
        ROS_INFO("Command received...");
        //std::cout<<msg<<std::endl;
	    commandAsked = msg;
	    // Set boolean values
	    //turning = true;
	    //moving = false;
	    //startMouvement = true;
	    //busy.data = true;
	    // publish busyness
	    //pubCommandState.publish(busy);  
	    
	    if(commandAsked.distance>0.2){
	        TurtleBotCommand::moveAndTurn(commandAsked.linearVelocity, commandAsked.angularVelocity*commandAsked.angle*5);
	    }  
	    else {
	        TurtleBotCommand::moveAndTurn(0, commandAsked.angularVelocity*commandAsked.angle*5);
	    }
	    

	    
}

// Sends linear and angular speeds to the /mobile_base/commands/velocity topic
void TurtleBotCommand::setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ)
{
    if(linearX > ROBOT_MAX_LINEAR_VELOCITY) mobileBaseCommandsVelocity.linear.x=ROBOT_MAX_LINEAR_VELOCITY;
    else mobileBaseCommandsVelocity.linear.x=linearX;
    mobileBaseCommandsVelocity.linear.y=linearY;
    mobileBaseCommandsVelocity.linear.z=linearZ;
    mobileBaseCommandsVelocity.angular.x=angularX;
    mobileBaseCommandsVelocity.angular.y=angularY;
    if(angularZ > ROBOT_MAX_ANGULAR_VELOCITY) mobileBaseCommandsVelocity.angular.z=ROBOT_MAX_ANGULAR_VELOCITY;
    else mobileBaseCommandsVelocity.angular.z=angularZ;
    // publish the message on the associated topic
    publisherMobileBaseCommandsVelocity.publish(mobileBaseCommandsVelocity);
}

// Displays the current custom command message
void TurtleBotCommand::displayMobileBaseCommandsVelocity()
{
    std::cout<<mobileBaseCommandsVelocity<<std::endl;
}

// Return the current custom command message 
geometry_msgs::Twist TurtleBotCommand::getMobileBaseCommandsVelocity() 
{
    return mobileBaseCommandsVelocity;
}

//Motions
// Stops the robot
void TurtleBotCommand::stop()
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, 0);
    //busy.data = false;
    //pubCommandState.publish(busy);
}

// Makes the robot move forward at a defined linear speed 
void TurtleBotCommand::move(const float linearVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, 0);
}

// Makes the robot move forward at the speed defined by the custom command message
void TurtleBotCommand::move()
{
    TurtleBotCommand::move(commandAsked.linearVelocity);
}

// Makes the robot turn on itself at a defined angular speed 
void TurtleBotCommand::turn(const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, angularVelocity);
}

// Makes the robot turn on itself at the speed defined by the custom command message 
void TurtleBotCommand::turn()
{
    TurtleBotCommand::turn(commandAsked.angularVelocity*1.42);
    // The angular velocity is multiplied by a correction calculated thanks to several mesurements
}

// Makes the robot move forward and turn at the same time at linear and angular defined seeds
void TurtleBotCommand::moveAndTurn(const float linearVelocity, const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, angularVelocity);
}


//Boolean States
bool TurtleBotCommand::start(){return startMouvement;}

bool TurtleBotCommand::movingPhase(){return moving;}

bool TurtleBotCommand::turningPhase(){return turning;}

bool TurtleBotCommand::commandBusy(){return busy.data;}


//Durations
ros::WallDuration TurtleBotCommand::turningDuration()
{
    ros::WallDuration duration = ros::WallDuration(0);
    if(commandAsked.angularVelocity == 0 || commandAsked.angle == 0) 
    {
        // Duration is zero if the angular speed or the angle commanded is zero 
        duration = ros::WallDuration(0);
    }
    else 
    {
        // Positive and negative angular speeds can be asked to the command, angles are always positive
        if (commandAsked.angularVelocity<0) duration = ros::WallDuration(-commandAsked.angle/commandAsked.angularVelocity);
        else duration = ros::WallDuration(commandAsked.angle/commandAsked.angularVelocity);
    }
    // Once the duration is calculated, the movement is started
    startMouvement = false;
    return duration;
    
}


ros::WallDuration TurtleBotCommand::movingDuration()
{
    ros::WallDuration duration;
    if(commandAsked.linearVelocity == 0 || commandAsked.distance == 0) 
    {
        // Duration is zero if the linear speed or the distance commanded is zero 
        duration = ros::WallDuration(0);
    }
    else 
    {
        // Positive and negative linear speeds can be asked to the command, distances are always positive
        if (commandAsked.linearVelocity<0) duration = ros::WallDuration(-commandAsked.distance/commandAsked.linearVelocity);
        else duration = ros::WallDuration(commandAsked.distance/commandAsked.linearVelocity);
    }
    startMouvement = false;
    return duration;
}

//End of motions
// Sets all the booleans values when the turning phase is over and stops the mouvement
void TurtleBotCommand::turningOver()
{
    // turning is over
    turning = false;
    // moving will start
    moving = true;
    // a new mouvement phase will start
    startMouvement = true;
    // the robot is stoped
    stop();
}

// Sets all the booleans values when the moving phase is over and stops the mouvement
void TurtleBotCommand::movingOver()
{
    // moving is over
    moving = false;
    // stop the robot
    stop();
    // the command is not busy anymore
    busy.data = false;
    // publish busyness
    pubCommandState.publish(busy);
}

// publishes the command state on the associated topic
void TurtleBotCommand::publishCommandState()
{
    pubCommandState.publish(busy);
}





