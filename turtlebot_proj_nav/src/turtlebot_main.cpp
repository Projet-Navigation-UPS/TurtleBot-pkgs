
#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <string>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/ButtonEvent.h>

//#include <dwa_local_planner/DWAPlannerROS.h>
//#include <dwa_planner_ros.h>
//#include <dynamic_reconfigure/Reconfigure.h>
//#include <DWAPlannerConfig.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


const float map_rotation=-9; // map rotation in degrees
const int MaxPoints=20;
const int MaxPaths=20;

struct goalPoint{
	std::string name;
	double x;
	double y;
	float rot;
};

struct path{
	std::string name;
	goalPoint points[MaxPoints];
	int num_points;
};



bool goTo(double x, double y, float rot, bool with_set_rotation);
void loadPoints(goalPoint points[],int &num_points);
void loadPaths(path paths[],int &num_paths);
void mimicPickUp();
void goPath(std::string name);
void ButtonCallback(kobuki_msgs::ButtonEvent event);


int main(int argc, char** argv){



  ros::init(argc, argv, "testing_node");
  ros::NodeHandle node;
	std::cout << "program is running\n";
	ros::Subscriber sub = node.subscribe("/mobile_base/events/button", 0, ButtonCallback);
        ros::spin();


/*
  char cont='n';

  std::string path_name;
do
{
std::cout << "name path\n";
std::cin >> path_name;
goPath(path_name);
std::cout << "new orders (y/n):\n"; std::cin >> cont;
} while (cont=='y');
*/

  return 0;
};

void ButtonCallback(kobuki_msgs::ButtonEvent event)
{
	if (event.state==event.PRESSED&&event.button==event.Button0){
		goPath("AB");
		goPath("BA");
	}
	else if (event.state==event.PRESSED&&event.button==event.Button1)
		goPath("B1");
	else if (event.state==event.PRESSED&&event.button==event.Button2)
		{
		goPath("B2");
		}
		
/*
	std::cout << "button event trigered\n";
	if (event.state==event.PRESSED)
		std::cout << "pressed\n";
	else
		std::cout << "released\n";
	if (event.button==event.Button0)
		std::cout << "B0\n";
	else if (event.button==event.Button1)
		std::cout << "B1\n";
	else if (event.button==event.Button2)
		std::cout << "B2\n";
*/
}


void updateParam(double rot, double dist){

dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::Config conf;

double_param.name = "yaw_goal_tolerance";
double_param.value = rot;
conf.doubles.push_back(double_param);

double_param.name = "xy_goal_tolerance";
double_param.value = dist;
conf.doubles.push_back(double_param);

srv_req.config = conf;

ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}




void goPath(std::string path_name){
std::cout<<"path called with name " << path_name << "....\n";
path paths[MaxPaths];
int num_paths;
loadPaths(paths,num_paths);
bool path_exists=false;
for (int i=0;i<num_paths;i++)
	if (path_name.compare(paths[i].name)==0){
		path_exists=true;
		std::cout<<"path exists";
		for (int x=0;x<paths[i].num_points-1;x++)
			while (ros::ok()&&goTo(	paths[i].points[x].x,
					paths[i].points[x].y,
					paths[i].points[x].rot,false)==false){
				std::cout << "Point failed retrying point " << paths[i].points[x].name << "..........\n";				
				ros::Duration(2).sleep();
				}
		while (ros::ok()&&goTo(	paths[i].points[paths[i].num_points-1].x,
				paths[i].points[paths[i].num_points-1].y,
				paths[i].points[paths[i].num_points-1].rot,true)==false){
			std::cout << "Point failed retrying " << paths[i].points[paths[i].num_points-1].name << "...........................\n";
			ros::Duration(2).sleep();}
			mimicPickUp();
	}
if (path_exists==false)
	std::cout << "path not found\n";
}











void mimicPickUp(){

ros::NodeHandle n;

ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/navigation_velocity_smoother/raw_cmd_vel", 1);
ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1);

ros::Rate loop_rate(0.5);

geometry_msgs::Twist cmd;
cmd.linear.y=0;
cmd.linear.z=0;
cmd.angular.x=0;
cmd.angular.y=0;
cmd.angular.z=0;
kobuki_msgs::Sound s;
s.value=0;

loop_rate.sleep();

cmd.linear.x=0.5;
vel_pub.publish(cmd);

loop_rate.sleep();

sound_pub.publish(s);

loop_rate.sleep();

cmd.linear.x=-1;
vel_pub.publish(cmd);

loop_rate.sleep();
}



void loadPoints(goalPoint points[],int &num_points)
{
	std::ifstream fs("/home/turtlebot/catkin_ws/src/project/src/goals.txt");
	if (fs.is_open()==true){	
	num_points=0;
	while (!fs.eof()){
		fs >> points[num_points].name >> points[num_points].x >> points[num_points].y >> points[num_points].rot;
		std::cout << "p:" << points[num_points].name << points[num_points].x << points[num_points].y << points[num_points].rot << std::endl;
		num_points++;

}
	fs.close();
	}else ROS_INFO("goals.txt failed to open");
	//ROS_INFO_STREAM("#points="<<num_points << "\n first point: " << points[0].name);
}

void loadPaths(path paths[],int &num_paths)
{
	std::ifstream fs("/home/turtlebot/catkin_ws/src/project/src/paths.txt");
	num_paths=0;
		if (fs.is_open()==true){
	while (!fs.eof()){
		fs >> paths[num_paths].name >> paths[num_paths].num_points;
		std::cout << "path:"<< paths[num_paths].name << " :" << paths[num_paths].num_points << " ";
	for (int i=0;i<paths[num_paths].num_points;i++){
		fs >> paths[num_paths].points[i].name;
		std::cout << 	paths[num_paths].points[i].name << " ";
		}
	std::cout << std::endl;
	num_paths++;
}
	fs.close();
	goalPoint points[20];
	int num_points;
	loadPoints(points,num_points);
	for (int i=0;i<num_paths;i++){
		std::cout << paths[i].name << " :";
	    for (int ppnum=0;ppnum<paths[i].num_points;ppnum++)		
		for (int x=0;x<num_points;x++)
			if (paths[i].points[ppnum].name.compare(points[x].name)==0){
				paths[i].points[ppnum].x=points[x].x;
				paths[i].points[ppnum].y=points[x].y;
				paths[i].points[ppnum].rot=points[x].rot;
					std::cout<<points[x].name << " ";
				}
		std::cout << std::endl;
	}
	}else 
		ROS_INFO("path.txt failed to open!");
			
}








bool goTo(double x, double y, float rot, bool precise)
{
ros::NodeHandle n;
MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
move_base_msgs::MoveBaseGoal goal;

goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = ros::Time::now();
goal.target_pose.pose.position.x = x; 
goal.target_pose.pose.position.y = y;
goal.target_pose.pose.orientation.w=cos(rot*3.1428/360); 
goal.target_pose.pose.orientation.z=sin(rot*3.1428/360); 




	
	if (precise==true){
		//n.setParam("/move_base/DWAPlannerROS/yaw_goal_tolerance", 0.05);
		//n.setParam("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.1);
		updateParam(0.05,0.1);
		ac.sendGoal(goal);
		ac.waitForResult();
	}else{
		//n.setParam("/move_base/DWAPlannerROS/yaw_goal_tolerance", 6.28);
		//n.setParam("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.5);
		updateParam(6.28,0.3);
		ac.sendGoal(goal);
		ac.waitForResult();
	}
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		return true;
	}else{
		ROS_INFO("failed to complete the goal");
		return false;
	}

}








