#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>


using namespace ros;
using namespace std;

class Planner{

	private:
		Publisher pub;
		Subscriber sub;
		geometry_msgs::Twist command;
		double stop_dist, tolerance;
		int angle;
		void sub_callback(const sensor_msgs::LaserScan::ConstPtr& scan);


	public:
		NodeHandle my_planner;
		Planner();


};

Planner::Planner(){
	pub = my_planner.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	sub = my_planner.subscribe("/scan", 1, &Planner::sub_callback, this);
	command.linear.x = 0.1;
	command.linear.y = 0.0;
	command.linear.z = 0.0;
	command.angular.x = 0.0;
	command.angular.y = 0.0;
	command.angular.z = 0.0;
        tolerance = 0;
        stop_dist = .5;
}

void Planner::sub_callback(const sensor_msgs::LaserScan::ConstPtr& scan){

	double min_dist;
	my_planner.getParam("stop_dist", stop_dist);
	my_planner.getParam("tolerance", tolerance);
	min_dist = DBL_MAX;
	angle = 0;
	for(int i = 0; i < scan->ranges.size(); i++){
		if(min_dist > scan->ranges[i]){
			min_dist = scan->ranges[i];
			angle = i;
		}
	}

	if(min_dist < (stop_dist - tolerance)){
		command.linear.x = -.1;

	}
        else if(min_dist > (stop_dist + tolerance)){
		command.linear.x = .1;
	}
	else{
		command.linear.x = 0.0;
	}
	pub.publish(command);
}

int main(int argc, char **argv) {
	// Initialize the subscriber node
	init(argc, argv, "my_planner");
	Planner pub;
        Rate r(10);
        while(ok()){
          spinOnce();
          r.sleep();
        }
}
