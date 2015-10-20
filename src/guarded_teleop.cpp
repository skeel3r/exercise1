#include <iostream>
#include <fstream>
#include <float.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>


using namespace ros;
using namespace std;

class Planner{
	private:
		Publisher pub, rth_pub;
		Subscriber sub, tel_sub, home_sub;
		geometry_msgs::Twist command, teleop;
                geometry_msgs::PoseStamped home;
		double stop_dist, tolerance;
                bool RTH, plan_sent;
		int angle;
		void sub_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
		void teleop_callback(const geometry_msgs::Twist::ConstPtr& user_input);
//		void home_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_pos);


	public:
		NodeHandle my_planner;
		Planner();


};

Planner::Planner(){
	pub = my_planner.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
	rth_pub = my_planner.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	sub = my_planner.subscribe("/scan", 1, &Planner::sub_callback, this);
	tel_sub = my_planner.subscribe("/user_cmd_vel", 1, &Planner::teleop_callback, this);
//	home_sub = my_planner.subscribe("/initialpose", 1, &Planner::home_callback, this);
	command.linear.x = 0.1;
	command.linear.y = 0.0;
	command.linear.z = 0.0;
	command.angular.x = 0.0;
	command.angular.y = 0.0;
	command.angular.z = 0.0;
        tolerance = 0.0;
        RTH = false;
        plan_sent = false;
        home.pose.position.x  = 2.105;
        home.pose.position.y = 2.019;
        home.pose.position.z = 0;
        home.pose.orientation.x = 0;
        home.pose.orientation.y = 0;
        home.pose.orientation.z = .009769;
        home.pose.orientation.w = .999995;

}
//void Planner::home_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_pos){
//        home.header = start_pos->header;
//        home.pose = start_pos->pose.pose;
//}

void Planner::teleop_callback(const geometry_msgs::Twist::ConstPtr& user_input){
        teleop = * user_input;
}
void Planner::sub_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
	double min_dist, tolerance;
        my_planner.getParam("RTH", RTH);
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

	//if(angle < 125){
	//	command.angular.z = 1;
	//}
	//if(angle > 125){
	//	command.angular.z = -1;
	//}

        if(RTH){
          if(plan_sent == false){
            home.header.stamp = Time::now();
            home.header.frame_id = "map";
            rth_pub.publish(home);
            plan_sent = true;
          }
        }
        else if(min_dist < (stop_dist - tolerance)){
	//	command.angular.z = -10;
		command.linear.x = -.1;
	        pub.publish(command);
	}
        else if(min_dist > (stop_dist )){//+ tolerance)){
	//	command.angular.z = 0;
		pub.publish(teleop);
	}
        else if (min_dist < (stop_dist + tolerance) && min_dist > (stop_dist - tolerance)){
	//	command.angular.z = 0;
		command.linear.x = 0.0001;
	        pub.publish(command);
	}
}

int main(int argc, char **argv) {
	// Initialize the subscriber nodd
	init(argc, argv, "my_planner");
	Planner pub;
        //MultiThreadedSpinner spinner(2);
        //spinner.spin();
        ///*
        Rate r(10);
        while(ros::ok()){
          spin();
          r.sleep();
        }
        //*/
}
