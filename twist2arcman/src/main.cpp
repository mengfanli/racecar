#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

int forward=0,backward=0,left=0,right=0,brake=0;

float steering_angle=0.0f;
float steering_angle_velocity=0.0f;
float speed=0.0f;
float acceleration=0.0f;
float jerk=0.0f;
bool update=false;

ackermann_msgs::AckermannDriveStamped message;


void callback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("Received a /cmd_vel message!");
	ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
	// Do velocity processing here:  
	
	steering_angle=steering_angle_velocity=speed=acceleration=jerk=0.0f;
	message.header.frame_id="base_link";
	message.header.stamp=ros::Time::now();
	message.drive.steering_angle=cmd_vel.angular.z;
	message.drive.steering_angle_velocity=steering_angle_velocity;
	message.drive.speed=cmd_vel.linear.x;
	message.drive.acceleration=acceleration;
	message.drive.jerk=jerk;
	update=true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "cmd_vel_listener");
	ros::NodeHandle n;
	ros::Publisher publisher=n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop",100);
	//ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
	
	//ros::spin();
	ros::Subscriber sub = n.subscribe("/cmd_vel", 100, callback);//用/turtle1/cmd_vel做测试哈
	while(ros::ok())
	{
		if(update){
			publisher.publish(message);
			ROS_INFO("publish a /ackermann message!");
			update=false;
		}
		ros::spinOnce();
	}


/*
//http://answers.ros.org/question/129506/subscriber-not-seeing-cmd_vel-messages/
	ros::Rate loop_rate(10);
	while( n.ok() ) 
	{
   		 ros::spin();
	}
*/
	return 1;
}



