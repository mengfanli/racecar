#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#define KEYBOARD_PATH "/dev/input/event4"
#define KEY_PRESSED 1
#define KEY_RELEASED 0

#define KEYCODE_UP 103
#define KEYCODE_DOWN 108
#define KEYCODE_LEFT 105
#define KEYCODE_RIGHT 106
#define KEYCODE_W 17
#define KEYCODE_S 31
#define KEYCODE_A 30
#define KEYCODE_D 32
#define KEYCODE_SPACE 57

int forward=0,backward=0,left=0,right=0,brake=0;

float steering_angle=0.0f;
float steering_angle_velocity=0.0f;
float speed=0.0f;
float acceleration=0.0f;
float jerk=0.0f;

int keyboard_fd;

void mySIGINTHandler(int sig)
{
	close(keyboard_fd);
	ros::shutdown();
}

void keyEventHandler(int keyCode,bool pressed)
{
	switch(keyCode)
	{
		case KEY_UP:
		case KEY_W:
			forward+=pressed?1:-1;
			break;
		case KEY_DOWN:
		case KEY_S:
			backward+=pressed?1:-1;
			break;
		case KEY_LEFT:
		case KEY_A:
			left+=pressed?1:-1;
			break;
		case KEY_RIGHT:
		case KEY_D:
			right+=pressed?1:-1;
			break;
		case KEY_SPACE:
			brake+=pressed?1:-1;
	}
	ROS_DEBUG("Key %d %s.\n",keyCode,pressed?"pressed":"released");
	return;
}

void updateParameter()
{
	steering_angle=steering_angle_velocity=speed=acceleration=jerk=0.0f;
	if(brake>0)
		return;
	if(forward>0)
		speed+=1.2f;
	if(backward>0)
		speed-=1.2f;
	if(left>0)
		//0.34 for about 20 degrees
		steering_angle+=0.34;
	if(right>0)
		steering_angle-=0.34;
}

int main(int argc,char **argv)
{
	struct input_event t;

	//Initialize ROS
	ros::init(argc,argv,"keyboard_remote_control");
	ros::NodeHandle n;
	ros::NodeHandle nh_private("~");
	
	std::string keyboardPath;
	nh_private.param<std::string>("keyboard_path", keyboardPath, KEYBOARD_PATH);
	
	//Initialize device
	ROS_INFO("Trying to remote control through %s...\n",keyboardPath.c_str());
	keyboard_fd=open(keyboardPath.c_str(),O_RDONLY);
	if(keyboard_fd<=0)
	{
		ROS_FATAL("Device %s failed to open.\n",keyboardPath.c_str());
		return -1;
	}
	ROS_INFO("Device initialized\n");
	
	signal(SIGINT,mySIGINTHandler);
	ros::Publisher publisher=n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop",100);

	while(ros::ok())
	{
		ackermann_msgs::AckermannDriveStamped message;

		if(read(keyboard_fd,&t,sizeof(t))==sizeof(t))
		{
			if(t.type==EV_KEY&&(t.value==KEY_PRESSED||t.value==KEY_RELEASED))
			{
				keyEventHandler(t.code,t.value==KEY_PRESSED);
				updateParameter();
			}
			message.header.frame_id="base_link";
			message.header.stamp=ros::Time::now();
			message.drive.steering_angle=steering_angle;
			message.drive.steering_angle_velocity=steering_angle_velocity;
			message.drive.speed=speed;
			message.drive.acceleration=acceleration;
			message.drive.jerk=jerk;
			publisher.publish(message);
		}
		ros::spinOnce();
	}
	close(keyboard_fd);
	return 0;
}
