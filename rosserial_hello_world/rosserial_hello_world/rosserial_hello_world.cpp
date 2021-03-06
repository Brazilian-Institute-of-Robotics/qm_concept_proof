// rosserial_hello_world.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <windows.h>

using std::string;

	void pose_callback(const std_msgs::String & msg)
	{
		//printf("Received pose %f, %f, %f\n", pose.position.x,
			//pose.position.y, pose.position.z);
		printf("msg received %s\n", msg.data);
	}

int main()
{
	ros::NodeHandle nh;
	char *ros_master = "192.168.0.1";

	printf("Connecting to server at %s\n", ros_master);
	nh.initNode(ros_master);

	printf("Advertising cmd_vel message\n");
	geometry_msgs::Twist twist_msg;

	ros::Subscriber<std_msgs::String> pose_sub("chatter", &pose_callback);
	nh.subscribe(pose_sub);

	ros::Publisher cmd_vel_pub("turtle1/cmd_vel", &twist_msg);
	nh.advertise(cmd_vel_pub);

	printf("Go robot go!\n");
	while (1)
	{
		twist_msg.linear.x = 5.1;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;
		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = -1.8;
		cmd_vel_pub.publish(&twist_msg);
		
		nh.spinOnce();
		Sleep(100);
	}

	printf("All done!\n");
	return 0;
}

