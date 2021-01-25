#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>
#include "wifibot/wifibot_cmd.h"

ros::Publisher wifibot_cmd;
wifibot::wifibot_cmd cmd_msg;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	  
	  cmd_msg.dir_r = false;
	  cmd_msg.dir_l = false;
	  
	signed int left = joy->axes[1]*65535/2 - joy->axes[0]*65535/2;
	signed int right = joy->axes[1]*65535/2 + joy->axes[0]*65535/2;

	  
	  if (left < 0) {
		cmd_msg.speed_l = 65535-left;
	  	cmd_msg.dir_l = true;
	  }
	  else
	  {
		cmd_msg.speed_l = left;
	  }

	  if (right < 0) {
		cmd_msg.speed_r = 65535-right;
	  	cmd_msg.dir_r = true;
	  }
	  else
	  {
		cmd_msg.speed_r = right;
	  }

	  wifibot_cmd.publish(cmd_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop");

	ros::NodeHandle n;

	wifibot_cmd = n.advertise<wifibot::wifibot_cmd>("wifibot_cmd", 1000);

	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);

	ros::spin();

	return 0;
}
