#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "Uart.h"
#include "Wifibot.h"

#include "wifibot/wifibot_sensors.h"
#include "wifibot/wifibot_cmd.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "wifibot");
	ros::NodeHandle n;

	ros::Publisher wifibot_pub = n.advertise<wifibot::wifibot_sensors>("wifibot_sensors", 1000);
	wifibot::wifibot_sensors wifibot_sensors_msg;
	
	ros::Rate loop_rate(15);

	Uart *uart = new Uart("/dev/ttyUSB0", 19200);
	if (uart->Open() <= 0)
		return -1;

	Wifibot *wifibot = new Wifibot(uart, &n);

	wifibot->run(); // start read & write threads

	ros::Subscriber wifibot_cmd = n.subscribe("wifibot_cmd", 1000, &Wifibot::cmd_callback, wifibot);
	
	while (ros::ok()) {
		
		wifibot_sensors_msg.odom_avg = wifibot->get_odom_avg();
		wifibot_sensors_msg.odom_avd = wifibot->get_odom_avd();
		wifibot_sensors_msg.odom_arg = wifibot->get_odom_arg();
		wifibot_sensors_msg.odom_ard = wifibot->get_odom_ard();

		wifibot_sensors_msg.temp = wifibot->get_temp();
		wifibot_sensors_msg.hygro = wifibot->get_hygro();
		wifibot_sensors_msg.tension= wifibot->get_tension();
		wifibot_sensors_msg.current = wifibot->get_current();

		wifibot_sensors_msg.speed_av = wifibot->get_speed_av();
		wifibot_sensors_msg.speed_ar = wifibot->get_speed_ar();
		
		wifibot_pub.publish(wifibot_sensors_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}


	uart->close_port();

}
