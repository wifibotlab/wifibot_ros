#ifndef WIFIBOT_H
#define WIFIBOT_H

using namespace std;
#include "Uart.h"
#include "wifibot/wifibot_cmd.h"
#include "ros/ros.h"

#define MODBUS_SYNC_BYTE 254

// ############ CMD LIST ###############

#define MODBUS_CMD_SET_WTD    0x2
/*
 * DATA OCT (1): wtd_h
 * DATA OCT (2): wtd_l
 */

#define MODBUS_CMD_SPEED 0x3
/*
 * DATA OCT (1): Left_L
 * DATA OCT (2): Left_H
 * DATA OCT (3): Right_L
 * DATA OCT (4): Right_H
 * DATA OCT (5): SPEED FLAG
 */

#define MODBUS_CMD_WTD 0x4
/*
 * DATA OCT (1): WTD_L
 * DATA OCT (2): WTD_H
 */

#define MODBUS_DEBUG      0xF0
#define MODBUS_STREAM     0xF1

class Wifibot {

	public:
		Wifibot(Uart *uart, ros::NodeHandle *nh);
		~Wifibot();
		void run(); // launch threads
		void set_speed(unsigned short speed_l, unsigned short speed_r);

		long get_odom_avg();
		long get_odom_avd();
		long get_odom_arg();
		long get_odom_ard();

		double get_temp();
		double get_hygro();
		double get_tension();
		double get_current();

		unsigned char get_speed_r();
		unsigned char get_speed_l();

		static short crc16(unsigned char *adresse_tab, unsigned char taille_max);
		static void parse_recep(Wifibot *wifibot);

		void cmd_callback(const wifibot::wifibot_cmd::ConstPtr& msg);

	private:

		static void *read_th(void *arg);
		static void *write_th(void *arg);

		long odom_avg;
		long odom_avd;
		long odom_arg;
		long odom_ard;

		double tension;
		double current;
		double temp;
		double hygro;

		unsigned char speed_l;
		unsigned char speed_r;

		signed int speed_cmd_l;
		signed int speed_cmd_r;

		unsigned int wtd;	
		bool dir_l;
		bool dir_r;

		Uart *uart;

		unsigned char buff_recep[256];

		ros::NodeHandle *nh;
};

#endif
