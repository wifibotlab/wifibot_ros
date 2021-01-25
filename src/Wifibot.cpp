#include "Wifibot.h"
#include "pthread.h"
#include "ros/ros.h"
#include "Uart.h"
#include <unistd.h>

using namespace std;

Wifibot::Wifibot(Uart *uart, ros::NodeHandle *nh) {
	this->uart = uart;

	this->speed_l = 0;
	this->speed_r = 0;

	this->nh = nh;

	this->odom_avg = 0;
	this->odom_avd = 0;
	this->odom_arg = 0;
	this->odom_ard = 0;

	this->tension = 0;
	this->current = 0;
	this->temp = 0;
	this->hygro = 0;

	this->dir_l = false;
	this->dir_r = false;

	this->wtd = 0;
}

void Wifibot::run() {
	
	int tmp;

	 this->nh->subscribe("wifibot_cmd", 1, &Wifibot::cmd_callback, this);

	pthread_t thread_read, thread_write;

	if ( pthread_create(&thread_read, NULL, Wifibot::read_th, this)) {
			ROS_INFO("THREAD READ ERROR");
	}


	if ( pthread_create(&thread_write, NULL, Wifibot::write_th, this)) {
			ROS_INFO("THREAD WRITE ERROR");
	}
}

void * Wifibot::write_th(void *arg) {

	Wifibot *wifibot;
	wifibot = (Wifibot *) arg;

	unsigned char buff[30];

	short crc;

	unsigned int l, r;

	while (1) {

		l = wifibot->speed_cmd_l / 273; // 240 max
		r = wifibot->speed_cmd_r / 273; // 240 max

		buff[0] = 255;
		buff[1] = 7;
		buff[2] = (unsigned char) (l & 0xff);
		buff[3] = (unsigned char) 0;
		buff[4] = (unsigned char) (r & 0xff);
		buff[5] = (unsigned char) 0;
		buff[6] = 0;
		if (!wifibot->dir_l)
			buff[6] += 64;
		
		if (!wifibot->dir_r)
			buff[6] += 16;

		crc = Wifibot::crc16(buff+1, 6);
		buff[7] = (unsigned char) (crc & 0xFF);
		buff[8] = (unsigned char) ((crc>>8) & 0xFF);

		
		wifibot->uart->send_str(buff, 9);
		usleep(10000);

	}
}

void * Wifibot::read_th(void *arg) {
	
	Wifibot *wifibot;
	wifibot = (Wifibot *) arg;

	int size;
	unsigned char buff_read;
	unsigned char buff[30];
	int nbuff=0;

	short crc_r;
	int state = 0;
	/*
	 * state:
	 * 0- waiting for...
	 * 1- Start byte OK !
	 */

	while (1) {
		
		buff_read = wifibot->uart->get_char();
		if (buff_read == 0xff && state == 0)  {

			state = 1;
			nbuff = 1;
			buff[0] = buff_read;
		//	printf("OK MAGIC NUMBER! \n\r");
		}
		else if (state == 1) {

			buff[nbuff] = buff_read;
			
			if (nbuff == 21) { // packet size = 21
				crc_r = Wifibot::crc16(buff+1, (unsigned char) 19);

				if ( (buff[nbuff-1] == (crc_r&0xFF) ) && (buff[nbuff] == ((crc_r>>8)&0xFF)) ) {
				//	printf("CRC OK !!!\r\n");
					state = 0;
					for (int i=0;i<nbuff;i++)
						wifibot->buff_recep[i] = buff[i];
					Wifibot::parse_recep(wifibot);
				}
				else
				{
					printf("CRC NOPE\n\r");
				}

			}
			if (nbuff > 22) {
				state = 0;
				printf("Uart error \n\r");
			}

			nbuff++;
		}

	}
}

long Wifibot::get_odom_avg() {
	return this->odom_avg;
}

long Wifibot::get_odom_avd() {
	return this->odom_avd;
}

long Wifibot::get_odom_arg() {
	return this->odom_arg;
}

long Wifibot::get_odom_ard() {
	return this->odom_ard;
}

double Wifibot::get_temp() {
	return this->temp;
}

double Wifibot::get_hygro() {
	return this->hygro;
}

double Wifibot::get_tension() {
	return this->tension;
}

double Wifibot::get_current() {
	return this->current;
}

unsigned char Wifibot::get_speed_r() {
	return this->speed_r;
}

unsigned char Wifibot::get_speed_l() {
	return this->speed_l;
}

short Wifibot::crc16(unsigned char *adresse_tab , unsigned char taille_max) {

    unsigned int Crc = 0xFFFF;
    unsigned int Polynome = 0xA001;
    unsigned int CptOctet = 0;
    unsigned int CptBit = 0;
    unsigned int Parity= 0;

    Crc = 0xFFFF;
    Polynome = 0xA001; // Polynôme = 2^15 + 2^13 + 2^0 = 0xA001.

    for ( CptOctet= 0 ; CptOctet < taille_max ; CptOctet++)
    {
        Crc ^= *( adresse_tab + CptOctet); //Ou exculsif entre octet message et CRC

        for ( CptBit = 0; CptBit <= 7 ; CptBit++) /* Mise a 0 du compteur nombre de bits */
        {
            Parity= Crc;
            Crc >>= 1; // Décalage a droite du crc
            if (Parity%2 == 1) Crc ^= Polynome; // Test si nombre impair -> Apres decalage à droite il y aura une retenue
        } // "ou exclusif" entre le CRC et le polynome generateur.
    }
    return(Crc);
}

void Wifibot::parse_recep(Wifibot *wifibot) {

	wifibot->speed_l = (wifibot->buff_recep[1] | wifibot->buff_recep[2] << 8);
	wifibot->speed_r = (wifibot->buff_recep[10] | wifibot->buff_recep[11] << 8);


	wifibot->odom_avg =  wifibot->buff_recep[6];
	wifibot->odom_avg += wifibot->buff_recep[7]<<8;
	wifibot->odom_avg += wifibot->buff_recep[8]<<16;
	wifibot->odom_avg += wifibot->buff_recep[9]<<24;

	wifibot->odom_avd =  wifibot->buff_recep[14];
	wifibot->odom_avd += wifibot->buff_recep[15]<<8;
	wifibot->odom_avd += wifibot->buff_recep[16]<<16;
	wifibot->odom_avd += wifibot->buff_recep[17]<<24;

	wifibot->tension = (double)wifibot->buff_recep[3];
	wifibot->current = (double)wifibot->buff_recep[18] ;

}

void Wifibot::cmd_callback(const wifibot::wifibot_cmd::ConstPtr& msg) {
	
	this->speed_cmd_l = msg->speed_l;
	this->speed_cmd_r = msg->speed_r;
	
	this->dir_l = msg->dir_l;
	this->dir_r = msg->dir_r;

	// ROS_INFO("CMD OK DIRL=(%d)", this->dir_l);
	 printf("R:%u\n\r", this->speed_cmd_r);
	 printf("L:%u\n\r", this->speed_cmd_l);
}

