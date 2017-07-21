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

	// this->nh->subscribe("wifibot_cmd", 1, &Wifibot::cmd_callback, this);

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

	while (1) {
		
		buff[0] = MODBUS_SYNC_BYTE;
		buff[1] = 8;
		buff[2] = MODBUS_CMD_SPEED;
		buff[3] = (unsigned char) (wifibot->speed_l & 0xff);
		buff[4] = (unsigned char) (wifibot->speed_l>>8 & 0xff);
		buff[5] = (unsigned char) (wifibot->speed_r & 0xff);
		buff[6] = (unsigned char) (wifibot->speed_r>>8 & 0xff);
		buff[7] = 0;
		if (wifibot->dir_l)
			buff[7] += 64;
		
		if (wifibot->dir_r)
			buff[7] += 16;

		crc = Wifibot::crc16(buff+1, 7);
		buff[8] = (unsigned char) (crc & 0xFF);
		buff[9] = (unsigned char) ((crc>>8) & 0xFF);

		
		wifibot->uart->send_str(buff, 10);
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
		if (buff_read == 0xfe && state == 0)  {

			state = 1;
			nbuff = 1;
			buff[0] = buff_read;
			//printf("OK MAGIC NUMBER! \n\r");
		}
		else if (state == 1) {

			buff[nbuff] = buff_read;
			
			if (nbuff == (buff[1]+1)) { // buff[1]: packet size
				crc_r = Wifibot::crc16(buff+1, (unsigned char) (nbuff-2));

				if ( (buff[nbuff-1] == (crc_r&0xFF) ) && (buff[nbuff] == ((crc_r>>8)&0xFF)) ) {
					//printf("CRC OK !!!\r\n");
					state = 0;
					for (int i=0;i<nbuff;i++)
						wifibot->buff_modbus[i] = buff[i];
					Wifibot::parse_modbus(wifibot);
				}
				else
				{
					printf("CRC NOPE\n\r");
				}

			}
			if (nbuff > (buff[1]+1)) {
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

unsigned char Wifibot::get_speed_av() {
	return this->speed_av;
}

unsigned char Wifibot::get_speed_ar() {
	return this->speed_ar;
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

void Wifibot::parse_modbus(Wifibot *wifibot) {
	switch (wifibot->buff_modbus[2]) {

                case MODBUS_STREAM:

                    wifibot->odom_avg =  wifibot->buff_modbus[3];
                    wifibot->odom_avg += wifibot->buff_modbus[4]<<8;
                    wifibot->odom_avg += wifibot->buff_modbus[5]<<16;
                    wifibot->odom_avg += wifibot->buff_modbus[6]<<24;

                    wifibot->odom_avd =  wifibot->buff_modbus[7];
                    wifibot->odom_avd += wifibot->buff_modbus[8]<<8;
                    wifibot->odom_avd += wifibot->buff_modbus[9]<<16;
                    wifibot->odom_avd += wifibot->buff_modbus[10]<<24;

                    wifibot->odom_arg =  wifibot->buff_modbus[11];
                    wifibot->odom_arg += wifibot->buff_modbus[12]<<8;
                    wifibot->odom_arg += wifibot->buff_modbus[13]<<16;
                    wifibot->odom_arg += wifibot->buff_modbus[14]<<24;

                    wifibot->odom_ard =  wifibot->buff_modbus[15];
                    wifibot->odom_ard += wifibot->buff_modbus[16]<<8;
                    wifibot->odom_ard += wifibot->buff_modbus[17]<<16;
                    wifibot->odom_ard += wifibot->buff_modbus[18]<<24;


                    wifibot->tension = (double)wifibot->buff_modbus[19] /10;
                    wifibot->current = (double)wifibot->buff_modbus[20] ;

                    wifibot->temp = (double)wifibot->buff_modbus[21] / 2;
                    wifibot->hygro = (double)wifibot->buff_modbus[22] / 2;

                    wifibot->speed_av = wifibot->buff_modbus[23];
                    wifibot->speed_ar = wifibot->buff_modbus[24];
                    break;

                default:
		    
                    break;
            }

}

void Wifibot::cmd_callback(const wifibot::wifibot_cmd::ConstPtr& msg) {
	
	this->speed_l = msg->speed_l;
	this->speed_r = msg->speed_r;
	
	this->dir_l = msg->dir_l;
	this->dir_r = msg->dir_r;

	// ROS_INFO("CMD OK DIRL=(%d)", this->dir_l);
	// printf("R:%u\n\r", this->speed_r);
	// printf("L:%u\n\r", this->speed_l);
}

