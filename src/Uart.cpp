//
// Created by rbeal on 15/03/17.
//

#include "Uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fcntl.h>
#include <string.h>
#include "ros/ros.h"
#include <termios.h>

Uart::Uart() {

}

Uart::Uart(char *file, int speed) {

    this->file = (char *)malloc(20);

    memccpy(this->file, file, 0, 20); // file copy
    this->speed = speed;              // speed copy
}

int Uart::Open() {

    struct termios options;

    fd = open(this->file, O_RDWR | O_NOCTTY);
    if (fd <= 0) {
        ROS_ERROR("UNABLE TO OPEN PORT");
        return this->fd;
    }

    tcgetattr(fd, &options);

    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);

    options.c_cflag = B19200 | CS8 | CREAD | CLOCAL | HUPCL;
    options.c_iflag = IGNPAR;
    options.c_lflag = 0;
    options.c_oflag = 0;

    tcsetattr(fd, TCSANOW, &options);

    return this->fd;
}

unsigned char Uart::get_char() {

    ssize_t size;
    char buff;

    size = read(this->fd, &buff, 1);
    if (size != 1)
        ROS_ERROR("READ ERROR");

    return (unsigned char)buff;
}

void Uart::send_char(unsigned char data) {

    write(this->fd, &data, 1);
}

void Uart::send_str(unsigned char *str, int len) {

    for (int i=0;i<len;i++)
        send_char(*(str+i));
}

void Uart::close_port() {

    close(this->fd);
}
