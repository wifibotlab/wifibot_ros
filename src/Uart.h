//
// Created by rbeal on 15/03/17.
//

#ifndef PROJECT_UART_H
#define PROJECT_UART_H


class Uart {
public:
    Uart();
    Uart(char* file, int speed);
    int Open();
    unsigned char get_char();
    void send_char(unsigned char data);
    void send_str(unsigned char *str, int len);
    void close_port();



private:
    char *file;
    int speed;
    int fd;
};


#endif //PROJECT_UART_H
