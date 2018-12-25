#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <iostream>
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <errno.h>      // ERROR Number Definitions
#include <termios.h>    // POSIX Terminal Control Definitions

using namespace std;

struct serial_transimt_date
{
    u_char raw_data[10];
    int size;
    u_char head = 0xaa;
    u_char end = 0xbb;
    void get_xy_date(int16_t x, int16_t y, int8_t found);
};

class _serial
{
public:
    //"/dev/ttyTHS0"
    _serial(const char* filename = "/dev/ttyUSB0", const int& buadrate = 115200);
    void send_data(const struct serial_transimt_date& data);
    void read_rate_cz(void);
    void read_rate_lz(void);
    int fd;
};




#endif // SERIAL_H
