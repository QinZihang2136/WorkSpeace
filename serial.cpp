#include <iostream>
#include <serial.h>
#include <termio.h>
#include <unistd.h>
#include <main.h>
using namespace std;

_serial::_serial(const char* filename, const int &buadrate)
{
    fd = open(filename, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
                                                    // No terminal will control the process
    if(fd == -1)
    {
        printf("open_port: Unable to open /dev/port_settingsUSB0.\n");
    }
    else
    {
        fcntl(fd, F_SETFL,0);
        printf("port is open.\n");
    }
    struct termios port_settings;               // structure to store the port settings in
    cfsetispeed(&port_settings, B115200);       // set baud rates
    cfsetospeed(&port_settings, B115200);
#if(0)
    port_settings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    port_settings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    port_settings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    port_settings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    port_settings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    port_settings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    port_settings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

    port_settings.c_oflag &= ~OPOST;/*No Output Processing*/
#else
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
#endif
    port_settings.c_cc[VMIN] = 0;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port
}

void _serial::send_data(const struct serial_transimt_date& data){
//    size_t size = 6;
//    u_char data[10] = {0x55,'f','u','c','k',0x77};
    if(data.size != write(fd, data.raw_data, data.size))
    {
        cout << "!!! send data failure !!!\n" <<endl;
    }
}

void _serial::read_rate_lz(void)
{
    int buffer_size;
    static bool get_head = false;
    unsigned char buffer[10], tmp;
    ioctl(fd, FIONREAD, &buffer_size);
    if(buffer_size > 20)
        cout<< "buffer size = "<< buffer_size <<endl;
    if(buffer_size>0)
    {
        /*
        if(get_head)
        {
            if(buffer_size >= 5)
            {
                read(fd, buffer, 5);
                cout<< "get buffer " << endl;
                if(buffer[4] == 0x77)
                {
                    cout << "get data = " << char(buffer[0]) << char(buffer[1]) << char(buffer[2]) <<endl;
                }
            }
        }*/
        while(buffer_size > 0)
        {
            read(fd, buffer, 1);
            cout<<"buffer = "<< int(buffer[0]) << endl;
            if(buffer[0] == 0x11)
            {
                car_color = 1;      // red
            }
            else if(buffer[0] == 0x22)
            {
                car_color = 2;
            }
            ioctl(fd, FIONREAD, &buffer_size);
        }
    }
}

void _serial::read_rate_cz(void)
{
    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */

    char read_buffer[32];   /* Buffer to store the data received              */
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */
    int i = 0;

    bytes_read = read(fd,&read_buffer,32); /* Read the data                   */

    printf("\n\n  Bytes Rxed -%d", bytes_read); /* Print the number of bytes read */
    printf("\n\n  ");

    for(i=0;i<bytes_read;i++)	 /*printing only the received characters*/
        printf("%c",read_buffer[i]);

    printf("\n +----------------------------------+\n\n\n");
}

/*
 * @breif send the date
 * @param x 
    
*/

void serial_transimt_date::get_xy_date(int16_t x, int16_t y, int8_t found)
{
    size = 7;
    raw_data[0] = head;
    raw_data[size-1] = end;
    raw_data[1] = y & 0xff;
    raw_data[2] = (y>>8) &0xff;
    raw_data[3] = x & 0xff;
    raw_data[4] = (x>>8) &0xff;
    raw_data[5] = found;
}
