#include <iostream>
#include <fcntl.h>
#include <unistd.h>
using namespace std;

// Function Section

/*
Where:

device
    The path to the serial port (e.g. /dev/ttyS0)
fd
    The returned file handle for the device. -1 if an error occurred
O_RDWR
    Opens the port for reading and writing
O_NOCTTY
    The port never becomes the controlling terminal of the process.
O_NDELAY
*/
int openPortToArduino()
{
    const char device[] = "/dev/ttyUSB0";
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        printf("failed to open port \n");
    }
    else
    {
        printf("opened port  %d \n", fd);
    }

    // Return the file handler
    return fd;
}

// Main Function
int main(int argc, char **argv)
{

    // Function Call
    int fd = openPortToArduino();
    //cout << fd;
    if (fd >= 0)
    {
        close(fd);
    }
    return 0;
}