#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>   // for config file reading routine
#include <algorithm> // for config file reading routine
#include <unistd.h>  // for config file reading routine - getting the path
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <errno.h>
#include <thread>
#include <signal.h>
#include <math.h>
#include <Winsock.h>

bool _signal_recieved = false;
double _car_speed = 0.0;
double _steering_angle = 0.0;
double _leftturn_angle = 0.0;
double _rightturn_angle = 0.0;
int _perspective_originL[2] = {200, 720};
int _perspective_originR[2] = {1080, 720};
float _perspective_angle = 50.0;
const double _PI_ = 3.14159265358979323846;

void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        printf("received SIGINT\n");
        _signal_recieved = true;
    }
}

/* msleep(): Sleep for the requested number of milliseconds. */
// https://stackoverflow.com/questions/1157209/is-there-an-alternative-sleep-function-in-c-to-milliseconds
int msleep(long msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do
    {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

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
    const char device[] = "/dev/ttyACM0"; // "/dev/ttyUSB0"; // for arduino mkr 1010 the port shows as ttyACM0
    int fd = open(device, O_RDWR); // not working on windows | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        printf("failed to open port to arduino \n");
    }
    else
    {
        printf("opened port to arduino: %d \n", fd);
    }

    // Return the file handler
    return fd;
}

/*
    example serial output from arduino:

    Serial.println("s15.0");
    Serial.println("a-2.0");
    // getting the g/second from the gyroscope
    Serial.println("L2.1");  Left turn
    Serial.println("R-2.0"); Right turn
    delay(50);
*/
void readDataFromArduino(int fd)
{

    // declaring argument of time()
    time_t my_time = time(NULL);

    char rcv_buf[10];
    int len, fs_sel;

    fd_set fs_read;

    struct timeval time;

    int newX;
    int newY;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = 10;
    time.tv_usec = 0;

    int ticksSinceGyroDataReceived = 3; // Normally, we receive 3 speed measurments / second
                                        // On arduino, if speed > 0 we ask for gyro data
                                        // If the gyro acceleration on X axis is > 1.0, then we sent the data
                                        // if we havent received gyro data for a whole second, revert perspective angle to
                                        // normal - car is MOST PROBABBLY GOING STRAIGHT ..:-)

    while (!_signal_recieved) // Loop read data
    {                         // Using select to realize multi-channel communication of serial port

        // auto start = std::chrono::high_resolution_clock::now(); // measuring time between receiving arduino data

        fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
        while (fs_sel)
        {
            len = read(fd, rcv_buf, 10); // 10 is enough for 3digitvalues for speed OR angle (including minus "-" sign)

            // printf("I am right!(version1.2) len = %d fs_sel = %d\n", len, fs_sel);
            if (len > 1)
            {
                rcv_buf[len] = '\0';
                // printf("received data: %s \n", rcv_buf);
                //_car_speed = getTagValue(rcv_buf, "s"); DEPRECATED FORMAT of arduino serial data
                //_steering_angle = getTagValue(rcv_buf, "a"); DEPRECATED

                if (rcv_buf[0] == 's')
                {
                    // move chars to the left
                    for (int i = 1; i <= len; i++)
                    {
                        rcv_buf[i - 1] = rcv_buf[i];
                    }
                    _car_speed = strtod(rcv_buf, NULL);

                    ticksSinceGyroDataReceived++;
                    // if we havent received gyro data for a whole second, revert perspective angle to
                    // normal - car is MOST PROBABBLY GOING STRAIGHT ..:-)
                    if (ticksSinceGyroDataReceived > 2)
                    {
                        _rightturn_angle = 0.0;
                        _leftturn_angle = 0.0;
                    }
                }
                else if (rcv_buf[0] == 'a')
                {
                    // move chars to the left
                    for (int i = 1; i <= len; i++)
                    {
                        rcv_buf[i - 1] = rcv_buf[i];
                    }
                    _steering_angle = strtod(rcv_buf, NULL);
                }
                else if (rcv_buf[0] == 'L')
                {
                    // car is turning LEFT
                    ticksSinceGyroDataReceived = 0;
                    _rightturn_angle = 0.0;
                    // move chars to the left
                    for (int i = 1; i <= len; i++)
                    {
                        rcv_buf[i - 1] = rcv_buf[i];
                    }
                    _leftturn_angle = strtod(rcv_buf, NULL);
                }
                else if (rcv_buf[0] == 'R')
                {
                    // car is turning RIGHT
                    ticksSinceGyroDataReceived = 0;
                    _leftturn_angle = 0.0;
                    // move chars to the left
                    for (int i = 1; i <= len; i++)
                    {
                        rcv_buf[i - 1] = rcv_buf[i];
                    }
                    _rightturn_angle = strtod(rcv_buf, NULL);
                }

                // printf("speed: %f  , steering angle = %f, , LEFT turn = %f , RIGHT turn  = %f\n", _car_speed, _steering_angle, _leftturn_angle, _rightturn_angle);

                // upper right point of polygon

                if (_rightturn_angle > 0)
                {
                    get_points_whileturning(newX, newY, _rightturn_angle, false, false);
                }
                else if (_leftturn_angle > 0)
                {
                    get_points_whileturning(newX, newY, _leftturn_angle, false, true);
                }
                else
                {
                    get_points_noturn(newX, newY, false); // get ending point for vector of the right line perspective of the collision box
                }

                // upper left point of polygon
                if (_rightturn_angle > 0)
                {
                    get_points_whileturning(newX, newY, _rightturn_angle, true, false);
                }
                else if (_leftturn_angle > 0)
                {
                    get_points_whileturning(newX, newY, _leftturn_angle, true, true);
                }
                else
                {
                    get_points_noturn(newX, newY, true); // get ending point for vector of the left line perspective of the collision box
                }
            }
            msleep(10);
            // auto stop = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            // printf("duration =  %ld \n", duration);
        }
        // else

        printf("Sorry, Reading from serial stopped ...!");
    }
}

// degrees to radians
// There are 360 degrees in a circle. And that 360 degrees is equivalent to 2*pi radians.
// So, converting and angle x degrees to radians is 2*pi * (x / 360).
// OR (pi * x / 180)
int getRadians(int degrees)
{

    int radians = (int)(_PI_ * degrees / 180);
    return radians;
}

void get_points_noturn(int &newX, int &newY, bool isForLeftLineOfPerspective)
{
    // Any point (x,y) on the path of the circle is x=r∗sin(θ),y=r∗cos(θ)
    // The point (0,r) ends up at x=rsinθ, y=rcosθ
    // In general, suppose that you are rotating about the origin clockwise through an angle θ
    // Then the point (s,t) ends up at (u,v) where
    // u=scosθ+tsinθ and v=−ssinθ+tcosθ.

    double meters = _car_speed * 5 / 12;
    double pixelsY = -9.3 * meters + 676;

    float theta = 0.0;
    if (isForLeftLineOfPerspective)
    {
        theta = 90 - _perspective_angle;
    }
    else
    {
        theta = 90 + _perspective_angle;
    }

    theta = _perspective_angle;

    newX = abs(pixelsY * sin(getRadians(theta)));
    // newY = abs(pixelsY * cos(getRadians(theta)));

    if (isForLeftLineOfPerspective)
    {
        newX = newX + _perspective_originL[0];
    }
    else
    {
        newX = _perspective_originR[0] - newX;
    }

    newY = pixelsY;

    printf("isForLeft: %d  , newX = %d,  newY = %d \n", isForLeftLineOfPerspective, newX, newY);
}


void get_points_whileturning(int &newX, int &newY, float theta, bool isForLeftLineOfPerspective, bool isTurningLeft){
    // Any point (x,y) on the path of the circle is x=r∗sin(θ),y=r∗cos(θ)
    //The point (0,r) ends up at x=rsinθ, y=rcosθ
    //In general, suppose that you are rotating about the origin clockwise through an angle θ
    //Then the point (s,t) ends up at (u,v) where
    //u=scosθ+tsinθ and v=−ssinθ+tcosθ.

	double meters = _car_speed * 5 / 12;
	double pixelsY = -9.3 * meters + 676;

    if (theta > 10.0){
        theta = 10.0;
	}


	if (isTurningLeft){
        if (isForLeftLineOfPerspective){ 
            theta = 90 - _perspective_angle + theta;
            //reduce R (distance from center oa a hypothetical circle)
            //on the wheel that is in on the "inside" of the turning angle
            pixelsY = pixelsY - (pixelsY * theta / 100);
		}else{
            theta = 90 + _perspective_angle + theta;
            //pixelsY = pixelsY - (pixelsY * theta / 100) # and less for the "outside" wheel
		}
	}else{ // turnnig  right
        if (isForLeftLineOfPerspective){ 
            theta = 90 - _perspective_angle - theta;
            //pixelsY = pixelsY - (pixelsY * theta / 100)
		}else{
            theta = 90 + _perspective_angle - theta;
            pixelsY = pixelsY - (pixelsY * theta / 100);
		}
	}

    newX = pixelsY * sin(getRadians(theta));
    newY = abs(pixelsY * cos(getRadians(theta)));

    if (isForLeftLineOfPerspective){
        newX = newX + _perspective_originL[0];
	}else{
        newX = _perspective_originR[0] - newX;
	}

}


int main(int argc, char **argv)
{

    // Function Call
    int serialFileDescriptionArduino = openPortToArduino();

    /*
     * attach signal handler
     */
    if (signal(SIGINT, sig_handler) == SIG_ERR)
        printf("can't catch SIGINT\n");

    // start arduino
    if (serialFileDescriptionArduino >= 0)
    {
        // readDataFromArduino(serialFileDescriptionArduino);
        std::thread thread_Arduino(readDataFromArduino, serialFileDescriptionArduino);
        thread_Arduino.join(); // IF WE JOIN THIS THREAD, thew program is stuck in the
                               // loop of the thread reading the arduino and does not
                               // continue to start the vision task
    }

    // close serial port to arduino
    if (serialFileDescriptionArduino >= 0)
    {
        close(serialFileDescriptionArduino);
    }

    std::terminate(); // terminate any thread we created

    return 0;
}
