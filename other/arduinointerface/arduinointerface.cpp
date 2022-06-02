#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <fstream>



//arduinointerface
bool _signal_recieved = false;
double _car_speed = 0.0;
double _steering_angle = 0.0;
// Function Section

void sig_handler(int signo)
{
	if (signo == SIGINT)
	{
		printf("received SIGINT\n");
		_signal_recieved = true;
	}
}

/* msleep(): Sleep for the requested number of milliseconds. */
//https://stackoverflow.com/questions/1157209/is-there-an-alternative-sleep-function-in-c-to-milliseconds
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

/* 
	example serial output from arduino:

    Serial.println("s15.0");
    Serial.println("a-2.0");
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

	FD_ZERO(&fs_read);
	FD_SET(fd, &fs_read);

	time.tv_sec = 10;
	time.tv_usec = 0;

	std::ofstream myfile;
	myfile.open("arduinointerface_output.txt", std::ios::out);

	while (!_signal_recieved) //Loop read data
	{						  //Using select to realize multi-channel communication of serial port

		//auto start = std::chrono::high_resolution_clock::now(); // measuring time between receiving arduino data

		fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
		if (fs_sel)
		{
			len = read(fd, rcv_buf, 10); // 10 is enough for 3digitvalues for speed OR angle (including minus "-" sign)

			//printf("I am right!(version1.2) len = %d fs_sel = %d\n", len, fs_sel);
			if (len > 1)
			{
				rcv_buf[len] = '\0';
				printf("received data: %s \n", rcv_buf);
				//_car_speed = getTagValue(rcv_buf, "s"); DEPRECATED FORMAT of arduino serial data
				//_steering_angle = getTagValue(rcv_buf, "a"); DEPRECATED

				if (rcv_buf[0] == 's'){
					// move chars to the left
					for (int  i = 1 ; i <= len ; i++){
						rcv_buf[i-1] = rcv_buf[i];	
					}
					_car_speed = strtod(rcv_buf, NULL);
				}else if (rcv_buf[0] == 'a'){
					// move chars to the left
					for (int  i = 1 ; i <= len ; i++){
						rcv_buf[i-1] = rcv_buf[i];	
					}
					_steering_angle = strtod(rcv_buf, NULL);
				}
				printf("speed: %f  , steering angle = %f\n", _car_speed, _steering_angle);
				
  				if (myfile.is_open()){
    				myfile << "s" << _car_speed << std::endl;    
  				}else printf("Unable to open file");


			}
			msleep(50);
			//auto stop = std::chrono::high_resolution_clock::now();
			//auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
			//printf("duration =  %ld \n", duration);
		}
		else
		{
			//printf("Sorry,I am wrong!");
		}
	}
 	if (myfile.is_open()){
		myfile.close();
		printf("closed output file\n");
	}
}

// Main Function
int main(int argc, char **argv)
{

    /*
	 * attach signal handler
	 */
	if (signal(SIGINT, sig_handler) == SIG_ERR)
        printf("can't catch SIGINT\n");

    // Function Call
    int fd = openPortToArduino();
    //cout << fd;
    if (fd >= 0)
    {
        readDataFromArduino(fd); // it is a loop that breaks with term signal       
    }

    if (fd >=0){
        close(fd);
        printf("closed port  %d \n", fd);
    }
    return 0;
}
