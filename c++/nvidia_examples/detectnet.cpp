/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <fstream>  // for config file reading routine
#include <algorithm> // for config file reading routine
#include <unistd.h> // for config file reading routine - getting the path
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <errno.h>
#include <thread>
#include "videoSource.h"
#include "videoOutput.h"

#include "detectNet.h"

#include <signal.h>
#include <openGJK.h>
#include <jetson-utils/cudaCrop.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <math.h>

#include <JetsonGPIO.h>

#ifdef HEADLESS
#define IS_HEADLESS() "headless" // run without display
#else
#define IS_HEADLESS() (const char *)NULL
#endif

bool _signal_recieved = false;
//bool is_croppedimage_mem_allocated = false;
//uchar3 *croppedImage = NULL;
double _cropImagePercentage = 0.4;
double _car_speed = 0.0;
double _steering_angle = 0.0;
double _leftturn_angle = 0.0;
double _rightturn_angle = 0.0;
int _perspective_originL[2] = {200,720};
int _perspective_originR[2] ={1080,720};
float _perspective_angle = 50.0;
const double _PI_ = 3.14159265358979323846;

int gpiopin_buzzer = 11; //USE "BOARD" PIN CONFIGURATION

void sig_handler(int signo)
{
	if (signo == SIGINT)
	{
		LogVerbose("received SIGINT\n");
		_signal_recieved = true;
	}
}

int usage()
{
	printf("usage: detectnet [--help] [--network=NETWORK] [--threshold=THRESHOLD] ...\n");
	printf("                 input_URI [output_URI]\n\n");
	printf("Locate objects in a video/image stream using an object detection DNN.\n");
	printf("See below for additional arguments that may not be shown above.\n\n");
	printf("positional arguments:\n");
	printf("    input_URI       resource URI of input stream  (see videoSource below)\n");
	printf("    output_URI      resource URI of output stream (see videoOutput below)\n\n");

	printf("%s", detectNet::Usage());
	printf("%s", videoSource::Usage());
	printf("%s", videoOutput::Usage());
	printf("%s", Log::Usage());

	return 0;
}

// cang

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

/* Read configuration file */

bool readConfig()
{
	bool result = false;
	char cwd[PATH_MAX];

	if (getcwd(cwd, sizeof(cwd)) != NULL) {
       printf("Current working dir: %s\n", cwd);
   	} else {
       perror("getcwd() error");
       return false;
   	}

    // std::ifstream is RAII, i.e. no need to call close
    std::ifstream cFile;
	cFile.open("/home/jimbo/jetson-inference/examples/detectnet/detectnet_config.txt", std::ifstream::in);
    if (cFile.is_open())
    {
        std::string line;
        while(getline(cFile, line))
       {
            line.erase(std::remove_if(line.begin(), line.end(), isspace),
                                 line.end());
            if( line.empty() || line[0] == '#' )
            {
                continue;
            }
            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);
            std::cout << name << " " << value << '\n';
        }

		result = true;
    }
    else 
    {
        std::cerr << "Couldn't open config file for reading.\n";
    }
	
	if (cFile.is_open())
    {
		cFile.close();
	}

	return result;
}

/*
https://stackoverflow.com/questions/8941213/split-parse-and-get-value-from-a-char-array

NOT USED ANYMORE. CHANGED FORMAT OF ARDUINO INPUT TO BE SIMPLER TO PARSE
e.g.

 
	example serial output from arduino:

    Serial.println("s15.0");
    Serial.println("a-2.0");
    delay(50);

	old format was : Serial.println("s=15.0&a=-2.0");

*/
double getTagValue(char *a_tag_list, char *a_tag)
{
	/* 'strtok' modifies the string. */
	char *tag_list_copy = (char*) malloc(strlen(a_tag_list) + 1);
	char *result = 0;
	double numResult = 0;
	char *s;

	strcpy(tag_list_copy, a_tag_list);

	s = strtok(tag_list_copy, "&");
	while (s)
	{
		char *equals_sign = strchr(s, '=');
		if (equals_sign)
		{
			*equals_sign = 0;
			if (0 == strcmp(s, a_tag))
			{
				equals_sign++;
				result = (char*) malloc(strlen(equals_sign) + 1);
				strcpy(result, equals_sign);
			}
		}
		s = strtok(0, "&");
	}
	free(tag_list_copy);

	numResult = strtod(result, NULL);
	free(result);
	return numResult;
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
	int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
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
void readDataFromArduino(int fd){

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

	int ticksSinceGyroDataReceived = 3; // Normally, we receive 3 speed measurments / second
										// On arduino, if speed > 0 we ask for gyro data
										// If the gyro acceleration on X axis is > 1.0, then we sent the data
										// if we havent received gyro data for a whole second, revert perspective angle to 
										// normal - car is MOST PROBABBLY GOING STRAIGHT ..:-)

	while (!_signal_recieved) //Loop read data
	{						  //Using select to realize multi-channel communication of serial port

		//auto start = std::chrono::high_resolution_clock::now(); // measuring time between receiving arduino data

		fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
		while (fs_sel)
		{
			len = read(fd, rcv_buf, 10); // 10 is enough for 3digitvalues for speed OR angle (including minus "-" sign)

			//printf("I am right!(version1.2) len = %d fs_sel = %d\n", len, fs_sel);
			if (len > 1)
			{
				rcv_buf[len] = '\0';
				//printf("received data: %s \n", rcv_buf);
				//_car_speed = getTagValue(rcv_buf, "s"); DEPRECATED FORMAT of arduino serial data
				//_steering_angle = getTagValue(rcv_buf, "a"); DEPRECATED

				if (rcv_buf[0] == 's'){
					// move chars to the left
					for (int  i = 1 ; i <= len ; i++){
						rcv_buf[i-1] = rcv_buf[i];	
					}
					_car_speed = strtod(rcv_buf, NULL);

					ticksSinceGyroDataReceived++;
					// if we havent received gyro data for a whole second, revert perspective angle to 
					// normal - car is MOST PROBABBLY GOING STRAIGHT ..:-)
					if (ticksSinceGyroDataReceived > 2){
						_rightturn_angle = 0.0;
						_leftturn_angle = 0.0;
					}
				}else if (rcv_buf[0] == 'a'){
					// move chars to the left
					for (int  i = 1 ; i <= len ; i++){
						rcv_buf[i-1] = rcv_buf[i];	
					}
					_steering_angle = strtod(rcv_buf, NULL);
				}else if (rcv_buf[0] == 'L'){
					// car is turning LEFT
					ticksSinceGyroDataReceived = 0;
					_rightturn_angle = 0.0;					
					// move chars to the left					
					for (int  i = 1 ; i <= len ; i++){
						rcv_buf[i-1] = rcv_buf[i];	
					}
					_leftturn_angle = strtod(rcv_buf, NULL);
				}else if (rcv_buf[0] == 'R'){
					// car is turning RIGHT
					ticksSinceGyroDataReceived = 0;
					_leftturn_angle = 0.0;
					// move chars to the left
					for (int  i = 1 ; i <= len ; i++){
						rcv_buf[i-1] = rcv_buf[i];	
					}					
					_rightturn_angle = strtod(rcv_buf, NULL);
				}
								
				//printf("speed: %f  , steering angle = %f, , LEFT turn = %f , RIGHT turn  = %f\n", _car_speed, _steering_angle, _leftturn_angle, _rightturn_angle);

			}
			msleep(10);
			//auto stop = std::chrono::high_resolution_clock::now();
			//auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
			//printf("duration =  %ld \n", duration);
		}
		//else
		
		printf("Sorry, Reading from serial stopped ...!");
		
	}
}

// GPIO using pjueon/JetsonGPIO library, already installed in the system.
// see Github for instructions

void setupGPIO(){
	GPIO::setmode(GPIO::BOARD); // using the "BOARD" pin configuration for identifying pins
	//buzzer pin is OUTPUT pin
	GPIO::setup(gpiopin_buzzer, GPIO::OUT, GPIO::LOW); //"LOW" is for starting with buzzer off
}

void soundBuzzer(int oneorzero){
	GPIO::output(gpiopin_buzzer, oneorzero); // on/off
}

void cleanupGPIO(){
	GPIO::cleanup();
}
/**
 * Crop the image received by the camera, to reduce the strain on the network
 * The cudaCrop() function uses the GPU to crop an images to a particular region of interest (ROI). 
 * For a more comprehensive example, see cuda-examples.py.
 * Note that the ROI rectangles are provided as (left, top, right, bottom) coordinates.
 */

bool cropImage(uchar3 *original, int inputWidth, int inputHeight, double crop_factor)
{

	// determine the amount of border pixels (cropping around the center by half)

	const int2 crop_border = make_int2((1.0f - crop_factor) * 0.5f * inputWidth, (1.0f - crop_factor) * 0.5f * inputHeight);

	// compute the ROI as (left, top, right, bottom)
	//const int4 crop_roi = make_int4(crop_border.x, crop_border.y, inputWidth - crop_border.x, inputHeight - crop_border.y);
	//const int4 crop_roi = make_int4(300, 250, 900 , 720); // for a 720p original image. Cut to the main part of FOV driving
	const int4 crop_roi = make_int4(crop_border.x, crop_border.y * 2, inputWidth - crop_border.x, inputHeight);

	//LogInfo(LOG_TRT "detectnet -- crop_border.x = %d , crop_border.y = %d\n", crop_border.x, crop_border.y);
	//LogInfo(LOG_TRT "detectnet -- crop_roi.left = %d , crop_roi.top = %d, crop_roi.right = %d , crop_roi.bottom = %d\n", crop_roi.x, crop_roi.y, crop_roi.z, crop_roi.w);

	// allocate the output image, with % of the size of the input
	// NO NEED - CROPPPING IS DONE WITH SAVING THE CROPPED IMAGE TO THE EXISTING BUFFER
	/*
	if (!is_croppedimage_mem_allocated)
	{
		if (!cudaAllocMapped(&croppedImage, inputWidth * crop_factor, inputHeight * crop_factor))
			return false;

		is_croppedimage_mem_allocated = true;
	}
	*/
	// crop the image
	if (CUDA_FAILED(cudaCrop(original, original, crop_roi, inputWidth, inputHeight)))
		return false;

	return true;
}

/**
 * Setting up the collsion box that will determinie which objects are in the
 * path of a car
 */
int setupCarCollisionBox(int imageWidth, int imageHeight, double ***pts, int *out)
{
	int npoints = 4;
	int idx = 3; // 0 to 3 (indexes of the positions array)

	// speed is in km/h
	// transform to m/sec with : meters  = speed * 5 /18
	// examples: 10km/h = 2.8 m/sec
	//           20km/h = 5.5 m/sec
	//           30km/h = 8.3 m/sec
	//           40km/h = 11.1 m/sec
	//           50km/h = 13.8 m/sec
	//           60km/h = 16.6 m/sec
	//           70km/h = 19.5 m/sec
	//           80km/h = 22.2 m/sec
	//           90km/h = 25.0 m/sec
	//           100km/h = 27.8 m/sec
	//           110km/h = 30.5 m/sec
	//           120km/h = 33.3 m/sec
	// but we need to allow some more distance / time to react than one second
	// To add half a second more to the safety distance we use instead : meters = speed * 5 / 12
	// So now, the safety distanc we expect when moving with e.g 90km/h is 37,5 meters (not 25)
	// the chart now looks like this:
	// examples: 10km/h = 4.2 m/sec
	//           20km/h = 8.3 m/sec
	//           30km/h = 12.5 m/sec
	//           40km/h = 16.6 m/sec
	//           50km/h = 20.8 m/sec
	//           60km/h = 25.0 m/sec
	//           70km/h = 29.1 m/sec
	//           80km/h = 33.3 m/sec
	//           90km/h = 37.5 m/sec
	//           100km/h = 41.6 m/sec
	//           110km/h = 45.8 m/sec
	//           120km/h = 50.0 m/sec

	double meters = _car_speed * 5 / 12;

	// use a polynomial ecuation to transform meters of distance to pixels in the screen
	// THIS IS DONE WITH recording footage from the camera on board
	// measuring actual distances in the real world,
	// matching them with pixel counting on the image
	// and using something like mycurvefit.com to load the data and get a function like :
	//y = 28 + 18.25714*x - 0.3142857*x^2

	//double pixelsY = 28 + 18.25714 * meters - 0.3142857 * meters * meters;
	
	// y = -9.273608x + 676.0775  -- based on camera fit on car
	double pixelsY = -9.3 * meters + 676;

	// seems good, but needs adjustment
	pixelsY = pixelsY * 0.9;

	// use another polynomial to tranfrom the horizon srinking (in pixels)

	//double pixelsHorizon = 506.2691 - 3.890255 * meters - 0.02255661 * meters * meters;

	// y = -15.16223*x + 952.276 -- based on camera fit on car
	// y = -17.7046*x + 926.0048
	// y = -18.57627*x + 892.7119
	double pixelsHorizon = -18.6 * meters + 892;
	
	// seems good, but needs adjustment
	pixelsHorizon = pixelsHorizon * 0.65;

	/* Allocate memory. */

	double **arr = (double **)malloc(npoints * sizeof(double *));
	for (int i = 0; i < npoints; i++)
		arr[i] = (double *)malloc(2 * sizeof(double));

	/* store vertices' coordinates. */

	arr[0][0] = 130; //(imageWidth / 2) - (imageWidth / 4);
	arr[0][1] = imageHeight;
	arr[0][2] = 0; //z
	arr[1][0] = 1150; //(imageWidth / 2) + (imageWidth / 4);
	arr[1][1] = imageHeight;
	arr[1][2] = 0; //z
	arr[2][0] = (imageWidth / 2) + (pixelsHorizon / 2);
	arr[2][1] = pixelsY; //imageHeight - pixelsY;
	arr[2][2] = 0; //z
	arr[3][0] = (imageWidth / 2) - (pixelsHorizon / 2);
	arr[3][1] = pixelsY; //imageHeight - pixelsY;
	arr[3][2] = 0; //z

	/* Pass pointers. */
	*pts = arr;
	*out = idx;

	return (0);
}


// degrees to radians
//There are 360 degrees in a circle. And that 360 degrees is equivalent to 2*pi radians.
//So, converting and angle x degrees to radians is 2*pi * (x / 360). 
// OR (pi * x / 180)
int getRadians(int degrees){

	int radians = (int) (_PI_ * degrees / 180);
	return radians;
}


void get_points_noturn(int &newX, int &newY, bool isForLeftLineOfPerspective){
    // Any point (x,y) on the path of the circle is x=r∗sin(θ),y=r∗cos(θ)
    //The point (0,r) ends up at x=rsinθ, y=rcosθ
    //In general, suppose that you are rotating about the origin clockwise through an angle θ
    //Then the point (s,t) ends up at (u,v) where
    //u=scosθ+tsinθ and v=−ssinθ+tcosθ.

	double meters = _car_speed * 5 / 12;
	double pixelsY = -9.3 * meters + 676;

    float theta = 0.0;
    if (isForLeftLineOfPerspective){
        theta = 90 - _perspective_angle;
	}else{
        theta = 90 + _perspective_angle;
	}

	theta = _perspective_angle;
    

    newX = abs(pixelsY * sin(getRadians(theta)));
    //newY = abs(pixelsY * cos(getRadians(theta)));

    if (isForLeftLineOfPerspective){
       	newX = newX + _perspective_originL[0];				
	}else{
       	newX = _perspective_originR[0] - newX;		
	}

	newY =  pixelsY;
	
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
/**
 * Setting up the collsion box that will determinie which objects are in the
 * path of a car
 */
int setupCarCollisionBoxSpeedAndAngle(int imageWidth, int imageHeight, double ***pts, int *out)
{
	int npoints = 4;
	int idx = 3; // 0 to 3 (indexes of the positions array)
	int newX;
	int newY;

	/* Allocate memory. */

	double **arr = (double **)malloc(npoints * sizeof(double *));
	for (int i = 0; i < npoints; i++)
		arr[i] = (double *)malloc(2 * sizeof(double));

	/* store vertices' coordinates. */

	arr[0][0] = _perspective_originL[0]; //(imageWidth / 2) - (imageWidth / 4);
	arr[0][1] = imageHeight;
	arr[0][2] = 0; //z
	arr[1][0] = _perspective_originR[0]; //(imageWidth / 2) + (imageWidth / 4);
	arr[1][1] = imageHeight;
	arr[1][2] = 0; //z

	// upper right point of polygon

	if (_rightturn_angle > 0){
		get_points_whileturning(newX, newY, _rightturn_angle, false, false);
	}else if (_leftturn_angle > 0){
		get_points_whileturning(newX, newY, _leftturn_angle, false, true);
	}else {
		get_points_noturn(newX, newY,false); // get ending point for vector of the right line perspective of the collision box
	}

	arr[2][0] = newX;
	arr[2][1] = newY;
	arr[2][2] = 0; //z

	// upper left point of polygon
	if (_rightturn_angle > 0){
		get_points_whileturning(newX, newY, _rightturn_angle, true, false);
	}else if (_leftturn_angle > 0){
		get_points_whileturning(newX, newY, _leftturn_angle, true, true);
	}else {
		get_points_noturn(newX, newY,true); // get ending point for vector of the left line perspective of the collision box
	}

	arr[3][0] = newX;
	arr[3][1] = newY; 
	arr[3][2] = 0; //z

	/* Pass pointers. */
	*pts = arr;
	*out = idx;

	return (0);
}

int transformDetectionBoxToCollisionBox(detectNet::Detection detection, double ***pts, int *out)
{
	int npoints = 4;
	int idx = 3; // 0 to 3 (indexes of the positions array)

	/* Allocate memory. */
	double **arr = (double **)malloc(npoints * sizeof(double *));
	for (int i = 0; i < npoints; i++)
		arr[i] = (double *)malloc(2 * sizeof(double));

	/* store vertices' coordinates. */

	arr[0][0] = detection.Left;
	arr[0][1] = detection.Bottom;
	arr[0][2] = 0; //z
	arr[1][0] = detection.Right;
	arr[1][1] = detection.Bottom;
	arr[1][2] = 0; //z
	arr[2][0] = detection.Right;
	arr[2][1] = detection.Top;
	arr[2][2] = 0; //z
	arr[3][0] = detection.Left;
	arr[3][1] = detection.Top;
	arr[3][2] = 0; //z

	/* Pass pointers. */
	*pts = arr;
	*out = idx;

	return (0);
}

int startVision(commandLine cmdLine)
{

	/* Squared distance computed by openGJK.                                 */
	double dd;
	/* Structure of simplex used by openGJK.                                 */
	struct simplex simplexStructForGJK;
	/* Number of vertices defining body 1 and body 2, respectively.          */
	int nvrtx1,
		nvrtx2;
	/* Structures of car collision box and detected body respectively.                        */
	struct bd bdCarCollisionBox;
	struct bd bdDetectedObject;
	/* Specify name of input files for body 1 and body 2, respectively.      */
	//char   inputfileA[40] = "userP.dat",
	//  inputfileB[40] = "userQ.dat";
	/* Pointers to vertices' coordinates of body 1 and body 2, respectively. */
	double(**vrtx1) = NULL;
	double(**vrtx2) = NULL;

	/*
	 * create input stream
	 */
	videoSource *input = videoSource::Create(cmdLine, ARG_POSITION(0));

	if (!input)
	{
		LogError("detectnet:  failed to create input stream\n");
		return 0;
	}

	/*
	 * create output stream
	 */
	videoOutput *output = videoOutput::Create(cmdLine, ARG_POSITION(1));

	if (!output)
		LogError("detectnet:  failed to create output stream\n");

	/*
	 * create detection network
	 */
	detectNet *net = detectNet::Create(cmdLine);

	if (!net)
	{
		LogError("detectnet:  failed to load detectNet model\n");
		return 0;
	}

	// parse overlay flags
	uint32_t overlayFlags;
	if (!output){
		overlayFlags = 0;
	}else{
	 overlayFlags = detectNet::OverlayFlagsFromStr(cmdLine.GetString("overlay", "box,labels,conf"));
	}

	int widthOfImageToBeProcessed;
	int heightOfImageToBeProcessed;
	/*
	 * processing loop
	 */
	while (!_signal_recieved)
	{

		/* For importing openGJK this is Step 2: adapt the data structure for the
   * two bodies that will be passed to the GJK procedure. */

		setupCarCollisionBoxSpeedAndAngle(input->GetWidth(), input->GetHeight(), &vrtx1, &nvrtx1); // 10meter long
		bdCarCollisionBox.coord = vrtx1;
		bdCarCollisionBox.numpoints = nvrtx1;
		/*
	for (int i = 0; i <= bdCarCollisionBox.numpoints; i++)
	{
		LogInfo(LOG_TRT "detectnet -- x = %f , y = %f\n", bdCarCollisionBox.coord[i][0], bdCarCollisionBox.coord[i][1]);
	}
*/
		if (output != NULL){
			// this is only for displying the collision box area. 
			// Actual calculations is done in this code file
			net->SetCollisionBox(bdCarCollisionBox);
		}
		// capture next image image
		uchar3 *image = NULL;

		if (!input->Capture(&image, 1000))
		{
			// check for EOS
			if (!input->IsStreaming())
				break;

			LogError("detectnet:  failed to capture video frame\n");
			continue;
		}

		// crop the image

		widthOfImageToBeProcessed = input->GetWidth();
		heightOfImageToBeProcessed = input->GetHeight();
		/*
		// crop image
		if (cropImage(image, input->GetWidth(), input->GetHeight(), _cropImagePercentage))
		{
			//image = croppedImage;
			widthOfImageToBeProcessed *= _cropImagePercentage;
			heightOfImageToBeProcessed *= _cropImagePercentage;
		}
		*/
		// detect objects in the frame
		detectNet::Detection *detections = NULL;

		const int numDetections = net->Detect(image, widthOfImageToBeProcessed, heightOfImageToBeProcessed, &detections, overlayFlags);

		int numOfPossibleCollisions = 0;

		if (numDetections > 0)
		{
			//LogVerbose("%i objects detected\n", numDetections);

			for (int n = 0; n < numDetections; n++) // calculate possible collision
			{
				//LogVerbose("detected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
				//LogVerbose("bounding box %i  (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());
				transformDetectionBoxToCollisionBox(detections[n], &vrtx1, &nvrtx1); //
				bdDetectedObject.coord = vrtx1;
				bdDetectedObject.numpoints = nvrtx1;
				/* Initialise simplex as empty */
				simplexStructForGJK.nvrtx = 0;
				/* For importing openGJK this is Step 3: invoke the GJK procedure. */
				/* Compute squared distance using GJK algorithm. */
				dd = gjk(bdCarCollisionBox, bdDetectedObject, &simplexStructForGJK);

				if (dd < 0.1)
				{
					// sound Bell
					//putchar('\07'); // a = alarm
					LogVerbose("collision detected\n");
					numOfPossibleCollisions++;
				}
			}
		}

		// sound buzzer
		if (numOfPossibleCollisions > 0){
			soundBuzzer(1);
		}else{
			soundBuzzer(0);
		}

		// render outputs
		if (output != NULL)
		{
			output->Render(image, widthOfImageToBeProcessed, heightOfImageToBeProcessed);

			// update the status bar
			char str[256];
			//sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS | Possible Collisions %i ", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS(), numOfPossibleCollisions);
			sprintf(str, " %.0f FPS | Collisions %i | car speed  %.0f ", net->GetNetworkFPS(), numOfPossibleCollisions, _car_speed);
			
			output->SetStatus(str);

			// check if the user quit
			if (!output->IsStreaming())
				_signal_recieved = true;
		}

		// print out timing info
		//net->PrintProfilerTimes();
	}

	/*
	 * destroy resources
	 */

	/* Free memory */
	for (int i = 0; i < bdCarCollisionBox.numpoints; i++)
		free(bdCarCollisionBox.coord[i]);
	free(bdCarCollisionBox.coord);
	for (int i = 0; i < bdDetectedObject.numpoints; i++)
		free(bdDetectedObject.coord[i]);
	free(bdDetectedObject.coord);
	//CUDA(cudaFreeHost(croppedImage));

	LogVerbose("detectnet:  shutting down...\n");

	SAFE_DELETE(input);
	SAFE_DELETE(output);
	SAFE_DELETE(net);

	LogVerbose("detectnet:  shutdown complete.\n");
}

int main(int argc, char **argv)
{

	LogVerbose("detectnet:  starting.\n");

	if ( ! readConfig()){
		LogError("can't read config file\n");
		return 0;
	}
	// Function Call
	int serialFileDescriptionArduino = openPortToArduino();

	/*
	 * parse command line
	 */
	commandLine cmdLine(argc, argv, IS_HEADLESS());

	if (cmdLine.GetFlag("help"))
		return usage();

	/*
	 * attach signal handler
	 */
	if (signal(SIGINT, sig_handler) == SIG_ERR)
		LogError("can't catch SIGINT\n");

	//setup GPIO
	setupGPIO();

	// start arduino
	if (serialFileDescriptionArduino >= 0)
	{
		//readDataFromArduino(serialFileDescriptionArduino);
		std::thread thread_Arduino(readDataFromArduino, serialFileDescriptionArduino);
		//thread_Arduino.join(); // IF WE JOIN THIS THREAD, thew program is stuck in the 
								// loop of the thread reading the arduino and does not
								// continue to start the vision task 

		//start vision
		startVision(cmdLine);
	}

	cleanupGPIO();

	// close serial port to arduino
	if (serialFileDescriptionArduino >= 0)
	{
		close(serialFileDescriptionArduino);
	}

	std::terminate(); // terminate any thread we created

	return 0;
}
