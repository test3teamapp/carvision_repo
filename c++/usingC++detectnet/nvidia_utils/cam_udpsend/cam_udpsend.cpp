/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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

#include "gstCamera.h"
#include "glDisplay.h"
#include "commandLine.h"

#include <signal.h>

#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h>


#define PORT 5001

bool signal_recieved = false;


void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
		signal_recieved = true;
	}
}


int main( int argc, char** argv )
{
	int sock = 0, valread;
    struct sockaddr_in serv_addr;
	
	commandLine cmdLine(argc, argv);
	
	/*
	 * attach signal handler
	 */	
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		printf("\ncan't catch SIGINT\n");

	/*
	 * create the camera device
	 */
	gstCamera* camera = gstCamera::Create(cmdLine.GetInt("width", gstCamera::DefaultWidth),
								   cmdLine.GetInt("height", gstCamera::DefaultHeight),
								   cmdLine.GetString("camera"));

	if( !camera )
	{
		printf("\ncamera-viewer:  failed to initialize camera device\n");
		return 0;
	}
	
	printf("\ncamera-viewer:  successfully initialized camera device (%ux%u)\n", camera->GetWidth(), camera->GetHeight());
	
	
    // Create socket
    memset(&serv_addr, 0, sizeof(serv_addr)); 
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 


	/*
	 * create openGL window
	 */
	glDisplay* display = glDisplay::Create();
	
	if( !display )
		printf("camera-viewer:  failed to create openGL display\n");
	

	/*
	 * start streaming
	 */
	if( !camera->Open() )
	{
		printf("camera-viewer:  failed to open camera for streaming\n");
		return 0;
	}
	
	printf("camera-viewer:  camera open for streaming\n");
	
	
	/*
	 * processing loop
	 */
	while( !signal_recieved )
	{
		// capture latest image
		float* imgRGBA = NULL;
		
		if( !camera->CaptureRGBA(&imgRGBA, 1000) ){
			printf("camera-viewer:  failed to capture RGBA image\n");
		}else {
			// we have an image. send it
			// checking size
			
			size_t lengthOfArray = camera->GetWidth() * camera->GetHeight() * sizeof(float);
			printf("camera-viewer:  size of imgRGBA array = %d\n",lengthOfArray );
			printf("Camera Viewer: image size (%ux%u)\n", camera->GetWidth(), camera->GetHeight());

			sendto(sock, (const float *)imgRGBA, lengthOfArray, 
			MSG_CONFIRM, (const struct sockaddr *) &serv_addr,  
            sizeof(serv_addr));
		}

		
		// update display
		if( display != NULL )
		{
			display->RenderOnce((float*)imgRGBA, camera->GetWidth(), camera->GetHeight());

			// update status bar
			char str[256];
			sprintf(str, "Camera Viewer (%ux%u) | %.0f FPS", camera->GetWidth(), camera->GetHeight(), display->GetFPS());
			display->SetTitle(str);	

			// check if the user quit
			if( display->IsClosed() )
				signal_recieved = true;
		}
	}
	

	/*
	 * destroy resources
	 */
	printf("\ncamera-viewer:  shutting down...\n");
	printf("\ncamera-viewer:  closing socket...\n");
	close(sock);
	
	SAFE_DELETE(camera);
	SAFE_DELETE(display);

	printf("camera-viewer:  shutdown complete.\n");
	return 0;
}
