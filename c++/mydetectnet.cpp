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

#include "videoSource.h"
#include "videoOutput.h"

#include "detectNet.h"

#include <signal.h>
#include <openGJK.h>

#ifdef HEADLESS
#define IS_HEADLESS() "headless" // run without display
#else
#define IS_HEADLESS() (const char *)NULL
#endif

bool signal_recieved = false;

void sig_handler(int signo)
{
	if (signo == SIGINT)
	{
		LogVerbose("received SIGINT\n");
		signal_recieved = true;
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

int setupCarCollisionBox(double meters, double ***pts, int *out)
{
	int npoints = 4;
	int idx = 3; // 0 to 3 (indexes of the positions array)

	/* Allocate memory. */
	double **arr = (double **)malloc(npoints * sizeof(double *));
	for (int i = 0; i < npoints; i++)
		arr[i] = (double *)malloc(2 * sizeof(double));

	/* store vertices' coordinates. */
	// BASED ON 720p image
	if (meters == 5) // 1-10 km/h
	{
		arr[0][0] = 200;
		arr[0][1] = 0;
		arr[0][0] = 520;
		arr[0][1] = 0;
		arr[0][0] = 520;
		arr[0][1] = 200;
		arr[0][0] = 200;
		arr[0][1] = 200;
	}
	else if (meters == 10)
	{ // 10+ - 40 km/h

		arr[0][0] = 200;
		arr[0][1] = 0;
		arr[0][0] = 520;
		arr[0][1] = 0;
		arr[0][0] = 470;
		arr[0][1] = 300;
		arr[0][0] = 250;
		arr[0][1] = 300;
	}
	else if (meters == 30)
	{ // 40+ km/h

		arr[0][0] = 200;
		arr[0][1] = 0;
		arr[0][0] = 520;
		arr[0][1] = 0;
		arr[0][0] = 420;
		arr[0][1] = 350;
		arr[0][0] = 300;
		arr[0][1] = 350;
	}

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
	arr[0][0] = detection.Right;
	arr[0][1] = detection.Bottom;
	arr[0][0] = detection.Right;
	arr[0][1] = detection.Top;
	arr[0][0] = detection.Left;
	arr[0][1] = detection.Top;

	/* Pass pointers. */
	*pts = arr;
	*out = idx;

	return (0);
}

// cang end

int main(int argc, char **argv)
{

	/* Squared distance computed by openGJK.                                 */
	double dd;
	/* Structure of simplex used by openGJK.                                 */
	struct simplex s;
	/* Number of vertices defining body 1 and body 2, respectively.          */
	int nvrtx1,
		nvrtx2;
	/* Structures of body 1 and body 2, respectively.                        */
	struct bd bdCarCollisionBox;
	struct bd bdDetectedObject;
	/* Specify name of input files for body 1 and body 2, respectively.      */
	//char   inputfileA[40] = "userP.dat",
	//  inputfileB[40] = "userQ.dat";
	/* Pointers to vertices' coordinates of body 1 and body 2, respectively. */
	double(**vrtx1) = NULL,
	(**vrtx2) = NULL;

	/* For importing openGJK this is Step 2: adapt the data structure for the
   * two bodies that will be passed to the GJK procedure. */

	setupCarCollisionBox(30.0, &vrtx1, &nvrtx1); // 30meter long
	bdCarCollisionBox.coord = vrtx1;
	bdCarCollisionBox.numpoints = nvrtx1;

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
	const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr(cmdLine.GetString("overlay", "box,labels,conf"));

	/*
	 * processing loop
	 */
	while (!signal_recieved)
	{
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

		// detect objects in the frame
		detectNet::Detection *detections = NULL;

		const int numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlayFlags);

		int numOfPossibleCollisions = 0;

		if (numDetections > 0)
		{
			LogVerbose("%i objects detected\n", numDetections);

			for (int n = 0; n < numDetections; n++)
			{
				//LogVerbose("detected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
				//LogVerbose("bounding box %i  (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());
				transformDetectionBoxToCollisionBox(detections[n], &vrtx1, &nvrtx1); //
				bdDetectedObject.coord = vrtx1;
				bdDetectedObject.numpoints = nvrtx1;
				/* Initialise simplex as empty */
				s.nvrtx = 0;
				/* For importing openGJK this is Step 3: invoke the GJK procedure. */
				/* Compute squared distance using GJK algorithm. */
				dd = gjk(bdCarCollisionBox, bdDetectedObject, &s);

				if (dd < 0.1)
				{
					// sound Bell
					putchar('\07'); // a = alarm
					numOfPossibleCollisions++;
				}
			}
		}

		// render outputs
		if (output != NULL)
		{
			output->Render(image, input->GetWidth(), input->GetHeight());

			// update the status bar
			char str[256];
			sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS | Possible Collisions %i ", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS(), numOfPossibleCollisions);
			output->SetStatus(str);

			// check if the user quit
			if (!output->IsStreaming())
				signal_recieved = true;
		}

		// print out timing info
		net->PrintProfilerTimes();
	}

	/*
	 * destroy resources
	 */

	/* Free memory */
	for (int i = 0; i < bdCarCollisionBox.numpoints; i++)
		free(bdCarCollisionBox.coord[i]);
	free(bdCarCollisionBox.coord);
	for (int i = 0; i < bdCarCollisionBox.numpoints; i++)
		free(bdCarCollisionBox.coord[i]);
	free(bdCarCollisionBox.coord);

	LogVerbose("detectnet:  shutting down...\n");

	SAFE_DELETE(input);
	SAFE_DELETE(output);
	SAFE_DELETE(net);

	LogVerbose("detectnet:  shutdown complete.\n");
	return 0;
}
