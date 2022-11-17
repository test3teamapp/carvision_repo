from pickle import FALSE
import time

import cv2
import os
import logging

from multiprocessing import Process, Value, Array
from multiprocessing import Queue
from enum import Enum

# for jetson
import jetson.inference
import jetson.utils
import sys
import cv2

_DEBUG = False 
_OUTPUT = True
_FPSINFO = False 


class DETECT_PROCESS_STATE(Enum):
    STOPPED = 1
    RUNNING = 2


class JetsonDetector:
    """Class for proccessing images on Jetson GPU"""

    # variables here are common to all instances of the class #

    def __init__(self, sharedImageQueue: Queue, sharedDataArray: Array): 
        self.detectProcess = Process()
        # create queues for accessing the state variable from the new process
        # and send the images received by the TCP process for processing
        self.imageQueue = sharedImageQueue
        
        #the data array should contain values for [0]speed,[1]y-axis accelaration 
        #that have been received by the UDP server (main process that has spawned the rest)
        #and are used here to be combined with the detections for defining possible collisions
        self.sharedDataArray = sharedDataArray
        
        self.stateQueue = Queue()
        self.startDetectProcess = False
        self.detectProcessState = DETECT_PROCESS_STATE.STOPPED
        self.stateQueue.put(self.detectProcessState)

    def cudaFromCV(self, cv_img):

        #print('OpenCV image size: ' + str(cv_img.shape))
        #print('OpenCV image type: ' + str(cv_img.dtype))

        # convert to CUDA (cv2 images are numpy arrays, in BGR format)
        bgr_img = jetson.utils.cudaFromNumpy(cv_img, isBGR=True)

        #print('BGR image: ')
        #print(bgr_img)

        # convert from BGR -> RGB
        rgb_img = jetson.utils.cudaAllocMapped(width=bgr_img.width,
                                       height=bgr_img.height,
                                       format='rgb8')

        jetson.utils.cudaConvertColor(bgr_img, rgb_img)

        #print('RGB image: ')
        #print(rgb_img)

        return rgb_img


    def detectInImageProcess(self, imgQ: Queue, stateQ: Queue):

        is_headless = ""
        threshold = 0.4
        networkName = "ssd-mobilenet-v2"

        if (_OUTPUT):
            # create video output object
            output = jetson.utils.videoOutput(
                "display://0")  # 'my_video.mp4' for file
        # load the object detection network
        net = jetson.inference.detectNet(
            networkName, sys.argv, threshold=0.5)

        overlayConfig = "box,labels,conf"

        self.detectProcessState = DETECT_PROCESS_STATE.RUNNING
        stateQ.put(self.detectProcessState)

        while (self.startDetectProcess):
            if (not imgQ.empty()):
                # prepare image
                cv2_img = imgQ.get()
                cudaImage = self.cudaFromCV(cv2_img)
                # self.my_print(f"received image size : {cv2_im.size}")
                self.my_print(f"received image dimentions : {cv2_img.shape}")
                # detect objects in the image (with overlay)
                detections = net.Detect(cudaImage, overlay=overlayConfig)

                if (_DEBUG):
                    # print the detections
                    print("detected {:d} objects in image".format(
                        len(detections)))

                    for detection in detections:
                        print(detection)

                if (_OUTPUT):
                    # render the image
                    output.Render(cudaImage)

                    # update the title bar
                    output.SetStatus("{:s} | Network {:.0f} FPS".format(
                        networkName, net.GetNetworkFPS()))

                if (_DEBUG):
                    # print out performance info
                    net.PrintProfilerTimes()

                if (_FPSINFO):
                    print("fps : {:.0f}".format(net.GetNetworkFPS()))


        self.detectProcessState = DETECT_PROCESS_STATE.STOPPED
        stateQ.put(self.detectProcessState)
        logging.info("Detect process : finishing")
        print("Detect process : finishing")

    def create_DetectProcess(self):
        # check if Detect Process is running

        # get the last item from the queue. the latest self.detectProcessState value
        state = DETECT_PROCESS_STATE.STOPPED
        while (not self.stateQueue.empty()):
            state = self.stateQueue.get()
        print(f"create_DetectProcess : latest state from queue : {state}")
        if (state == DETECT_PROCESS_STATE.STOPPED):
            self.startDetectProcess = True
            self.detectProcess = Process(
                target=self.detectInImageProcess, args=(self.imageQueue, self.stateQueue,))
            self.detectProcess.start()

    def terminate_DetectProcess(self):
        self.startDetectProcess = False
        self.detectProcess.terminate()
        self.detectProcess.kill()

    def my_print(self, str):
        if (_DEBUG):
            print(str)
