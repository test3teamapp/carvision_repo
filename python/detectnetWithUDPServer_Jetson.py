#!/usr/bin/python3
#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import jetson.inference
import jetson.utils

import argparse
import sys
from multiprocessing import Process, Value
import time
import socket
import logging
import math
import ipaddress
import netifaces as neti

# for buzzer connected on pin12
import RPi.GPIO as GPIO

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

#is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

print("Argument List:", str(sys.argv))
print("opt:", opt)
_DEBUG = False
_FPSINFO = False
_IS_HEADLESS = True
#set this accordingly
is_headless =  ["--headless"]
#is_headless =  [""]

_localIP = "10.132.0.2"  # receive UDP broadcast by using '' as local address
_broadcastIP = "10.132.0.255"#for sending to everybody. These IPs wiil be configured on start up
_localUDPPort = 20001
_bufferSize = 1024
cardataSpeed = Value('i',0)
cardataRPM = Value('i',0)
cardataG_X = Value('d', 0.0)
udprocessStarted = Value('B',0)
carEnginStarted = Value('B',0)
alarmFromDetectionRaised = Value('B',0) # for raising alarm to send a UDP msg. SLOW 
udpProcess = Process()
p1 = p2 = p3 = p4 = [0,0] #collision box points

#--input-width=1280
#--input-height=720
#--threshold=0.3
#--headless
#--log-level=error

widthImage = 1280
heightImage = 720
inference_size = (widthImage, heightImage)
threshold = 0.3
# from coco labels we care aobut objects:
# 0  person
# 1  bicycle
# 2  car
# 3  motorcycle
# 5  bus
# 7  truck
# 17  dog
roadObstacles_cocoids = [0,1,2,3,5,7]

# Pin Definitions
buzzer_output_pin = 18  # BCM pin 18, BOARD pin 12
# Pin Setup:
GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
# set pin as an output pin with optional initial state of HIGH
GPIO.setup(buzzer_output_pin, GPIO.OUT, initial=GPIO.LOW)

### test
#exit()

def detectCollision(detections):
    #print(detections)
    #print(f"speed = {cardataSpeed.value}, RPM = {cardataRPM.value},G_x = {cardataG_X.value}")
    global widthImage, heightImage, inference_size, roadObstacles_cocoids
    global p1, p2, p3, p4#, buzzer
    global alarmFromDetectionRaised
    global buzzer_output_pin
    #global beeb_wave_obj, play_sound_obj

    def ccw(A,B,C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    # Return true if line segments AB and CD intersect
    def intersect(A,B,C,D):
        return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


    #print(f"checking point {point[0]},{point[1]}")
    # from the point we check, we draw a line to the most right part of the screen
    # if this line intersect with the right line of the collision box
    # it COULD be within the collision box
    # IF so, we draw a second line to the most left part of the screen
    # and IF that line intersect with the left line of the collision box too
    # THEN the point is within the boundary of the collision box    
  
    scale_x, scale_y = widthImage / inference_size[0], heightImage / inference_size[1]
    for obj in detections:
        # we only care about detecting relative "road obstacles"
        # from coco labels we care aobut objects:
        # 0  person
        # 1  bicycle
        # 2  car
        # 3  motorcycle
        # 5  bus
        # 7  truck
        # 17  dog // remove

        if (obj.ClassID in roadObstacles_cocoids):
            point = [0,0]
            point[0] = int(((obj.Right - obj.Left) / 2) + obj.Left)
            point[1] = int(obj.Bottom)

            #1 check to the right
            intersectsToTheRight = intersect((point[0],point[1]),(widthImage,point[1]),(p3[0],p3[1]),(p4[0],p4[1]))

            if (intersectsToTheRight):
                #2 check to the left
                intersectsToTheLeft = intersect((point[0],point[1]),(0,point[1]),(p1[0],p1[1]),(p2[0],p2[1]))

                if (intersectsToTheLeft):
                    if (_DEBUG):
                        print(f"collision with point {point[0]},{point[1]}")
                    # raise alarm to send UDP msg. IT IS SLOW
                    # alarmFromDetectionRaised.value = 1
                    # buzzer
                    GPIO.output(buzzer_output_pin, GPIO.HIGH)


def updateCollisionBoxByTheta():
    global cardataG_X
    global p1, p2, p3, p4

    #### avoid division by 0, and normalise values withn -0.8 and 0.8 
    if (cardataG_X.value == 0.0):
      cardataG_X.value = 0.001
    elif (cardataG_X.value > 16.0):
      cardataG_X.value = 16.0
    elif (cardataG_X.value < -16.0):
      cardataG_X.value = -16.0

    gx = cardataG_X.value / 20  

    newP2 = [0,0]
    newP4 = [0,0]
    #new X of top point for left line of collision area
    newP2[0] = int(math.cos(gx) * (p2[0] - p1[0]) - (math.sin(gx) * (p2[1] - p1[1])) + p1[0])
    #new Y of top point for left line of collision area
    newP2[1] = int(math.sin(gx) * (p2[0] - p1[0]) + (math.cos(gx) * (p2[1] - p1[1])) + p1[1])
    p2 = newP2
    #new X of top point for left line of collision area
    newP4[0] = int(math.cos(gx) * (p4[0] - p3[0]) - (math.sin(gx) * (p4[1] - p3[1])) + p3[0])
    #new Y of top point for left line of collision area
    newP4[1] = int(math.sin(gx) * (p4[0] - p3[0]) + (math.cos(gx) * (p4[1] - p3[1])) + p3[1])
    p4 = newP4


def updateCollisionBox():
    global cardataG_X
    global cardataSpeed
    global p1, p2, p3, p4
    global widthImage
    global heightImage

    #print(f"updateCollisionBox with xG = {cardataG_X.value}, speed = {currentSpeed.value}")
    

    ### Trial and error. Scaling 0-100 kmh to 0 to Height of image
    if (cardataSpeed.value > 100):
      cardataSpeed.value = 100

    speed = cardataSpeed.value
    if (cardataSpeed.value < 10):
        speed = cardataSpeed.value * 12
    elif (cardataSpeed.value < 20):
        speed = cardataSpeed.value * 11
    elif (cardataSpeed.value < 30):
        speed = cardataSpeed.value * 10
    elif (cardataSpeed.value < 40):
        speed = cardataSpeed.value * 9
    elif (cardataSpeed.value < 50):
        speed = cardataSpeed.value * 8
    elif (cardataSpeed.value < 60):
        speed = cardataSpeed.value * 7
    elif (cardataSpeed.value < 70):
        speed = cardataSpeed.value * 5
    elif (cardataSpeed.value < 80):
        speed = cardataSpeed.value * 3


    #first calculate the length / height of the coalision area, using 0.1 as the default value for Gx accelaration
    gx = 0.001

  
    ### p1 - lower Left of collision box
    ### p2 - upper Left
    ### p3 - lower Right
    ### p4 - upper Right

    # we have to take under consideration that
    # the points are in the upside down coordinates system of the 
    # image processed, 
    # where 0.0 is 0,imageHeight and imageWidth,imageHeight is imageWidth,0

    p1 = (int(widthImage/5), heightImage)
    movementOfGuidelinesOnXAxis =  int( (speed / gx) - 
                    ((speed / gx) * math.cos(gx)) +
                    (speed * 0.2))
    movementOfGuidelinesOnYAxis = int(((speed / gx) * math.sin(gx)) - (abs(gx)* heightImage/5))
    
    p2 = (p1[0] + movementOfGuidelinesOnXAxis,           
          p1[1] - movementOfGuidelinesOnYAxis)
              
    p3 = (int(widthImage/5) * 4, heightImage)
    p4 = (p3[0] - movementOfGuidelinesOnXAxis , p2[1]) # - (2 * (speed * 0.2))

    # now rotate the lines according to currentXG reading, if there is a significant value 
    if (cardataG_X.value > 0.01 or cardataG_X.value < -0.01):
      updateCollisionBoxByTheta()    

    #print(f"speed = {speed}, gx = {cardataG_X.value}")
    #print (f"p1={p1} p2={p2} p3={p3} p4={p4}")

def process_UDPServer(ipaddr, port):
    # Create a datagram socket (NOT GREAT FOR RECEIVING IMAGES)
    UDPServerSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    #allow broadcast sending
    UDPServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # Bind to address and port
    UDPServerSocket.bind((ipaddr, _localUDPPort))  # using '' for broacast address
    logging.info("UDP Thread @ %s:%s starting", ipaddr, port)
    my_print("UDP server up and listening")
    # Listen for incoming datagrams

    while(True):

        #if we need to sent alert, do it now
        if (alarmFromDetectionRaised.value):
            print("Sending UDP alert msg from detection process")
            responceMsg = f"alert:"
            UDPServerSocket.sendto(str.encode(responceMsg), (_broadcastIP, _localUDPPort))
            alarmFromDetectionRaised.value = 0
        # now try to receive
        bytesAddressPair = UDPServerSocket.recvfrom(_bufferSize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        messageLen = len(message)
        #clientMsg = "UDP Message :{}".format(message)
        #clientIP  = "UDP Client IP Address:{}".format(address)

        # my_print(clientMsg)
        # my_print(clientIP)
        messageStr = message.decode('utf-8', 'ingore')
        
        if (messageStr.startswith('rpm:')):
            parts = messageStr.split(":")
            rpm = int(parts[1])
            cardataRPM.value = rpm
            if (rpm > 0):
                if (carEnginStarted.value == 0):
                    carEnginStarted.value = 1                          
            else:
                # stop the process if the car notifies us that the engine is stopped
                if (carEnginStarted.value == 1):
                    print("Car reported 0 RPM. Terminating object detection process")
                    carEnginStarted.value = 0      
                       

        if (messageStr.startswith('speed:')):
            parts = messageStr.split(":")
            cardataSpeed.value = int(parts[1])
        if (messageStr.startswith('gx:')): # it could be both e.g gx:1.4 gx:-1.2
            parts = messageStr.split(":")
            cardataG_X.value = float(parts[1])

    UDPServerSocket.close()
    logging.info("UDP Thread : finishing")

def extract_ip_old():
    interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
    allips = [ip[-1][0] for ip in interfaces]

    for interface in interfaces:
        print(f'found interface {interface}')
    for ip in allips:
        print(f'found ip {ip}')
                
    st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        st.connect(('10.255.255.255', 1))
        IP = st.getsockname()[0]
    except Exception:
        my_print(f"extract_ip : Exception : {Exception}")
        IP = socket.gethostbyname(socket.gethostname())
    finally:
        st.close()
    return IP

def extract_ips():  
    global _localIP, _broadcastIP
    _localIP = neti.ifaddresses('wlan0')[neti.AF_INET][0]['addr']
    _broadcastIP = neti.ifaddresses('wlan0')[neti.AF_INET][0]['broadcast']

def my_print(str):
    if(_DEBUG):
        print(str)        

extract_ips()
#_localIP = extract_ip()
#MASK = '255.255.255.0'
#host = ipaddress.IPv4Address(_localIP)
#net = ipaddress.IPv4Network(_localIP + '/' + MASK, False)
if (_DEBUG):
    print('IP:', _localIP)
#print('Mask:', MASK)
#print('Broadcast:', net.broadcast_address)
#_broadcastIP = str(net.broadcast_address)
if (_DEBUG):
    print('Broadcast:', _broadcastIP)

logging.info(f"Main    : starting  UDP Server @ {_localIP}:{_localUDPPort}")
udpProcess = Process(target=process_UDPServer, args=('', _localUDPPort))  # using '' for ip adress to receive broadcast packets
udprocessStarted.value = 1
udpProcess.start()  

#### test
#carEnginStarted.value = 1
#cardataSpeed.value = 30
#cardataG_X.value = 0.1

# ----  WAIT UNTIL RPMS ARE > 0 ------
while(carEnginStarted.value == 0):
	time.sleep(1)


# ----- carEnginStarted.value will be updated in the UDP process and we will continue

if (not _IS_HEADLESS):
	# create video output object 
	output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)
	
# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

# create video sources
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)


# buzzer 2 short bips to notify for starting detection
GPIO.output(buzzer_output_pin, GPIO.HIGH)
time.sleep(0.2)
GPIO.output(buzzer_output_pin, GPIO.LOW)
time.sleep(0.2)
GPIO.output(buzzer_output_pin, GPIO.HIGH)
time.sleep(0.2)
GPIO.output(buzzer_output_pin, GPIO.LOW)
# process frames until the car rpm goes to 0
while(carEnginStarted.value):
    # capture the next image
    img = input.Capture()
    # # detect objects in the image (with overlay)
    if (not _IS_HEADLESS):
        net.SetCollisionLines(x1=float(p1[0]), y1=float(p1[1]), x2=float(p2[0]), y2=float(p2[1]), x3=float(p3[0]), y3=float(p3[1]), x4=float(p4[0]), y4=float(p4[1]))
    detections = net.Detect(img, overlay=opt.overlay)
    #if (_DEBUG):
    #print the detections
    #print("detected {:d} objects in image".format(len(detections)))

    #for detection in detections:
    #print(detections)

    #reset buzzer and alerts
    alarmFromDetectionRaised.value = 0
    GPIO.output(buzzer_output_pin, GPIO.LOW)
    
    if (len(detections) > 0):
        updateCollisionBox()        
        detectCollision(detections)

    if (not _IS_HEADLESS):
        # render the image
        output.Render(img)
        # update the title bar
        output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))       
    

	# print out performance info
	#net.PrintProfilerTimes()

	# exit on input/output EOS\
    if not input.IsStreaming():
        break

if input.IsStreaming():
    input.Close()

if (not _IS_HEADLESS):
    #destroy window
    output.Close()
        
udpProcess.terminate()
#overlayProcess.terminate()
