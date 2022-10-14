# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A demo which runs object detection on camera frames using GStreamer.

Run default object detection:
python3 detect.py

Choose different camera and input encoding
python3 detect.py --videosrc /dev/video1 --videofmt jpeg

TEST_DATA=../all_models
Run face detection model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_face_quant_postprocess_edgetpu.tflite

Run coco model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite \
  --labels ${TEST_DATA}/coco_labels.txt


BLE provided by Bleak

python3 -m pip install bleak

Successfully installed async-timeout-4.0.2 bleak-0.18.1 dbus-fast-1.29.1 typing-extensions-4.4.0


"""
import argparse
import gstreamer
import cv2
import os
import time
from datetime import datetime
import math

from common import avg_fps_counter, SVG
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

from multiprocessing import Process, Value
import threading
import ctypes


import sys
import asyncio
import platform

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

import array  # for conversion of byte array to float

import simpleaudio as sa


disconnected_event = False
cardataSpeed = Value('i',0)
cardataRPM = Value('i',0)
cardataG_X = Value('d', 0.0)
detectionProcessStarted = Value('B',0)
detectProcess = Process()
detectThread = threading.Thread()
p1 = p2 = p3 = p4 = [0,0] #collision box points
widthImage = 640
heightImage = 480
inference_size = (widthImage, heightImage)
default_model_dir = '/home/mendel/coral/examples-camera/all_models'
default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
default_labels = 'coco_labels.txt'
top_k = 10
threshold = 0.3
# PRELOAD the beeb sound for collition detection
beeb_wave_obj = sa.WaveObject.from_wave_file("/home/mendel/sounds/beep1.wav")
# play a sound so that the play object is initialised and can be used latter on 
play_sound_obj : sa.PlayObject

_DEBUG = True
_IS_HEADLESS = False

def detectCollision(objs):
    #print(objs)
    #print(f"speed = {cardataSpeed.value}, RPM = {cardataRPM.value},G_x = {cardataG_X.value}")
    global widthImage, heightImage, inference_size
    global p1, p2, p3, p4
    global beeb_wave_obj, play_sound_obj

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
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        point = [0,0]
        point[0] = int((x1 - x0) / 2) + x0
        point[1] = y1

        #1 check to the right
        intersectsToTheRight = intersect((point[0],point[1]),(widthImage,point[1]),(p3[0],p3[1]),(p4[0],p4[1]))

        if (intersectsToTheRight):
            #2 check to the left
            intersectsToTheLeft = intersect((point[0],point[1]),(0,point[1]),(p1[0],p1[1]),(p2[0],p2[1]))

            if (intersectsToTheLeft):
                print(f"collision with point {point[0]},{point[1]}")
                if (not play_sound_obj.is_playing()):
                    play_sound_obj = beeb_wave_obj.play()


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

    speed = cardataSpeed.value * 5 
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

    p1 = (int(widthImage/4), heightImage)
    movementOfGuidelinesOnXAxis =  int( (speed / gx) - 
                    ((speed / gx) * math.cos(gx)) +
                    (speed * 0.2))
    movementOfGuidelinesOnYAxis = int(((speed / gx) * math.sin(gx)) - (abs(gx)* heightImage/4))
    
    p2 = (p1[0] + movementOfGuidelinesOnXAxis,           
          p1[1] - movementOfGuidelinesOnYAxis)
              
    p3 = (int(widthImage/4) * 3, heightImage)
    p4 = (p3[0] - movementOfGuidelinesOnXAxis , p2[1]) # - (2 * (speed * 0.2))

    # now rotate the lines according to currentXG reading, if there is a significant value 
    if (cardataG_X.value > 0.01 or cardataG_X.value < -0.01):
      updateCollisionBoxByTheta()    

    #print(f"speed = {speed}, gx = {cardataG_X.value}")
    #print (f"p1={p1} p2={p2} p3={p3} p4={p4}")



# we should display the collision box as well
# Call updateCollisionBox() before calling this
def generate_svg(src_size, inference_box, objs, labels, text_lines):
    global p1, p2, p3, p4
    #print(objs)
    #print (f"p1={p1} p2={p2} p3={p3} p4={p4}")
    svg = SVG(src_size)
    src_w, src_h = src_size
    box_x, box_y, box_w, box_h = inference_box
    scale_x, scale_y = src_w / box_w, src_h / box_h

    for y, line in enumerate(text_lines, start=1):
        svg.add_text(10, y * 20, line, 20)
    for obj in objs:
        bbox = obj.bbox
        if not bbox.valid:
            continue
        # Absolute coordinates, input tensor space.
        x, y = bbox.xmin, bbox.ymin
        w, h = bbox.width, bbox.height
        # Subtract boxing offset.
        x, y = x - box_x, y - box_y
        # Scale to source coordinate space.
        x, y, w, h = x * scale_x, y * scale_y, w * scale_x, h * scale_y
        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))
        svg.add_text(x, y - 5, label, 20)
        svg.add_rect(x, y, w, h, 'red', 2)

    # display collision box
    svg.add_line(p1[0], p1[1], p2[0], p2[1])
    svg.add_line(p3[0], p3[1], p4[0], p4[1])
    
    return svg.finish()

def append_objs_to_img(cv2_im, inference_size, objs, labels):
    global p1, p2, p3, p4

    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    
    # display collision box
    cv2.line(cv2_im, (p1[0], p1[1]), (p2[0], p2[1]), (0, 255, 0), 2)
    cv2.line(cv2_im, (p3[0], p3[1]), (p4[0], p4[1]), (0, 255, 0), 2)

    return cv2_im

def detectObjects_cv2():
    global detectionProcessStarted, default_model_dir , default_model 
    global default_labels, top_k, threshold, _IS_HEADLESS,_DEBUG
    global detectProcess, inference_size
    global play_sound_obj, beeb_wave_obj
    # play a sound so that the play object is initialised and can be used latter on
    play_sound_obj = sa.WaveObject.from_wave_file("/home/mendel/sounds/button-8.wav").play()

    try:
        print('Loading {} with {} labels.'.format(default_model , default_labels))
        interpreter = make_interpreter(os.path.join(default_model_dir,default_model))
        interpreter.allocate_tensors()
        labels = read_label_file(os.path.join(default_model_dir, default_labels))
        inference_size = input_size(interpreter)

        cap = cv2.VideoCapture(1)
    
        start = time.time()
        numOfFrames = 0

        while (cap.isOpened() and (detectionProcessStarted.value == 1)):
            ret, frame = cap.read()
            if not ret:
                break
        
            if (_DEBUG):
                numOfFrames += 1
                if (numOfFrames == 100):
                    now = datetime.now()
                    current_time = now.strftime("%H:%M:%S")
                    fps = 100 / (time.time() - start)
                    print(f"fps={fps} / time = {current_time}")
                    start = time.time()
                    numOfFrames = 0

            cv2_im = frame

            cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
            cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
            run_inference(interpreter, cv2_im_rgb.tobytes())
            objs = get_objects(interpreter, threshold)[:top_k]

            updateCollisionBox()
            detectCollision(objs)

            #if (_DEBUG):
                #cv2_im = append_objs_to_img(cv2_im, inference_size, objs, labels)
                #if (isHeadless):
                    #cv2.imwrite("detection_output.jpg", cv2_im)
                #else:            
                    #cv2.imshow('frame', cv2_im)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        print("IF WE REACH THIS POINT --- pipeline interupted. Detection Process ended")
        detectionProcessStarted.value = 0
    except:
        # kill the process.
        # a new RPM > 0 value from the car will triger it again        
        print("Terminating object detection process due to exception")
        detectProcess.terminate()        
        detectionProcessStarted.value = 0


# use only for testing visually the results
# does not work well when put on a new spawned process
# works with thread. but thread is more difficult to terminate.
# more over it, it uses more cpu power - more heat
# for production, use cv2 alternative (cv2 can not display on monitor, 
# due to some funny opencv / qt pluggin not playing nice on coral)

def detectObjects():
    global widthImage, heightImage, inference_size
    global detectionProcessStarted, default_model_dir , default_model 
    global default_labels, top_k, threshold, _IS_HEADLESS,_DEBUG, detectProcess
    global play_sound_obj, beeb_wave_obj

    play_sound_obj = beeb_wave_obj.play()

    print('Loading {} with {} labels.'.format(default_model , default_labels))
    interpreter = make_interpreter(os.path.join(default_model_dir,default_model))
    interpreter.allocate_tensors()
    labels = read_label_file(os.path.join(default_model_dir, default_labels))
    inference_size = input_size(interpreter)

    # Average fps over last 30 frames.
    fps_counter = avg_fps_counter(30)

    def user_callback(input_tensor, src_size, inference_box):
        nonlocal fps_counter
        global detectionProcessStarted
        global _IS_HEADLESS

        if (detectionProcessStarted.value == 0):
            #os.system('sudo reboot')
            detectProcess.terminate()
            return 
        
        start_time = time.monotonic()
        run_inference(interpreter, input_tensor)
        # For larger input image sizes, use the edgetpu.classification.engine for better performance
        objs = get_objects(interpreter, threshold)[:top_k]
        end_time = time.monotonic()
        if (_DEBUG):
            text_lines = [
            'Inference: {:.2f} ms'.format((end_time - start_time) * 1000),
            'FPS: {} fps'.format(round(next(fps_counter))),
            ]
        #print(' '.join(text_lines))

        if (_IS_HEADLESS):
            updateCollisionBox()
            detectCollision(objs)
            return #SVG(src_size)
        else:
            updateCollisionBox()
            detectCollision(objs)
            return generate_svg(src_size, inference_box, objs, labels, text_lines)    
    
    result = gstreamer.run_pipeline(user_callback,
                                    src_size=(widthImage, heightImage),
                                    #src_size=(1280,720),
                                    appsink_size=inference_size,
                                    videosrc='/dev/video0',
                                    videofmt='raw',
                                    headless=_IS_HEADLESS)
    print("IF WE REACH THIS POINT --- pipeline interupted. Detection Process ended")
    detectionProcessStarted.value = 0


def speed_notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    global cardataSpeed
    s = int.from_bytes(data, "little")    
    #print(f"Speed: {s}")
    cardataSpeed.value = s

def rpm_notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    global detectProcess, detectThread, cardataRPM
    rpm = int.from_bytes(data, "little")
    #print(f"RPM: {rpm}")
    cardataRPM.value = rpm
    if (rpm > 0):
        
        if (detectionProcessStarted.value == 0):
            detectProcess = Process(target=detectObjects_cv2, args=())
            detectProcess.start()
            # gstreamer based method does not play weel with processes. using thread
            #detectThread = threading.Thread(target=detectObjects, args=())        
            #detectThread.start()
            detectionProcessStarted.value = 1   

def gx_notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    global cardataG_X
    arr = array.array('f')
    arr.frombytes(data)
    #print(arr)
    #print(f"Gx: {arr[0]}")
    # we need to reverse sign. 
    cardataG_X.value = -1 * arr[0]

def disconnected_callback(client):
    global disconnected_event
    global detectionProcessStarted, detectProcess, detectThread

    print("Disconnected callback called!")
    print("Connected:", client.is_connected)
    disconnected_event = True    
    for task in asyncio.all_tasks():
        task.cancel()
    if (detectionProcessStarted.value == 1):
        print("Terminating object detection process as well...")
        detectProcess.terminate()        
        detectionProcessStarted.value = 0   
    

async def connectToCarBLE():
    global disconnected_event
    isClientConnected = False

    address = "8C:AA:B5:89:C7:32"
    speed_char_uuid = "0000dd31-76d9-48e9-aa47-d0538d18f701"
    rpm_char_uuid = "0000dd32-76d9-48e9-aa47-d0538d18f701"
    gx_char_uuid = "0000dd33-76d9-48e9-aa47-d0538d18f701"

    devices = await BleakScanner.discover()
    for d in devices:
        print(d)
    bleClient = BleakClient(address, disconnected_callback=disconnected_callback)

    while(not isClientConnected):
        try:
            print(f"Connecting..")
            await bleClient.connect()
            if (bleClient.is_connected):
                    isClientConnected = True

        except Exception as e:
            print(f"Exception while trying to connect... {e}")
            time.sleep(10)
            #await asyncio.sleep(10)

    try:
        print(f"Connected: {bleClient.is_connected}")
        if (bleClient.is_connected):
            isClientConnected = True
            await bleClient.start_notify(speed_char_uuid, speed_notification_handler)
            await bleClient.start_notify(rpm_char_uuid, rpm_notification_handler)
            await bleClient.start_notify(gx_char_uuid, gx_notification_handler)                
            while(not disconnected_event):           
                await asyncio.sleep(1)

    except Exception as e:
        print(f"Exception while receiving notifications {e}")
                       

        

def main():    
    asyncio.run(
        connectToCarBLE()
    )    

if __name__ == '__main__':
    main()
