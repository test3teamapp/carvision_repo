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
import os
import time

from common import avg_fps_counter, SVG
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

from multiprocessing import Process, Value


import sys
import asyncio
import platform

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

disconnected_event = False
cardataSpeed = Value('i',0)
cardataRPM = Value('i',0)
cardataG_X = Value('d', 0.0)

def detectCollision(objs, cardataSpeed, cardataRPM, cardataG_X):
    print(objs)
    print(f"speed = {cardataSpeed.Value}, RPM = {cardataRPM.Value},G_x = {cardataG_X.Value}")


def generate_svg(src_size, inference_box, objs, labels, text_lines):
    #print(objs)
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
    return svg.finish()

def detectObjects(cardataSpeed, cardataRPM, cardataG_X):
    isHeadless = False
    default_model_dir = '../all_models'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    top_k = 10
    threshold = 0.3

    print('Loading {} with {} labels.'.format(default_model , default_labels))
    interpreter = make_interpreter(os.path.join(default_model_dir,default_model))
    interpreter.allocate_tensors()
    labels = read_label_file(os.path.join(default_model_dir, default_labels))
    inference_size = input_size(interpreter)

    # Average fps over last 30 frames.
    fps_counter = avg_fps_counter(30)

    def user_callback(input_tensor, src_size, inference_box):
      nonlocal fps_counter
      start_time = time.monotonic()
      run_inference(interpreter, input_tensor)
      # For larger input image sizes, use the edgetpu.classification.engine for better performance
      objs = get_objects(interpreter, threshold)[:top_k]
      end_time = time.monotonic()
      text_lines = [
          'Inference: {:.2f} ms'.format((end_time - start_time) * 1000),
          'FPS: {} fps'.format(round(next(fps_counter))),
      ]
      #print(' '.join(text_lines))

      if (isHeadless):
        return SVG(src_size)
      else:
        return generate_svg(src_size, inference_box, objs, labels, text_lines)    
    
    result = gstreamer.run_pipeline(user_callback,
                                    src_size=(640, 480),
                                    #src_size=(1280,720),
                                    appsink_size=inference_size,
                                    videosrc='/dev/video0',
                                    videofmt='raw',
                                    headless=isHeadless)



def speed_notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    print(f"Speed: {data}")

def rpm_notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    print(f"RPM: {data}")

def gx_notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    print(f"Gx: {data}")

def disconnected_callback(client):
    global disconnected_event
    global cardataSpeed, cardataRPM, cardataG_X
    print("Disconnected callback called!")
    print("Connected:", client.is_connected)
    disconnected_event = True    
    for task in asyncio.all_tasks():
        task.cancel()   
    

async def connectToCarBLE(cardataSpeed, cardataRPM, cardataG_X):
    global disconnected_event
    isClientConnected = False

    address = "8C:AA:B5:89:C7:32"
    speed_char_uuid = "0000dd31-76d9-48e9-aa47-d0538d18f701"
    rpm_char_uuid = "0000dd32-76d9-48e9-aa47-d0538d18f701"
    gx_char_uuid = "0000dd33-76d9-48e9-aa47-d0538d18f701"

    #devices = await BleakScanner.discover()
    #for d in devices:
    #    print(d)
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
    global cardataSpeed, cardataRPM, cardataG_X
    
    asyncio.run(
        connectToCarBLE(cardataSpeed, cardataRPM, cardataG_X)
    )
    #detectObjects(cardataSpeed, cardataRPM, cardataG_X)

if __name__ == '__main__':
    main()
