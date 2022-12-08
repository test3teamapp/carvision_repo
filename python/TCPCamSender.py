
import socket
import time
from datetime import datetime
import cv2
#import io
import numpy as np
import threading
import logging
#from turbojpeg import TurboJPEG, TJPF_GRAY, TJSAMP_GRAY
from multiprocessing import Process, Value
from multiprocessing import Queue
import base64
from enum import IntEnum
from bitstring import BitArray

#import signal
#import os
# for converting cv2 image to PIL Image, to feed the detector
from PIL import Image

from CoralDetector import CoralDetector

_SHOULD_DETECT = False
_DEBUG = True

class TCP_STATE(IntEnum):
    DOWN = 1
    LISTENING = 2
    CONNECTED = 3
    CLOSED = 4

class TCPCamSender:
    """Class for spawning and controlling a process for openning a TCP connection for receiving images """

    # variables here are common to all instances of the class #
            

    def __init__(self):
        self.remoteIP = ""
        self.tcpPort = -1
        # see https://docs.python.org/2/library/array.html#module-array
        # for ctypes available for Multiprocessing.Values
        self.startTCP = Value('B',0)
        self.tcpState = Value('i',int(TCP_STATE.DOWN))
        self.tcpProcess = Process()
   
    def setRemoteIPandPort(self,remoteIP, tcpPort):
        self.remoteIP = remoteIP
        self.tcpPort = tcpPort

# https://stackoverflow.com/questions/48024720/python-how-to-check-if-socket-is-still-connected
    def is_socket_closed(self,sock: socket.socket) -> bool:
        try:
            # this will try to read bytes without blocking and also without removing them from buffer (peek only)
            data = sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
            if len(data) == 0:
                return True
        except BlockingIOError:
            return False  # socket is open and reading from it would block
        except ConnectionResetError:
            return True  # socket was closed for some other reason
        except Exception as e:
            logging.exception(
                "unexpected exception when checking if a socket is closed")
            return False
        return False

    def recvSome(self,sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return b'\x00'
            buf += newbuf
            count -= len(newbuf)
        return buf

    def process_TCPServer(self, ipaddr, port):#, stateQueue):

        logging.info("TCP process connecting @ %s:%s ", ipaddr, port)
        self.my_print(f"TCP process connecting @ {ipaddr}:{port} ")

        # If you run an interactive ipython session, and want to use highgui windows, do cv2.startWindowThread() first.
        # In detail: HighGUI is a simplified interface to display images and video from OpenCV code.
        #cv2.startWindowThread()
        #cv2.namedWindow("preview")
        #cv2.moveWindow("preview", 20, 20)

        # TCP socket        
        TCPServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            TCPServerSocket.connect((ipaddr , port))
        except BaseException as err:
            print("TCP port remote connection failed")
            print(f"Error: {err}, {type(err)}")            
            return

        self.my_print(f"TCP server connection established @ {ipaddr}")
        self.tcpState.value = int(TCP_STATE.CONNECTED)

        if (_SHOULD_DETECT):
            # create a detector
            sharedImageQueue = Queue()
            myCoralDetector = CoralDetector(sharedImageQueue)
            myCoralDetector.create_DetectProcess()

        cap = cv2.VideoCapture(1) # camera id
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        numOfFrames = 0
        start = time.time()

        while(self.startTCP.value):
            try:

                #############  --- capture and send image 

                if cap.isOpened():
                    ret, frame = cap.read()
                    if not ret:
                        break
        
                    numOfFrames += 1
                    if (numOfFrames == 1000):
                        now = datetime.now()
                        current_time = now.strftime("%H:%M:%S")
                        fps = 1000 / (time.time() - start)
                        self.my_print(f"camera fps={fps} / time = {current_time}")
                        start = time.time()
                        numOfFrames = 0

                    encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),90]
                    result, imgencode = cv2.imencode('.jpg', frame, encode_param)
                    data = np.array(imgencode)
                    ba1 = bytearray(data.tobytes())
                    lengthInt = len(ba1)
                    #save lenght in a 4byte configuration 
                    ba2 = bytearray(lengthInt.to_bytes(4, byteorder='little'))                                       
                    #self.my_print(f"Sending length of image data : {lengthInt} and then data")
                    TCPServerSocket.sendall(ba2 + ba1)


            except BaseException as err:
                print(f"Unexpected {err}, {type(err)}")
                self.startTCP.value = 0
                break


        #TCPconnection.close()  
        cap.release()     
        TCPServerSocket.close()
        self.startTCP.value = 0
        self.tcpState.value = int(TCP_STATE.DOWN)
        #stateQueue.put(self.tcpState)
        cv2.destroyAllWindows()
        logging.info("TCP process : finishing")
        self.my_print("TCP process : finishing")

    def create_TCPProcess(self, remoteIP, tcpPort):
        self.remoteIP = remoteIP
        self.tcpPort = tcpPort
        # check if TCP Process is running

        # get the last item from the queue. the latest self.tcpState value
        #state = TCP_STATE.DOWN
        #while (not self.queue.empty()):
        #    state = self.queue.get()
        self.my_print(f"create_TCPProcess : state = {self.tcpState.value}")            
        if (self.tcpState.value == int(TCP_STATE.DOWN) or self.tcpState.value == int(TCP_STATE.CLOSED)):
            self.startTCP.value = 1
            self.tcpProcess = Process(
                target=self.process_TCPServer, args=(self.remoteIP, self.tcpPort))
            self.tcpProcess.start()

    def terminate_TCPProcess(self):
        self.startTCP.value = 0
        self.tcpProcess.terminate()
        self.tcpProcess.kill()
        self.tcpState.value = int(TCP_STATE.DOWN)

    def my_print(self,str):
        if(_DEBUG):
            print(str)
    
    
        
