
import socket
from time import time
import cv2
#import io
import numpy as np
import threading
import logging
#from turbojpeg import TurboJPEG, TJPF_GRAY, TJSAMP_GRAY
from multiprocessing import Process
from multiprocessing import Queue
#import time
from enum import Enum

#import signal
#import os
# for converting cv2 image to PIL Image, to feed the detector
from PIL import Image

from CoralDetector import CoralDetector


class TCP_STATE(Enum):
    DOWN = 1
    LISTENING = 2
    CONNECTED = 3
    CLOSED = 4

class TCPConnectionHandler:
    """Class for spawning and controlling a process for openning a TCP connection for receiving images """

    # variables here are common to all instances of the class #
            

    def __init__(self, localIP, tcpPort):
        self.localIP = localIP
        self.tcpPort = tcpPort
        self.startTCP = False
        self.tcpState = TCP_STATE.DOWN
        self.tcpProcess = Process()
        # created an unbounded queue for accessing the state variable from the new process
        self.queue = Queue()
        self.queue.put(self.tcpState)
   
    def setIPandPort(self,localIP, tcpPort):
        self.localIP = localIP
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

    def process_TCPServer(self, ipaddr, port, stateQueue):

        logging.info("TCP process @ %s:%s starting", ipaddr, port)

        # If you run an interactive ipython session, and want to use highgui windows, do cv2.startWindowThread() first.
        # In detail: HighGUI is a simplified interface to display images and video from OpenCV code.
        #cv2.startWindowThread()
        #cv2.namedWindow("preview")
        #cv2.moveWindow("preview", 20, 20)

        # Use default library installation
        #jpeg = TurboJPEG()

        # TCP socket        
        TCPServerSocket = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_STREAM)
        try:
            TCPServerSocket.bind((self.localIP, self.tcpPort))
        except BaseException as err:
            print("TCP port binding failed")
            print(f"Error: {err}, {type(err)}")            
            return
        # wait
        print("TCP server up and listening")
        self.tcpState = TCP_STATE.LISTENING
        # add an item to the queue
        stateQueue.put(self.tcpState)
        TCPServerSocket.listen()
        # accepts TCP connection        
        TCPconnection, addr = TCPServerSocket.accept()
        print(f"TCP server accepted connection from {addr}")
        self.tcpState = TCP_STATE.CONNECTED
        stateQueue.put(self.tcpState)

        # create a detector
        sharedImageQueue = Queue()
        myCoralDetector = CoralDetector(sharedImageQueue)
        myCoralDetector.create_DetectProcess()

        while(self.startTCP):
            try:
                lengthAsBytes = self.recvSome(TCPconnection, 4)
                intLength = int.from_bytes(lengthAsBytes, "little")
                #print(f"image size in bytes: {intLength}")
                if (intLength > 0):
                    imageData = self.recvSome(TCPconnection, intLength)

                    # arbitrary logical number for a jpg image of resonalble size
                    if (len(imageData) > 1000):
                        # display image
                        #buffer = io.BytesIO(message)
                        # buffer.seek(0)
                        #inp = np.asarray(bytearray(message), dtype=np.uint8)
                        i = cv2.imdecode(np.frombuffer(
                            imageData, dtype=np.uint8), cv2.IMREAD_COLOR)
                        #i = jpeg.decode(message)
                        #cv2.imshow("preview", i)
                        # cv2.waitKey(0)
                         # WE NEED TO TRANSFORM THE CV2 image TO A PIL Image
                        #img = cv2.cvtColor(i, cv2.COLOR_BGR2RGB)
                        #im_pil = Image.fromarray(img)                       
                        #myCoralDetector.detectInImageProcess(im_pil)
                        ## put in image que for the detector process to get and process
                        
                        #### EDGE TPU might not be able to process all frames we sent 
                        #### queing them in a FIFO way is not usefull for our purpose
                        #### WE NEED THE TPU TO PROCESS ALWAYS THE LATEST IMAGE (camera frame)
                        #### so...
                        if (sharedImageQueue.empty()):
                            sharedImageQueue.put(i)
                        ### else , skip this frame. the TPU has not processed the previous one
                    
                else:
                    # if length of image buffer is 0, check if connection is closed
                    if (self.is_socket_closed(TCPconnection)):
                        self.startTCP = False
                        self.tcpState = TCP_STATE.CLOSED
                        stateQueue.put(self.tcpState)
                        myCoralDetector.terminate_DetectProcess()
                        break

            except BaseException as err:
                print(f"Unexpected {err}, {type(err)}")

        TCPconnection.close()       
        TCPServerSocket.close()
        self.startTCP = False
        self.tcpState = TCP_STATE.DOWN
        stateQueue.put(self.tcpState)
        cv2.destroyAllWindows()
        logging.info("TCP process : finishing")
        print("TCP process : finishing")

    def create_TCPProcess(self):
        # check if TCP Process is running

        # get the last item from the queue. the latest self.tcpState value
        state = TCP_STATE.DOWN
        while (not self.queue.empty()):
            state = self.queue.get()
        print(f"create_TCPProcess : latest tcp state from queue : {state}")            
        if (state == TCP_STATE.DOWN or state == TCP_STATE.CLOSED):
            self.startTCP = True
            self.tcpProcess = Process(
                target=self.process_TCPServer, args=(self.localIP, self.tcpPort,self.queue))
            self.tcpProcess.start()

    def terminate_TCPProcess(self):
        self.startTCP = False
        self.tcpProcess.terminate()
        self.tcpProcess.kill()
    
        
