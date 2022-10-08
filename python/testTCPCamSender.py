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

#import signal
#import os
# for converting cv2 image to PIL Image, to feed the detector
from PIL import Image



def recvSome(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return b'\x00'
        buf += newbuf
        count -= len(newbuf)
    return buf

def process_TCPServer(ipaddr, port):

    # TCP socket
    TCPServerSocket = socket.socket(
        family=socket.AF_INET, type=socket.SOCK_STREAM)
    try:
        TCPServerSocket.bind((ipaddr, port))
    except BaseException as err:
        print("TCP port binding failed")
        print(f"Error: {err}, {type(err)}")
        return
    # wait
    print("TCP server up and listening")
    TCPServerSocket.listen()
    # accepts TCP connection
    TCPconnection, addr = TCPServerSocket.accept()
    print(f"TCP server accepted connection from {addr}")

    while(True):
        try:
            lengthAsBytes = recvSome(TCPconnection, 4)
            intLength = int.from_bytes(lengthAsBytes, "little")
            #print(f"image size in bytes: {intLength}")
            if (intLength > 0):
                
                imageData = recvSome(TCPconnection, intLength)
                #print(f" received image data of length {len(imageData)}")
                # arbitrary logical number for a jpg image of resonalble size
                if (len(imageData) > 1000):
                    i = cv2.imdecode(np.frombuffer(
                        imageData, dtype=np.uint8), cv2.IMREAD_COLOR)
                    #print(f" image decoded ")
                    cv2.imshow('server frame', i)
                    cv2.waitKey(1)

                else:
                    break

        except BaseException as err:
            print(f"Unexpected {err}, {type(err)}")
            break

    TCPconnection.close()
    TCPServerSocket.close()
    cv2.destroyAllWindows()
    print("TCP process : finishing")



def process_TCPClient(ipaddr, port):#, stateQueue):

    # TCP socket
    TCPServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        TCPServerSocket.connect((ipaddr , port))
    except BaseException as err:
        print("TCP port remote connection failed")
        print(f"Error: {err}, {type(err)}")
        return

    print(f"TCP server connection established @ {ipaddr}")

    cap = cv2.VideoCapture(0) # camera id
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 315)
    numOfFrames = 0
    start = time.time()

    while(1):
        try:
            #time.sleep(2)

            #############  --- capture and send image

            if cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                numOfFrames += 1
                if (numOfFrames == 30):
                    now = datetime.now()
                    current_time = now.strftime("%H:%M:%S")
                    fps = 30 / (time.time() - start)
                    print(f"camera fps={fps} / time = {current_time}")
                    start = time.time()
                    numOfFrames = 0

                #cv2_im = frame
                #cv2.imshow('client frame', frame)
                #cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
                encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),90]
                result, imgencode = cv2.imencode('.jpg', frame, encode_param)
                #cv2.imshow('client frame', imgencode)
                data = np.array(imgencode)
                ba1 = bytearray(data.tobytes())
                #byteData = data.tobytes() #.tostring()# base64.b64encode(data)
                #length = str(len(stringData))
                lengthInt = len(ba1)
                ba2 = bytearray(lengthInt.to_bytes(4, byteorder='little'))
                #save lenght in a 4byte configuration
                #bytesTosent = BitArray(uint=lengthInt, length=32)
                #aad the bytes of the image
                #bytesTosent = bytesTosent + BitArray(byteData)
                #print(f"Sending length of image data : {lengthInt} and then data")
                #print(bytesTosent.tobytes())
                #print(lengthInt.to_bytes(4, byteorder='little'))
                TCPServerSocket.sendall(ba2 + ba1)
                #TCPServerSocket.sendall(byteData)
                #TCPServerSocket.sendall(length.encode('utf-8').ljust(4))
                #TCPServerSocket.sendall(stringData)
                #TCPServerSocket.send(stime.encode('utf-8').ljust(64))
                #cv2.waitKey(1)

        except BaseException as err:
            print(f"Unexpected {err}, {type(err)}")
            break


    #TCPconnection.close()
    cap.release()
    TCPServerSocket.close()
    cv2.destroyAllWindows()
    logging.info("TCP process : finishing")
    print("TCP process : finishing")


##### our entry point of the program #######
if __name__ == '__main__':
    localIP     = '127.0.0.1'#"192.168.1.151" # receive UDP broadcast by using '' as address
    localTCPPort = 20003

    serverThread = threading.Thread(target=process_TCPServer, args=(localIP,localTCPPort))
    clientThread = threading.Thread(target=process_TCPClient, args=(localIP,localTCPPort))
    serverThread.start()
    time.sleep(2)
    clientThread.start()
    serverThread.join()
    clientThread.join()
