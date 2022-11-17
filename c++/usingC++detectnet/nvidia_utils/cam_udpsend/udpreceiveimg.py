import socket
import cv2
import numpy
import time


UDP_IP = '192.168.1.7'
#UDP_IP = '127.0.0.1'
UDP_PORT = 5001

#Create socket here
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((UDP_IP, UDP_PORT))

while (1):
    #read image data here
    print("waiting for data")
    #tempFile = s.makefile('rb')
    floatData,addr = s.recvfrom(3686400) # image size = 1280 x 720  * sizeof(float)
    #stringData,addr = s.recvfrom(8) # image size = 1280 x 720  * sizeof(float)
    #floatData = tempFile.read(3686400)
    print("received data: ")# + str(stringData))
    #Convert to opencv image format
    data = numpy.frombuffer(floatData, dtype=float)
    decimg=cv2.imdecode(data,0)

    #Show image to check if it work correctly

    cv2.imshow('SERVER-RECEIVED',decimg)

#cv2.waitKey(3000)


s.close()

cv2.destroyAllWindows()