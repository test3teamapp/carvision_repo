
import socket
from time import time
import cv2
#import io
import numpy as np
import threading
import logging
#from turbojpeg import TurboJPEG, TJPF_GRAY, TJSAMP_GRAY
from multiprocessing import Process
import time
from enum import Enum

import signal
import os

from TCPConnectionHandler import TCP_STATE
from TCPConnectionHandler import TCPConnectionHandler

_localIP = "10.132.0.2"  # receive UDP broadcast by using '' as address
_localUDPPort = 20001
_localTCPPort = 20002
_bufferSize = 1024

_DEBUG = True

def thread_UDPServer(ipaddr, port):
    _localIP = extract_ip()
    myTCPConnectionHandler = TCPConnectionHandler(_localIP, _localTCPPort)
    # Create a datagram socket (NOT GREAT FOR RECEIVING IMAGES)
    UDPServerSocket = socket.socket(
        family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Bind to address and port
    UDPServerSocket.bind((ipaddr, _localUDPPort))  # using '' for broacast address
    logging.info("UDP Thread @ %s:%s starting", ipaddr, port)
    my_print("UDP server up and listening")
    # Listen for incoming datagrams

    while(True):
        bytesAddressPair = UDPServerSocket.recvfrom(_bufferSize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        messageLen = len(message)
        clientMsg = "UDP Message :{}".format(message)
        #clientIP  = "UDP Client IP Address:{}".format(address)

        # my_print(clientMsg)
        # my_print(clientIP)
        messageStr = message.decode('utf-8', 'ingore')
        if (messageStr == "tcp"):
            my_print(f"User requested TCP details for connection")
            _localIP = extract_ip()
            print(f"myTCPConnectionHandler.tcpState = {myTCPConnectionHandler.tcpState}")
            # the app wants to connect. Make sure there is no TCP process running and blocking the port
            #if (myTCPConnectionHandler.tcpState == TCP_STATE.CONNECTED):
            #    my_print(
            #        f"TCP server was running. Shutting it down... {myTCPConnectionHandler.startTCP}")
            #    myTCPConnectionHandler.terminate_TCPProcess()
                # setting startTCP to False should exit the while loop of TCP server
                # The process should then join and end
                # wait for a second untill all this is done
                # time.sleep(1) # Sleep for 1 second
            #    my_print(
            #        f"Is TCP server running : {myTCPConnectionHandler.startTCP}")
                # start a new TCP process
            #    my_print(f"Starting a new TCP server process")
            #    myTCPConnectionHandler.create_TCPProcess()
            if (myTCPConnectionHandler.tcpState == TCP_STATE.DOWN or myTCPConnectionHandler.tcpState == TCP_STATE.CLOSED):
                my_print(f"No TCP server was running. Starting a new process")
                myTCPConnectionHandler.create_TCPProcess()
            else:
                my_print(f"TCP server is already listenning for connection")
            # Sending a reply to client
            responceMsg = f"tcp:{_localIP}:{_localTCPPort}"
            UDPServerSocket.sendto(str.encode(responceMsg), address)
        if (messageStr.startswith('size:')):
            parts = messageStr.split(":")
            #latestReceivedSizeOfImage = int(parts[1])
        if (messageStr.startswith('x_rad:')):
            parts = messageStr.split(":")  
            my_print(messageStr)  
        if (messageStr.startswith('speed:')):
            parts = messageStr.split(":")  
            my_print(messageStr)        
                        

    UDPServerSocket.close()
    logging.info("UDP Thread : finishing")


def receiveSignal(signalNumber, frame):
    # output current process id
    my_print(f"receiveSignal @ PID : {os.getpid()}")
    my_print(f"receiveSignal : {signalNumber}")
    return

def extract_ip():
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

def my_print(str):
    if(_DEBUG):
        print(str)

##### our entry point of the program #######
if __name__ == '__main__':

    _localIP = extract_ip()
    my_print(f"host ip = {_localIP}")

    if (_DEBUG):
        my_print(f"__main__ @ PID : {os.getpid()}")
        # signal listener
        # register the signals to be caught
        signal.signal(signal.SIGCHLD, receiveSignal)
        signal.signal(signal.SIGHUP, receiveSignal)
        signal.signal(signal.SIGINT, receiveSignal)
        signal.signal(signal.SIGQUIT, receiveSignal)
        signal.signal(signal.SIGILL, receiveSignal)
        signal.signal(signal.SIGTRAP, receiveSignal)
        signal.signal(signal.SIGABRT, receiveSignal)
        signal.signal(signal.SIGBUS, receiveSignal)
        signal.signal(signal.SIGFPE, receiveSignal)
        signal.signal(signal.SIGKILL, receiveSignal)
        signal.signal(signal.SIGUSR1, receiveSignal)
        signal.signal(signal.SIGSEGV, receiveSignal)
        signal.signal(signal.SIGUSR2, receiveSignal)
        signal.signal(signal.SIGPIPE, receiveSignal)
        signal.signal(signal.SIGALRM, receiveSignal)
        signal.signal(signal.SIGTERM, receiveSignal)

    logging.info(f"Main    : starting  UDP Server @ {_localIP}:{_localUDPPort}")
    udpServerThread = threading.Thread(target=thread_UDPServer, args=(
        '', _localUDPPort))  # using '' for ip adress to receive broadcast packets
    udpServerThread.start()
    udpServerThread.join()
