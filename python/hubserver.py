#!/usr/bin/env python3

import socket
from PIL import Image
import struct
from io import StringIO, BytesIO
import base64

HOST = '10.42.0.1'  # Standard loopback interface address (localhost)
PORT = 8080        # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)        
        while True:
            typeOfData = 0
            sizeOfData = 0
            buf = ''
            while len(buf)<4:
                data = conn.recv(4-len(buf))
                buf += data.decode('utf-8')

            
            print ('indicator of data type (string buffer): ', buf)
            #typeOfData = struct.unpack('i', buf)
            #print ('indicator of data type : ', typeOfData)
            buf = ''
            while len(buf)<4:
                data = conn.recv(4-len(buf))
                buf += data.decode('utf-8') 
            
            print ('Size of data (in bytes) (string buffer): ', buf)
            sizeOfData = struct.unpack('!i', buf.encode())
            print ('Size of data (in bytes)  : ', sizeOfData)
            buf = ''
            if (typeOfData == 105 and sizeOfData > 0):
                data = conn.recv(sizeOfData)
                buf += base64.b64decode(data.decode('utf-8'))                 
                im = Image.open(StringIO(buf))
                im.show()    

            # data is coming from Class DataOutputStream
            # with .writeChar() -> Writes a char to the underlying output stream as a 2-byte value, high byte first.
            # .writeInt() -> Writes an int to the underlying output stream as four bytes, high byte first.
            # and write(byte[] b, int off, int len) -> Writes len bytes from the specified byte array starting at offset off to the underlying output stream.
            # 307200 is the raw YUV420 image data from samsung j4+ ,
            # plus 1 byte for indicator - 'i' for image, 'd' for other data , 
            # plus 4 bytes for length of bytearray containg the data
            #print('indicator of data type : ', int.from_bytes(data[0:4],byteorder='big'))
                           
            if not conn:
                conn, addr = s.accept()
            #conn.sendall(data)
