
import math
import matplotlib.pyplot as plt
import serial
import io
import time

twoPI = math.pi * 2.0
PI = math.pi

origin1 = [200,0]
origin2 = [1080,0]
speed = 30
originalTheta = 50.0
currentTheta = originalTheta

def get_points(originX, originY, distance, theta, isForLeftLineOfPerspective, isTurning, isTurningLeft):
    # Any point (x,y) on the path of the circle is x=r∗sin(θ),y=r∗cos(θ)
    #The point (0,r) ends up at x=rsinθ, y=rcosθ
    #In general, suppose that you are rotating about the origin clockwise through an angle θ
    #Then the point (s,t) ends up at (u,v) where
    #u=scosθ+tsinθ and v=−ssinθ+tcosθ.

    
    
    theta = abs(theta)

    if (theta > 80.0):
        theta = 80.0
    
    if isTurning:
        #print((theta, isTurning, isTurningLeft))
        if isTurningLeft:
            if (isForLeftLineOfPerspective): 
                theta = 90 - originalTheta + theta
                # reduce R (distance from center oa a hypothetical circle)
                # on the wheel that is in on the "inside" of the turning angle
                distance = distance - ( distance * (math.abs(theta - originalTheta) / originalTheta * 100) / 100)
            else:
                theta = 90 + originalTheta + theta
        else:
            if (isForLeftLineOfPerspective): 
                theta = 90 - originalTheta - theta
            else:
                theta = 90 + originalTheta - theta
                distance = distance - ( distance * (math.abs(theta - originalTheta) / originalTheta * 100) / 100)
        
        print((distance,theta))

    else:
        if (isForLeftLineOfPerspective): 
            theta = 90 - originalTheta
        else:
            theta = 90 + originalTheta
    

    x = distance * math.sin(math.radians(theta)) #originX * math.cos(theta) + originY * math.sin(theta) #
    y = abs(distance * math.cos(math.radians(theta))) #-originX * math.sin(theta) + originY * math.cos(theta) #

    if (isForLeftLineOfPerspective):
        x = x + originX
    else:
        x = originX - x

         
    newPoint = [x,y]
    return newPoint

def speedToDepthYPixels(speed):
    meters = speed * 5 / 12
    pixelsY = (-10.3 * meters) + 676
    return pixelsY


ser = serial.Serial('COM9', 115200)  # open serial port
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

figure, ax = plt.subplots()
plt.ion()
#plt.xticks(range(0, 1280))
#plt.yticks(range(0, 720))

newPoint1 = get_points(origin1[0],origin1[1], speedToDepthYPixels(speed), originalTheta,True, False, False)
newPoint2 = get_points(origin2[0],origin2[1], speedToDepthYPixels(speed), originalTheta, False, False, False)

plot1, = ax.plot((origin1[0],newPoint1[0]),(origin1[1],newPoint1[1]))
plot2, = ax.plot((origin2[0],newPoint2[0]),(origin2[1],newPoint2[1]))
plot3, = ax.plot((newPoint1[0],newPoint2[0]),(newPoint1[1],newPoint2[1]))

plt.show()
serialbuffer = ''
ticksWithoutG = 0
while (1):
    theta = originalTheta
    isTurning = False
    isTurningLeft = False
    #line = ser.readline().strip()
    try:
        if (ser.in_waiting < 1):
            continue
        dataready = False
        line = ser.read(1)          
        line = line.decode()
        #print(line)
        if (line == 's' or line == 'g'):
            serialbuffer = line
        elif line == '\r':
            dataready = True
            #print(serialbuffer)
        else:
            serialbuffer = serialbuffer + line
        
        if dataready:
            line = serialbuffer
            values = line.split('=')
            #print(values)
            if (values[0] == 's'):
                speed = int(values[1])
                ticksWithoutG = ticksWithoutG + 1
                if ticksWithoutG > 3:
                    currentTheta = originalTheta
                    isTurning = False
                    isTurningLeft = False 
            elif (values[0] == 'gL'):
                ticksWithoutG = 0
                theta = float(values[1]) 
                currentTheta = currentTheta - theta  
                isTurning = True
                isTurningLeft = True                    
            elif (values[0] == 'gR'):     # this comes in negative values           
                ticksWithoutG = 0
                theta = abs(float(values[1]))
                currentTheta = currentTheta + theta
                isTurning = True
                isTurningLeft = False                     
    
            theta = currentTheta   
            # test
            speed = 60     
        
            newPoint1 = get_points(origin1[0],origin1[1], speedToDepthYPixels(speed), theta,True, isTurning, isTurningLeft)
            newPoint2 = get_points(origin2[0],origin2[1], speedToDepthYPixels(speed), theta, False, isTurning, isTurningLeft)
            plot1.set_xdata((origin1[0],newPoint1[0]))
            plot1.set_ydata((origin1[1],newPoint1[1]))
            plot2.set_xdata((origin2[0],newPoint2[0]))
            plot2.set_ydata((origin2[1],newPoint2[1]))
            plot3.set_xdata((newPoint1[0],newPoint2[0]))
            plot3.set_ydata((newPoint1[1],newPoint2[1]))
            figure.canvas.draw()
            figure.canvas.flush_events()
            time.sleep(0.1)
            
    except:
        #do nothing, carry on
        pass



