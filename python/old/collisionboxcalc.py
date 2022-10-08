import matplotlib.pyplot as plt
import numpy as np
import random

width = 600
height = 400
speed = 10
wheelPosition = 0

x1low = width / 6
y1low = 1
x2low = x1low * 5
y2low = y1low

x1top = (x1low + speed)
y1top = y1low + speed
x2top = (x2low - speed)
y2top = y1top

while True:
    speed = random.randint(10, 100)
    wheelPosition = random.randrange(-50,50,5)
    print("speed is", speed, ", wheel is @ position",wheelPosition,"\n")
    
    x1low = width / 6
    y1low = 1
    x2low = x1low * 5
    y2low = y1low

    x1top = (x1low + speed) + (wheelPosition * 2)
    y1top = y1low + speed
    x2top = (x2low - speed) + (wheelPosition * 2)
    y2top = y1top
    
    
    plt.plot([x1low, x2low], [y1low, y2low])
    plt.plot([x1low, x1top], [y1low, y1top])
    plt.plot([x2low, x2top], [y2low, y2top])
    plt.plot([x1top, x2top], [y1top, y2top])

    plt.show()
