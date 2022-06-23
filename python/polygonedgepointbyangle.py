import math
import matplotlib.pyplot as plt

twoPI = math.pi * 2.0
PI = math.pi

def get_points(width, height, theta):
    theta %= twoPI

    aa = width
    bb = height

    rectAtan = math.atan2(bb,aa)
    tanTheta = math.tan(theta)

    xFactor = 1
    yFactor = 1
    
    # determine regions
    if theta > twoPI-rectAtan or theta <= rectAtan:
        region = 1
    elif theta > rectAtan and theta <= PI-rectAtan:
        region = 2

    elif theta > PI - rectAtan and theta <= PI + rectAtan:
        region = 3
        xFactor = -1
        yFactor = -1
    elif theta > PI + rectAtan and theta < twoPI - rectAtan:
        region = 4
        xFactor = -1
        yFactor = -1
    else:
        print(f"region assign failed : {theta}")
        raise
    
    # print(region, xFactor, yFactor)
    edgePoint = [0,0]
    ## calculate points
    if (region == 1) or (region == 3):
        edgePoint[0] += xFactor * (aa / 2.)
        edgePoint[1] += yFactor * (aa / 2.) * tanTheta
    else:
        edgePoint[0] += xFactor * (bb / (2. * tanTheta))
        edgePoint[1] += yFactor * (bb /  2.)

    return region, edgePoint

l_x = []
l_y = []
theta = 90
for _ in range(10000):
    r, (x, y) = get_points(600,300, theta)
    l_x.append(x)
    l_y.append(y)
    theta += (0.01 / PI)

    if _ % 100 == 0:
        print(r, x,y)

plt.plot(l_x, l_y)
plt.show()