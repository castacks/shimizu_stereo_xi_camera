from __future__ import print_function

import cv2
import math
import numpy as np
from numpy.linalg import norm

H = 3008
W = 4112

class Validator(object):
    def __init__(self):
        pass

    def is_valid(self, x, y):
        return False

class RadiusValidator(Validator):
    def __init__(self, center, R, width):
        super(RadiusValidator, self).__init__()

        self.R = R
        self.center = center # A two element NumPy array. [x, y].
        self.width = width

        if ( self.width <= 0 ):
            raise Exception("self.width wrong. self.width = {}".format(self.width))
    
    # Overide parent's function.
    def is_valid(self, x, y):
        x = x - self.center[0]
        y = y - self.center[1]

        r = math.sqrt( x * x + y * y )

        if ( r >= self.R - self.width and r <= self.R + self.width ):
            return True
        else:
            return False

class PolarLineSegmentValidator(Validator):
    def __init__(self, center, theta, length, width):
        super(PolarLineSegmentValidator, self).__init__()

        self.center = center # A two element NumPy array. [x, y].
        self.theta  = theta
        self.length = length
        self.width  = width

        self.endP    = np.zeros((2,), dtype=np.float32)
        self.endP[0] = self.length * math.cos( self.theta )
        self.endP[1] = self.length * math.sin( self.theta )

        self.dir = self.endP / self.length

    def is_valid(self, x, y):
        # Transform the coordinate to the local frame.
        v = np.zeros((2, ), dtype=np.float32)

        v[0] = x - self.center[0]
        v[1] = y - self.center[1]

        # Projection.
        proj = self.dir.dot( v )

        if ( proj < 0 ):
            return False
        elif ( proj > self.length ):
            return False

        # Othogonal vector.
        oth = v - proj*self.dir

        # Distance.
        d = norm( oth )

        if ( d <= self.width ):
            return True
        else:
            return False

if __name__ == "__main__":
    print("List the indices for AEAG brightness evaluation.")

    # Create a blank image with all zeros.
    img = np.zeros((H, W), dtype=np.uint8)

    imgCenter = np.array([ W/2, H/2 ], dtype=np.float32)

    # Create the validators.
    vList = []
    vList.append( RadiusValidator( imgCenter, min( W/2, H/2 ) * 0.75, 20 ) )
    vList.append( PolarLineSegmentValidator( imgCenter,  math.pi/2, min( W/2, H/2 ) * 0.75, 20 ) )
    vList.append( PolarLineSegmentValidator( imgCenter,  math.pi/4, min( W/2, H/2 ) * 0.75, 20 ) )
    vList.append( PolarLineSegmentValidator( imgCenter,          0, min( W/2, H/2 ) * 0.75, 20 ) )
    vList.append( PolarLineSegmentValidator( imgCenter, -math.pi/4, min( W/2, H/2 ) * 0.75, 20 ) )
    vList.append( PolarLineSegmentValidator( imgCenter, -math.pi/2, min( W/2, H/2 ) * 0.75, 20 ) )

    for i in range(H):
        for j in range(W):
            flag = False
            for v in vList:
                flag = flag or v.is_valid( j, i )
            
            if ( True == flag ):
                img[i, j] = 255

        print("%d," % (i), end=None)
    
    # Save the image.
    cv2.imwrite("ValidPixels.png", img)
