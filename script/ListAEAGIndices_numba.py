from __future__ import print_function

import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
from numba import cuda
import time

@cuda.jit(device=True)
def d_radius_validate(cx, cy, R, width, x, y):
    x = x - cx
    y = y - cy

    r = math.sqrt( x * x + y * y )

    if ( r >= R - width and r <= R + width ):
        return 255
    else:
        return 0

@cuda.jit(device=True)
def d_polar_line_segment_validate(cx, cy, theta, length, width, x, y):
    n0 = length * math.cos(theta) / length
    n1 = length * math.sin(theta) / length

    v0 = x - cx
    v1 = y - cy

    proj = n0 * v0 + n1 * v1

    if ( proj < 0 ):
        return 0
    elif ( proj > length ):
        return 0
    
    oth0 = v0 - proj*n0
    oth1 = v1 - proj*n1

    d = math.sqrt( oth0 * oth0 + oth1 * oth1 )

    if ( d <= width ):
        return 255
    else:
        return 0

@cuda.jit
def k_validate(imgOut):
    tx = cuda.blockIdx.x*cuda.blockDim.x + cuda.threadIdx.x
    ty = cuda.blockIdx.y*cuda.blockDim.y + cuda.threadIdx.y

    xStride = cuda.blockDim.x * cuda.gridDim.x
    yStride = cuda.blockDim.y * cuda.gridDim.y

    cx, cy = 2056, 1504
    R = 1504*0.75
    halfWidth = 10.0

    for y in range( ty, imgOut.shape[0], yStride ):
        for x in range( tx, imgOut.shape[1], xStride ):
            flag = d_radius_validate( cx, cy, R, halfWidth, 1.0*x, 1.0*y )
            if ( 0 != flag ):
                imgOut[y, x] = flag

            flag = d_polar_line_segment_validate( cx, cy, -math.pi/2, R, halfWidth, 1.0*x, 1.0*y )
            if ( 0 != flag ):
                imgOut[y, x] = flag

            flag = d_polar_line_segment_validate( cx, cy, -math.pi/4, R, halfWidth, 1.0*x, 1.0*y )
            if ( 0 != flag ):
                imgOut[y, x] = flag
            
            flag = d_polar_line_segment_validate( cx-halfWidth, cy, 0.0, R + halfWidth, halfWidth, 1.0*x, 1.0*y )
            if ( 0 != flag ):
                imgOut[y, x] = flag
            
            flag = d_polar_line_segment_validate( cx, cy, math.pi/4, R, halfWidth, 1.0*x, 1.0*y )
            if ( 0 != flag ):
                imgOut[y, x] = flag
            
            flag = d_polar_line_segment_validate( cx, cy, math.pi/2, R, halfWidth, 1.0*x, 1.0*y )
            if ( 0 != flag ):
                imgOut[y, x] = flag

def get_index(img):
    x = np.linspace(0, img.shape[1]-1, img.shape[1], dtype=np.int)
    y = np.linspace(0, img.shape[0]-1, img.shape[0], dtype=np.int)

    x, y = np.meshgrid(x, y)
    
    mask = img > 0

    x = x[mask]
    y = y[mask]
    f = []

    fR = 0.299 # 0
    fG = 0.587 # 1
    fB = 0.114 # 2

    cR = 0
    cG = 0
    cB = 0

    for i in range(x.shape[0]):
        ix = x[i]
        iy = y[i]

        if ( iy % 2 == 0 ):
            if ( ix % 2 == 1 ):
                f.append(1)
                cG += 1
            else:
                f.append(2)
                cB += 1
        else:
            if ( ix % 2 == 0 ):
                f.append(1)
                cG += 1
            else:
                f.append(0)
                cR += 1
    
    n = len(f)

    if ( n != cR + cG + cB ):
        raise Exception("n != cR + cG + cB. n = %d, cR = %d, cG = %d, cB = %d." % ( n, cR, cG, cB ))
    
    print("(R, G, B) = (%d, %d, %d)" % (cR, cG, cB))

    # import ipdb; ipdb.set_trace()

    for i in range(len(f)):
        if ( 0 == f[i] ):
            f[i] = fR / cR
        elif ( 1 == f[i] ):
            f[i] = fG / cG
        elif ( 2 == f[i] ):
            f[i] = fB / cB
        else:
            raise Exception("Unexpected f[%d] == %d." % (i, f[i]))

    f = np.array(f, dtype=np.float32)

    x = x.reshape((-1, 1))
    y = y.reshape((-1, 1))
    f = f.reshape((-1, 1))

    m = np.concatenate((x, y, f), axis=1)

    return m

if __name__ == "__main__":
    # Create an image.
    img = np.zeros((3008, 4112), dtype=np.int32)

    # Record the starting time.
    start = time.time()

    dImg = cuda.to_device(img)

    cuda.synchronize()
    k_validate[[100, 100, 1], [16, 16, 1]](dImg)
    cuda.synchronize()

    img = dImg.copy_to_host()

    # Record the ending time.
    end = time.time()

    print(end - start)

    # Save the image.
    cv2.imwrite("ValidPixels_numba.png", img)

    # Get the index.
    idx = get_index(img)

    print("idx.shape = {}".format(idx.shape))

    np.savetxt("index.dat", idx, fmt="%d %d %.8f")

    # Plot the index.
    plt.plot( idx[:, 0], idx[:, 1], '.' )
    plt.show()

    print("Done.")
