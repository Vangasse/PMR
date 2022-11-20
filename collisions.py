import sys
import numpy as np
from matplotlib import pyplot as plt
import math
import os
import cv2 as cv

def check_connection(img, start_point, end_point, resolution=.25):

    num = int(abs(start_point[1] - end_point[1])/resolution)

    x = np.round(np.linspace(start_point[1],end_point[1],num=num))
    y = np.round(np.linspace(start_point[0],end_point[0],num=num))
    x = x.reshape(len(x), 1)
    y = y.reshape(len(y), 1)
    line = np.concatenate((x,y), axis=1)
    
    for point in line:
        if all(img[int(point[0]), int(point[1])] == 0):
            return 0, int(point[0]), int(point[1])
    return 1, -1, -1

img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/subsampled.npy'))
print(img.shape)
img = np.stack((img,)*3, axis=-1)

start_point = (24, 36)
end_point = (36, 28)

connection, hit_x, hit_y = check_connection(img, start_point, end_point)

img[start_point[1], start_point[0]] = (0, 1, 0)
img[hit_x, hit_y] = (1, 0, 0)
img[end_point[1], end_point[0]] = (1, 0, 1)

plt.imshow(img,vmin=0,vmax=1),plt.title('Subsampled')
plt.show()