import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import math
import os

def getStep(img, position):
    neighbor_indexes = [(position[0],position[1]-1),
                        (position[0]-1,position[1]),
                        (position[0]+1,position[1]),
                        (position[0],position[1]+1)]
    
    neghbor_values = np.array([])
    for neighbor_index in neighbor_indexes:
        neghbor_values = np.append(neghbor_values, img[neighbor_index[0], neighbor_index[1]])

    min_neighbors = np.where(neghbor_values == np.min(neghbor_values[np.nonzero(neghbor_values)]))[0]
    min_neighbor_index = neighbor_indexes[min_neighbors[0]]

    if img[min_neighbor_index[0], min_neighbor_index[1]] < img[position[0], position[1]]:
        return min_neighbor_index[0], min_neighbor_index[1]
    else:
        return position[0], position[1]

img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'wave.npy'))

normal = 255*img/np.max(img)

color = cv.cvtColor(np.uint8(normal),cv.COLOR_GRAY2RGB)

position = (110,90)
while True:
    x, y = getStep(img, position)

    if position == (x, y):
        break

    position = (x, y)
    color[x, y, 0] = 255
    color[x, y, 1] = 0
    color[x, y, 2] = 0

plt.subplot(121),plt.imshow(img,'gray'),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(color),plt.title('Path')
plt.xticks([]), plt.yticks([])
plt.show()