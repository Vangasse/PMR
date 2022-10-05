import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import math

def wavefront(wave, goal):
    queue = [goal]

    while queue:
        current_node = queue.pop(0)
        print(current_node)
        current_value = wave[current_node[0]][current_node[1]]

        neighbor_indexes = [(current_node[0]-1,current_node[1]-1),  (current_node[0],current_node[1]-1),    (current_node[0]+1,current_node[1]-1),
                            (current_node[0]-1,current_node[1]),                                            (current_node[0]+1,current_node[1]),
                            (current_node[0]-1,current_node[1]+1),  (current_node[0],current_node[1]+1),    (current_node[0]+1,current_node[1]+1) ]

        for neighbor_index in neighbor_indexes:
            i = neighbor_index[0]
            j = neighbor_index[1]
            neighbor_value = wave[i][j]

            print(neighbor_index, wave[i][j])

            if neighbor_value == 0:
                continue
            if neighbor_value == 1:
                wave[i,j] = current_value + 1
                queue.append(neighbor_index)
            elif current_value + 1 < neighbor_value:
                wave[i,j] = current_value + 1
        # break
    return 255*wave/(np.max(wave))


img = cv.imread('/home/vangasse/PMR_ws/src/PMR/img/maze.png',0)

ret,thresh = cv.threshold(img,127,255,cv.THRESH_BINARY_INV)

blur = cv.medianBlur(thresh,5)

contours, hierarchy = cv.findContours(blur, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
# cv.drawContours(blur, contours[1], -1, (0,255,0), 3)
contours = np.ravel(contours[1])
min_contour = np.min(contours)
max_contour = np.max(contours)

croped = blur[min_contour:max_contour, min_contour:max_contour]

map_size = 10
robot_diameter = .6
pixel_size = 10/len(croped)
kernel_size = math.ceil(robot_diameter/pixel_size)

kernel = np.ones((kernel_size,kernel_size), np.uint8)

final = cv.erode(croped, kernel)

contours, hierarchy = cv.findContours(final, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
# cv.drawContours(blur, contours, -1, (0,255,0), 3)

wave = wavefront(final/255, (70,440))

plt.subplot(131),plt.imshow(img,'gray',vmin=0,vmax=255),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(132),plt.imshow(final,'gray',vmin=0,vmax=255),plt.title('Treated Map')
plt.xticks([]), plt.yticks([])
plt.subplot(133),plt.imshow(wave,'gray',vmin=0,vmax=255),plt.title('Wavefront')
plt.xticks([]), plt.yticks([])
plt.show()