import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import math
import os

def subsample(img, kernel):
    img = cv.bitwise_not(img)

    qx = int(img.shape[0]/kernel)
    qy = int(img.shape[1]/kernel)

    img2 = np.ones((qx, qy))

    for i in range(0, qx):
        for j in range(0, qy):
            block = img[i*kernel : i*kernel + kernel, j*kernel : j*kernel + kernel]
            if np.any(block):
                img2[i, j] = 0

    return img2

img = cv.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maze.png'),0)

## Segmentação entre obstáculos(Preto) e espaço navegável(Branco)
ret,thresh = cv.threshold(img,127,255,cv.THRESH_BINARY_INV)
## Remoção de artefatos do grid do blender
blur = cv.medianBlur(thresh,5)
## Remoção das bordas do recorte da imagem
contours, hierarchy = cv.findContours(blur, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
contours = np.ravel(contours[1])
min_contour = np.min(contours)
max_contour = np.max(contours)
croped = blur[min_contour:max_contour, min_contour:max_contour]
## Aumento de obstáculos para o espaço de configurações
map_size = 10
robot_diameter = .64
pixel_size = map_size/len(croped)
kernel_size = math.ceil(robot_diameter/pixel_size)
kernel = np.ones((kernel_size,kernel_size), np.uint8)
eroded = cv.erode(croped, kernel)
## Rotação para sistema de coordenadas do Gazebo
final = cv.rotate(eroded, cv.ROTATE_90_CLOCKWISE)

subsampled = subsample(final, kernel=17)
print(subsampled.shape)

test = np.array([[1, 0],[0, 1]])

plt.subplot(131),plt.imshow(img,'gray',vmin=0,vmax=255),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(132),plt.imshow(final,'gray',vmin=0,vmax=255),plt.title('Treated Map')
plt.xticks([]), plt.yticks([])
plt.subplot(133),plt.imshow(subsampled,'gray'),plt.title('Wavefront')
plt.xticks([]), plt.yticks([])
plt.show()