import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):

    # Take each frame
    _, frame = cap.read()
    kernel = np.ones((5,5),np.uint8)

    #erosion  & dilate & edge detection
    erosion = cv2.erode(frame,kernel,iterations = 2)
    dilation = cv2.dilate(frame,kernel,iterations = 3)
    edges = cv2.Canny(frame,100,200)

    cv2.imshow('Erosion', erosion)
    cv2.imshow('Dilation', dilation)
    cv2.imshow('Canny', edges)  

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()



