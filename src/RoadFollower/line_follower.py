import cv2
import numpy as np
import math
import serial
import os

actual_path = os.path.dirname(os.path.abspath(__file__))
image = cv2.imread(os.path.join(actual_path, 'test1.jpg')) 
image = cv2.resize(image, (640, 480))
imshow = cv2.imshow('image', image)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

k_width = 5
k_height = 5

# Image smoothing
blurred = cv2.GaussianBlur(gray, (k_width, k_height), 0)

# Edge detection
threshold1 = 50
threshold2 = 50
edged = cv2.Canny(blurred, threshold1, threshold2)

# Find contours
contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

print("Number of contours found = " + str(len(contours)))
print("Number of hierarchy found = " + str(len(hierarchy))) 

# Draw all contours
# -1 signifies drawing all contours
cv2.drawContours(image, contours, -1, (0, 255, 0), 3) 
cv2.imshow('Contours', image)
cv2.waitKey(20000)
cv2.destroyAllWindows()