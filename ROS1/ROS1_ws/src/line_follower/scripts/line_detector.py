#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import math
import os
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


# Gaussian blur window size
kernel_size = 3

# Edge detection threshold
low_threshold = 50
high_threshold = 150

# Setting up a trapezoidal area of interest
trap_bottom_width = 1.  # Bottom width as a percentage
trap_top_width = 0.07  # Top Width as a percentage
trap_height = 0.4  # Height as a percentage

# Hough transform parameters
rho = 2  # Distance resolution (in pixels of the Hoff grid)
theta = 1 * np.pi/180  # Angular resolution (radians) of the Hough grid
threshold = 15	 # Minimum threshold (intersection in a Hough grid cell)
min_line_length = 10  # Minimum number of pixels to form a line
max_line_gap = 20  # Maximum pixel spacing between connectable line segments

# Camera parameters
fx = 604.7211303710938  # x-axis focal length
fy = 603.2431640625  # y-axis focal length
width = 1280  # 640
height = 720  # 480
cam_height = 1.0  # Camera mounting height
cam_center = 0.1  # Camera distance from middle position

# goal pos for limo
center_off = 0

# ROS parameters
img = 0
pub_img = 0


def grayscale(img):
	"""
    Converting images to greyscale
    """
	return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
	"""
    Edge detection
    """
	return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
	"""
    Gaussian filtering, blurring
    """
	return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def region_of_interest(img, vertices):
	"""
    Box the area of interest, i.e. the trapezoidal area where the lane lines are located
    """
	# Initialising a blank image
	mask = np.zeros_like(img)

	# Define 3 channels or 1 channel of colour to fill the mask according to the input image
	if len(img.shape) > 2:
		channel_count = img.shape[3]  # i.e. 3 or 4 depending on your image
		ignore_mask_color = (255,) * channel_count
	else:
		ignore_mask_color = 255

	# Fill the pixels inside the polygon defined by
	cv2.fillPoly(mask, vertices, ignore_mask_color)

	# Operate with the original image
	masked_image = cv2.bitwise_and(img, mask)
	return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
	"""
     Drawing lane lines
    The slope ((y2-y1)/(x2-x1)) determines which lines are part of the left-hand side
    This function is used to draw "lines" using "colour" and "thickness".
	"""
	# Determine if the parameters are correct
	if lines is None:
		return
	if len(lines) == 0:
		return
	draw_right = True
	draw_left = True

	# Find the slope of all straight lines
	# But only lines concerned with abs(slope) > slope threshold

	slope_threshold = 0.5
	slopes = []
	new_lines = []
	for line in lines:
		x1, y1, x2, y2 = line[0]  # line = [[x1, y1, x2, y2]]

		# Calculating the slope
		if x2 - x1 == 0.:  # Avoid dividing whole numbers by zero
			slope = 999.  # Slope maximum
		else:
			slope = 1.0*(y2 - y1) / (x2 - x1)

		# Slope based filter lines
		if abs(slope) > slope_threshold:
			slopes.append(slope)
			new_lines.append(line)

	lines = new_lines

	# Split the line into a right and a left lane to indicate the right and left lane lines
	# The right/left lane line must have a positive/negative slope and be located in the right/left half of the image
	right_lines = []
	left_lines = []
	for i, line in enumerate(lines):
		x1, y1, x2, y2 = line[0]
		img_x_center = img.shape[1] / 2  # x coordinate of center of image
		if slopes[i] > 0 and x1 > img_x_center and x2 > img_x_center:
			right_lines.append(line)
		elif slopes[i] < 0 and x1 < img_x_center and x2 < img_x_center:
			left_lines.append(line)

	# Linear regression to find the best-fit line for the left and right lane lines
	# Right lane line
	right_lines_x = []
	right_lines_y = []

	for line in right_lines:
		x1, y1, x2, y2 = line[0]

		right_lines_x.append(x1)
		right_lines_x.append(x2)

		right_lines_y.append(y1)
		right_lines_y.append(y2)

	if len(right_lines_x) > 0:
		right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
	else:
		right_m, right_b = 1, 1
		draw_right = False

	# Left lane line
	left_lines_x = []
	left_lines_y = []

	for line in left_lines:
		x1, y1, x2, y2 = line[0]

		left_lines_x.append(x1)
		left_lines_x.append(x2)

		left_lines_y.append(y1)
		left_lines_y.append(y2)

	if len(left_lines_x) > 0:
		left_m, left_b = np.polyfit(left_lines_x, left_lines_y, 1)  # y = m*x + b
	else:
		left_m, left_b = 1, 1
		draw_left = False

	# Find two endpoints for the left and right lines for drawing lines
	# y = m*x + b --> x = (y - b)/m
	y1 = img.shape[0]
	y2 = img.shape[0] * (1 - trap_height)

	right_x1 = (y1 - right_b) / right_m
	right_x2 = (y2 - right_b) / right_m

	left_x1 = (y1 - left_b) / left_m
	left_x2 = (y2 - left_b) / left_m

	# Converting a float to an int type
	y1 = int(y1)
	y2 = int(y2)
	right_x1 = int(right_x1)
	right_x2 = int(right_x2)
	left_x1 = int(left_x1)
	left_x2 = int(left_x2)
	# Drawing left and right lane lines
	if draw_right:
		cv2.line(img, (right_x1, y1), (right_x2, y2), color, thickness)
	if draw_left:
		cv2.line(img, (left_x1, y1), (left_x2, y2), color, thickness)
	if draw_right and draw_left:
		left, right = calc_distance_from_center(
		    0.5*width-1.0*left_x1, 1.0*right_x1-0.5*width)
		center_off = (left-right)/2-cam_center
		text = "left:"+str(round(left, 2))+"  right:"+str(round(right, 2)
		                   )+"  "+str(round(center_off, 2))+"  m off center"
		img = cv2.putText(img, text, (50, 50),
		                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
		pub_distance.publish(center_off)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
	"""
	Finding straight lines using the Hough transform
	"""
	lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array(
	    []), minLineLength=min_line_len, maxLineGap=max_line_gap)
	line_img = np.zeros((img.shape[0], img.shape[1], 3),
	                    dtype=np.uint8)  # 3-channel RGB image
	draw_lines(line_img, lines)
	return line_img


def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
	"""
	For merging images of the yellow and white line areas
	"""
	return cv2.addWeighted(initial_img, a, img, b, c)


def filter_colors(image):
	"""
	Filter the image, leaving the white parts and the yellow parts
	"""
	# Filter white
	white_threshold = 200  # 130
	lower_white = np.array([white_threshold, white_threshold, white_threshold])
	upper_white = np.array([255, 255, 255])
	white_mask = cv2.inRange(image, lower_white, upper_white)
	white_image = cv2.bitwise_and(image, image, mask=white_mask)

	# Filter yellow
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower_yellow = np.array([90, 100, 100])
	upper_yellow = np.array([110, 255, 255])
	yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)

	# Merge images
	image2 = cv2.addWeighted(white_image, 1., yellow_image, 1., 0.)

	return image2


def annotate_image(image_in):
	"""
    Processing of an image
    """
	# Filter colours
	image = filter_colors(image_in)

	# Converting images to greyscale
	gray = grayscale(image)

	# Gaussian blurring
	blur_gray = gaussian_blur(gray, kernel_size)

	# Edge detection
	edges = canny(blur_gray, low_threshold, high_threshold)

	# Setting the area of interest
	imshape = image.shape
	vertices = np.array([[\
		((imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0]),\
		((imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height),\
		(imshape[1] - (imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height),\
		(imshape[1] - (imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0])]]\
		, dtype=np.int32)
	masked_edges = region_of_interest(edges, vertices)

	# Hof Linear Inspection
	line_image = hough_lines(masked_edges, rho, theta, threshold, min_line_length, max_line_gap)
	

	# Draw lane lines
	initial_image = image_in.astype('uint8')
	annotated_image = weighted_img(line_image, initial_image)
	
	return annotated_image

def calc_distance_from_center(left,right):
    z_px=math.sqrt(fx*fx+left*left)
    z=z_px/(0.5*height/cam_height)
    left=1.0*left/(z_px/z)
    
    z_px=math.sqrt(fx*fx+right*right)
    z=z_px/(0.5*height/cam_height)
    right=1.0*right/(z_px/z)
    return left,right

def callback(imgmsg):
	bridge=CvBridge()
	img=bridge.imgmsg_to_cv2(imgmsg, "rgb8")
	height=img.shape[0]
	width=img.shape[1]
	img=annotate_image(img)
	pub_img.publish(bridge.cv2_to_imgmsg(img, "rgb8"))


if __name__ == '__main__':
	rospy.init_node('lane_detection',anonymous=True)
	rospy.Subscriber("/camera/rgb/image_raw",Image,callback)
	pub_img=rospy.Publisher('/lane_detection/image_raw', Image, queue_size=1) 
	pub_distance=rospy.Publisher('/lane_detection/off_center', Float32, queue_size=1) 
	rospy.spin()