import cv2
import numpy as np
import time
import fake_rpi
import sys 
sys.modules['RPi'] = fake_rpi.RPi     # Fake RPi
sys.modules['RPi.GPIO'] = fake_rpi.RPi.GPIO # Fake GPIO
import RPi.GPIO as GPIO



# Draws bounding circle (ball outline), returns center[] and radius
# Will eventually be void type when we mount the camera and determine const ball pos
def drawOutlineCircle(frame):
	height, width = frame.shape[:2]
	centerX = width//2
	centerY = height//2
	center = (centerX, centerY)
	radius = 10
	color = (255, 255, 255)
	thickness = 2
	cv2.circle(frame, center, radius, color, thickness) 
	return [center, radius]


# Returns pixel color 
def getColor(frame, point_coords):
	pixel_hsv = frame[point_coords[1], point_coords[0]]

	red_ranges = [((0,100,100),(10,255,255)), ((160, 100, 100),(180, 255, 255))]
	
	for lower, upper in red_ranges:
		lower = np.array(lower)
		upper = np.array(upper)
		if np.all((pixel_hsv >= lower) & (pixel_hsv <= upper)):
			return "red"	
	if pixel_hsv[0] < 33:
		return "yellow"
	if pixel_hsv[0] < 160:
		return "blue"
	
	return ("unknown color")


def generatePoints(num_samples, circle_dims):
	points = []
	ball_center_x, ball_center_y  = circle_dims[0]
	radius = circle_dims[1]

	for i in range(num_samples):

		theta_rand = np.random.rand() * 2 * np.pi
		dist_rand = np.random.rand() * radius
		dist_rand_x = dist_rand * np.cos(theta_rand)
		dist_rand_y = dist_rand * np.sin(theta_rand)
		
		rand_point_x = ball_center_x + dist_rand_x
		rand_point_y = ball_center_y + dist_rand_y

		points.append([rand_point_x, rand_point_y])

	return points

def sort():

	colors_detected = {"red":0, "yellow":0, "blue":0}		# dict to keep track of how many instances of each color occur in circle	

	cap = cv2.VideoCapture(0)
	cv2.namedWindow("window")

	while True:
		ret, frame_bgr = cap.read()
		frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
		dims = drawOutlineCircle(frame_bgr)
		cv2.imshow("window", frame_bgr)

		sampling_points = generatePoints(5, dims)
		for point in sampling_points:
			color = getColor(frame_hsv, dims[0])		# returns color (str) of one sampled pixel

		if cv2.waitKey(1) & 0xFF==27:
			break

	cv2.destroyAllWindows()
	cap.release()




sort()
