import cv2
import numpy as np
import os
import subprocess
import time
import RPi.GPIO as GPIO

SERVO_1_PIN = 12
SERVO_2_PIN = 13
PWM_FREQ = 1000  # Hz

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_1_PIN, GPIO.OUT)
GPIO.setup(SERVO_2_PIN, GPIO.OUT)
Servo1 = GPIO.PWM(SERVO_1_PIN, PWM_FREQ)
Servo2 = GPIO.PWM(SERVO_2_PIN, PWM_FREQ)
Servo1.start(0)
Servo2.start(0)

# Draws bounding circle (ball outline), returns center[] and radius
def drawOutlineCircle(frame):
	height, width = frame.shape[:2]
	centerX = width // 2
	centerY = height // 2
	center = (centerX, centerY)
	radius = 100
	color = (255, 255, 255)
	thickness = 2
	cv2.circle(frame, center, radius, color, thickness)

	output_path = "/home/seniordesign/gittest/Senior-Design/captured_image.jpg"
	cv2.imwrite(output_path, frame)
	return [center, radius]


# Returns pixel color
def getColor(frame, point_coords):
	pixel_hsv = frame[int(point_coords[1]), int(point_coords[0])]

	red_ranges = [((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (180, 255, 255))]

	for lower, upper in red_ranges:
		lower = np.array(lower)
		upper = np.array(upper)
		if np.all((pixel_hsv >= lower) & (pixel_hsv <= upper)):
			return "red"
	if pixel_hsv[0] < 33:
		return "yellow"
	if pixel_hsv[0] < 160:
		return "blue"

	return "other"


# Generates n random points within limits of bounding circle
def generatePoints(n, circle_dims):
	points = []
	ball_center_x, ball_center_y = circle_dims[0]
	radius = circle_dims[1]

	for i in range(n):
		theta_rand = np.random.rand() * 2 * np.pi
		dist_rand = np.random.rand() * radius
		dist_rand_x = dist_rand * np.cos(theta_rand)
		dist_rand_y = dist_rand * np.sin(theta_rand)

		rand_point_x = ball_center_x + dist_rand_x
		rand_point_y = ball_center_y + dist_rand_y

		points.append([rand_point_x, rand_point_y])

	return points


def capture_image(output_path):
	# Use rpicam-jpeg to capture an image
	subprocess.run(["rpicam-jpeg", "--output", output_path, "--width", "640", "--height", "480"])
	time.sleep(1)  # Ensure the image is saved before continuing


def sort():
	output_path = "/home/seniordesign/gittest/Senior-Design/captured_image.jpg"
	
	# Capture a still image
	capture_image(output_path)
	
	# Read the captured image
	frame_bgr = cv2.imread(output_path)
	if frame_bgr is None:
		print("Error: Failed to read the captured image.")
		continue
	
	# Convert to HSV and process the frame
	frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
	dims = drawOutlineCircle(frame_bgr)
	
	# Analyze colors within the bounding circle
	colors_detected = {"red": 0, "yellow": 0, "blue": 0, "other": 0}
	sampling_points = generatePoints(100, dims)
	for point in sampling_points:
		color = getColor(frame_hsv, point)
		colors_detected[color] += 1
	
	# Determine the dominant color
	color_determined = max(colors_detected, key=colors_detected.get)
	print("Colors detected:", colors_detected)
	print("Dominant color:", color_determined)
	
	cv2.destroyAllWindows()
	return color_determined
	
	# Show the frame (optional, for debugging when not headless)
	#cv2.imshow("Captured Frame", frame_bgr)
	#if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
	#	break

