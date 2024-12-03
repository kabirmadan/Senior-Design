import cv2
import numpy as np
import os
import subprocess
import time
import RPi.GPIO as GPIO

topServo = 27
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(topServo, GPIO.OUT)
topServo_p = GPIO.PWM(topServo, 50)
topServo_p.start(11)

slideServo = 17
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(slideServo, GPIO.OUT)
slideServo_p = GPIO.PWM(slideServo, 50)
slideServo_p.start(1)

# Draws bounding circle (ball outline), returns center[] and radius
def drawOutlineCircle(frame):
    height, width = frame.shape[:2]
    centerX = width //2 - 20
    centerY = height // 2 + 125
    center = (centerX, centerY)
    radius = 70
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
    
    return color_determined
    
    # Show the frame (optional, for debugging when not headless)
    cv2.imshow("Captured Frame", frame_bgr)
    #if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
    #break
    #cv2.destroyAllWindows()


try:
    # move servo to catch ball
    topServo_p.ChangeDutyCycle(11)
    time.sleep(0.5)
    topServo_p.ChangeDutyCycle(0)
    time.sleep(3)
    # move servo to capture color
    topServo_p.ChangeDutyCycle(9)
    time.sleep(0.5)
    topServo_p.ChangeDutyCycle(0)
    time.sleep(5)
    color_determined = sort()
    print(color_determined)
        
    if(color_determined == "blue"):
        slideServo_p.ChangeDutyCycle(5)
        time.sleep(0.5)
        print("5")
        slideServo_p.ChangeDutyCycle(0)
    elif(color_determined == "yellow"):
        slideServo_p.ChangeDutyCycle(6)
        time.sleep(0.5)
        print("6")
        slideServo_p.ChangeDutyCycle(0)
    elif(color_determined == "red"):
        slideServo_p.ChangeDutyCycle(7)
        time.sleep(0.5)
        print("7")
        slideServo_p.ChangeDutyCycle(0)
        
    # move servo to drop ball
    topServo_p.ChangeDutyCycle(5.5)
    time.sleep(0.5)
    topServo_p.ChangeDutyCycle(0)
        
except KeyboardInterrupt:
    topServo_p.stop()
    slideServo_p.stop()
    GPIO.cleanup()
