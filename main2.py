### import libraries ###
import pygame
import time
import RPi.GPIO as GPIO
import serial
import cv2
import numpy as np
import os
import subprocess

### initialize joystick ###
pygame.init()
pygame.joystick.init()

ser = serial.Serial('/dev/serial0', baudrate = 9600, timeout = 1)

### initalize motor pins ###
flywheel = 6
pulley = 5
launcher_extend = 17
launcher_retract = 27
weightlift_extend = 9
weightlift_retract = 10
topServo = 12
slideServo = 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(flywheel, GPIO.OUT)
GPIO.setup(pulley, GPIO.OUT)
GPIO.setup(launcher_extend, GPIO.OUT)
GPIO.setup(launcher_retract, GPIO.OUT)
GPIO.setup(weightlift_extend, GPIO.OUT)
GPIO.setup(weightlift_retract, GPIO.OUT)
GPIO.setup(topServo, GPIO.OUT)
GPIO.setup(slideServo, GPIO.OUT)

flywheel_p = GPIO.PWM(flywheel, 2000)
pulley_p = GPIO.PWM(pulley, 2000)
launcher_extend_p = GPIO.PWM(launcher_extend, 2000)
launcher_retract_p = GPIO.PWM(launcher_retract, 2000)
weightlift_extend_p = GPIO.PWM(weightlift_extend, 2000)
weightlift_retract_p = GPIO.PWM(weightlift_retract, 2000)
topServo_p = GPIO.PWM(topServo, 2000)
topServo_p.start(11)
slideServo_p = GPIO.PWM(slideServo, 50)
slideServo_p.start(1)

### functions ###
def set_motor_speed(motor_pin, speed):
    motor_pin.ChangeDutyCycle(speed/2.55)

def map_value(value, in_min, in_max, out_min, out_max):
    in_range = in_max - in_min
    out_range = out_max - out_min
    scaled_value = (value - in_min) * (out_range/in_range)
    map_val = round(out_min + scaled_value, 3)
    return map_val

def set_drive_speed(turn, speed):
    left_wheel_speed = int(speed*(1 + turn))
    right_wheel_speed = int(speed*(1 - turn))
    if speed > 0:
        left_motor_speed = 64 + max(-63, min(63, left_wheel_speed))
        right_motor_speed = 192 + max(-63, min(63, right_wheel_speed))
    elif speed < 0:
        left_motor_speed = 64 + min( max(-63, left_wheel_speed), 63)
        right_motor_speed = 192 + min( max(-63, right_wheel_speed), 63)
    elif speed == 0:
        left_motor_speed = 64
        right_motor_speed = 192

    print(f"Left speed: {left_motor_speed} Right speed: {right_motor_speed}")

    ser.write(bytes([left_motor_speed]))
    ser.write(bytes([right_motor_speed]))

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

### main loop ###

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detected {joystick.get_name()}")
    running_main_code = False
    print(f"Running main code {running_main_code}")

    try:
        speed = 0
        running = True

        # set all actions as False
        intake_on = False
        launcher_extended = False
        weightlift_extended = False
        running_main_code = False
        sorting_on = False
        launch_ball_sorted = False

        print("running")
        while running:
            for event in pygame.event.get(0):
                if event.type == pygame.QUIT:
                    running = False

            if joystick.get_button(10):
                print("Button 10 is pressed")
                if not running_main_code:
                    running_main_code = True
                    print(f"Running main code {running_main_code}")
                    time.sleep(0.5)

            while running_main_code:
                for event in pygame.event.get(0):
                    if event.type == pygame.QUIT:
                        running_main_code = False

                ## set drive motors
                foward_speed_axis = round(joystick.get_axis(5), 3)
                foward_speed = map_value(foward_speed_axis, -1, 1, 0, 63)

                backward_speed_axis = round(joystick.get_axis(2), 3)
                backward_speed = map_value(backward_speed_axis, -1, 1, 0, -63)

                turn = round(joystick.get_axis(0), 3)
                if abs(turn) < 0.1:
                    turn = 0

                ## send motor speed to motors
                if foward_speed > 1:
                    set_drive_speed(turn, foward_speed)
                    time.sleep(0.1)
                elif backward_speed < -1:
                    set_drive_speed(turn, backward_speed)
                    time.sleep(0.1)
                elif(foward_speed < 1 and backward_speed > -1):
                    set_drive_speed(0, 0)
                    time.sleep(0.1)

                ## break function
                if joystick.get_button(4):
                    print("break pressed")
                    set_drive_speed(0, 0)
                    time.sleep(0.5)

                ## set intake motors
                if joystick.get_button(2):
                    if not intake_on:
                        intake_on = True
                        set_motor_speed(flywheel_p, 255)
                        set_motor_speed(pulley_p, 255)
                        print("intake on")
                        time.sleep(0.5)
                    elif intake_on:
                        intake_on = False
                        set_motor_speed(flywheel_p, 0)
                        set_motor_speed(pulley_p, 0)
                        print("intake off")
                        time.sleep(0.5)

                ## set launch/ weightlift motor
                hat_val = joystick.get_hat(0)
                if hat_val == (0, 1):
                    if not launcher_extended:
                        launcher_extended = True
                        set_motor_speed(launcher_extend_p, 255)
                        print("launcher extended")
                        time.sleep(0.5)
                    elif launcher_extended:
                        launcher_extended = False
                        set_motor_speed(launcher_retract_p, 255)
                        print("launcher retracted")
                        time.sleep(0.5)
                
                if hat_val == (0, -1):
                    if not weightlift_extended:
                        weightlift_extended = True
                        set_motor_speed(weightlift_extend_p, 255)
                        print("weightlift extended")
                        time.sleep(0.5)
                    elif weightlift_extended:
                        weightlift_extended = False
                        set_motor_speed(weightlift_retract_p, 255)
                        print("weightlift retracted")
                        time.sleep(0.5)
                
                ## select color to launch
                if joystick.get_button(1):
                    launch_color = "blue"
                    print("blue selected")
                    time.sleep(0.5)


                if joystick.get_button(3):
                    launch_color = "yellow"
                    print("yellow selected")
                    time.sleep(0.5)

                if joystick.get_button(0):
                    launch_color = "red"
                    print("red selected")
                    time.sleep(0.5)
                
                ## start sorting system
                if joystick.get_button(9):
                    print("Button 9 pressed")
                    if not sorting_on:
                        sorting_on = True
                        print("sorting on")
                        topServo_p.ChangeDutyCycle(11)
                        time.sleep(0.5)
                        topServo_p.ChangeDutyCycle(0)
                        time.sleep(3)
                        # move servo to capture color
                        topServo_p.ChangeDutyCycle(9)
                        time.sleep(0.5)
                        topServo_p.ChangeDutyCycle(0)
                        color_determined = sort()
                        print(color_determined)
                        time.sleep(3)

                        if(color_determined == launch_color and launch_ball_sorted == False):
                            slideServo_p.ChangeDutyCycle(3.75)
                            time.sleep(0.5)
                            print("3.75 - launch ball sorted")
                            slideServo_p.ChangeDutyCycle(0)
                            launch_ball_sorted = True
                        elif(color_determined == "blue"):
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
                    elif sorting_on:
                        sorting_on = False
                        time.sleep(0.5)
                        print("sorting off")
                        

                if joystick.get_button(10):
                    print("Button 10 is pressed")
                    if running_main_code:
                        running_main_code = False
                        print(f"not running main code {running_main_code}")
                        running = False

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n exiting")
else:
    print("No joystick detected")
topServo_p.stop()
slideServo_p.stop()
GPIO.cleanup()
pygame.quit()
