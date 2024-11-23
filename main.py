### import libraries ###
import pygame
import time
import RPi.GPIO as GPIO
import serial

### initialize joystick ###
pygame.init()
pygame.joystick.init()

ser = serial.Serial('/dev/serial0', baudrate = 9600, timeout = 1)

### intake motors ###
M1A = 13
M2A = 12
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(M1A, GPIO.OUT)
GPIO.setup(M2A, GPIO.OUT)

M1A_p = GPIO.PWM(M1A, 2000)
M2A_p = GPIO.PWM(M2A, 2000)

### functions ###
def start_intake_motors():
    speed = 255
    M1A_p.ChangeDutyCycle(speed/2.55)
    M2A_p.ChangeDutyCycle(speed/2.55)

def stop_intake_motors():
    speed = 0
    M1A_p.ChangeDutyCycle(speed/2.55)
    M2A_p.ChangeDutyCycle(speed/2.55)

def map_value(value, in_min, in_max, out_min, out_max):
    in_range = in_max - in_min
    out_range = out_max - out_min
    scaled_value = (value - in_min) * (out_range/in_range)
    map_val = round(out_min + scaled_value, 3)
    return map_val

def set_motor_speed(turn, speed):
    left_wheel_speed = int(speed*(1 + turn))
    right_wheel_speed = int(speed*(1 - turn))
    left_motor_speed = 64 + max(-63, min(63, left_wheel_speed))
    right_motor_speed = 192 + max(-63, min(63, right_wheel_speed))
    print(f"Left speed: {left_motor_speed} Right speed: {right_motor_speed}")

    ser.write(bytes([left_motor_speed]))
    ser.write(bytes([right_motor_speed]))

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
        intake_on = False
        running_main_code = False
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

                if joystick.get_button(2):
                    if not intake_on:
                        intake_on = True
                        start_intake_motors()
                        print("intake on")
                        time.sleep(0.5)
                    elif intake_on:
                        intake_on = False
                        stop_intake_motors()
                        print("intake off")
                        time.sleep(0.5)

                foward_speed_axis = round(joystick.get_axis(5), 3)
                foward_speed = map_value(foward_speed_axis, -1, 1, 0, 63)

                backward_speed_axis = round(joystick.get_axis(2), 3)
                backward_speed = map_value(foward_speed_axis, -1, 1, 0, -63)

                turn = round(joystick.get_axis(0), 3)
                if abs(turn) < 0.1:
                    turn = 0

                if foward_speed > 0:
                    set_motor_speed(turn, foward_speed)
                    time.sleep(0.1)
                if backward_speed > 0:
                    set_motor_speed(turn, backward_speed)
                    time.sleep(0.1)  

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
pygame.quit()
