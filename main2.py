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
flywheel = 13
pulley = 12
launcher_extend = 17
launcher_retract = 27
weightlift_extend = 9
weightlift_retract = 10

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(flywheel, GPIO.OUT)
GPIO.setup(pulley, GPIO.OUT)
GPIO.setup(launcher_extend, GPIO.OUT)
GPIO.setup(launcher_retract, GPIO.OUT)
GPIO.setup(weightlift_extend, GPIO.OUT)
GPIO.setup(weightlift_retract, GPIO.OUT)

flywheel_p = GPIO.PWM(flywheel, 2000)
pulley_p = GPIO.PWM(pulley, 2000)
launcher_extend_p = GPIO.PWM(launcher_extend, 2000)
launcher_retract_p = GPIO.PWM(launcher_retract, 2000)
weightlift_extend_p = GPIO.PWM(weightlift_extend, 2000)
weightlift_retract_p = GPIO.PWM(weightlift_retract, 2000)

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
        launcher_extended = False
        weightlift_extended = False
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

                ## set drive motors
                foward_speed_axis = round(joystick.get_axis(5), 3)
                foward_speed = map_value(foward_speed_axis, -1, 1, 0, 63)

                backward_speed_axis = round(joystick.get_axis(2), 3)
                backward_speed = map_value(foward_speed_axis, -1, 1, 0, -63)

                turn = round(joystick.get_axis(0), 3)
                if abs(turn) < 0.1:
                    turn = 0

                if foward_speed > 0:
                    set_drive_speed(turn, foward_speed)
                    time.sleep(0.1)
                if backward_speed > 0:
                    set_drive_speed(turn, backward_speed)
                    time.sleep(0.1)  

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
