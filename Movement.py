import cv2
import numpy as np
import RPi.GPIO as GPIO

Kp = 0.5
Kd = 0.1


base_speed = 50


prev_error_x = 0
prev_error_y = 0



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


left_motor_pin = 17
right_motor_pin = 18


pwm_freq = 1000


GPIO.setup(left_motor_pin, GPIO.OUT)
GPIO.setup(right_motor_pin, GPIO.OUT)


left_motor_pwm = GPIO.PWM(left_motor_pin, pwm_freq)
right_motor_pwm = GPIO.PWM(right_motor_pin, pwm_freq)


left_motor_pwm.start(0)
right_motor_pwm.start(0)


def move_to_point(center_between):
    global prev_error_x, prev_error_y

    current_x, current_y = 320, 240

    error_x = center_between[0] - current_x
    error_y = center_between[1] - current_y

    d_error_x = error_x - prev_error_x
    d_error_y = error_y - prev_error_y

    U_x = Kp * error_x + Kd * d_error_x
    U_y = Kp * error_y + Kd * d_error_y

    prev_error_x = error_x
    prev_error_y = error_y

    motor_left = base_speed + U_x
    motor_right = base_speed - U_x

    motor_left = max(0, min(100, motor_left))
    motor_right = max(0, min(100, motor_right))

    print(f"Moving to point: {center_between}")
    print(f"Left Motor Speed: {motor_left}%, Right Motor Speed: {motor_right}%")

    left_motor_pwm.ChangeDutyCycle(motor_left)
    right_motor_pwm.ChangeDutyCycle(motor_right)

def stop_motors():
    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)



if cv2.waitKey(1) & 0xFF == ord('q'):
    stop_motors()
