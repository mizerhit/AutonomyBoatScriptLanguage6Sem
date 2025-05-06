import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

Kp = 0.1
Ki = 0.001
Kd = 0.05

base_speed = 100
max_speed = 50
robot_state = "tracking_blue"
blue_lost_time = None

prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0
last_time = time.time()

GPIO.setmode(GPIO.BCM)

ENA = 24
ENB = 23
IN1 = 17
IN2 = 27
IN3 = 18
IN4 = 22

GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

freq = 100

pwm_left = GPIO.PWM(ENA, freq)
pwm_right = GPIO.PWM(ENB, freq)
pwm_left.start(0)
pwm_right.start(0)

def set_motor_speeds(left_speed, right_speed):
    left_speed = max(min(left_speed, max_speed), -max_speed)
    right_speed = max(min(right_speed, max_speed), -max_speed)

    if left_speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        pwm_left.ChangeDutyCycle(left_speed)
    elif left_speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        pwm_left.ChangeDutyCycle(-left_speed)
    else:
        pwm_left.ChangeDutyCycle(0)

    if right_speed > 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_right.ChangeDutyCycle(right_speed)
    elif right_speed < 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_right.ChangeDutyCycle(-right_speed)
    else:
        pwm_right.ChangeDutyCycle(0)

def adjust_brightness_contrast(image, brightness=30, contrast=30):
    alpha = contrast / 127 + 1
    beta = brightness
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

def find_contours_and_centers(image, lower_bound, upper_bound, min_area=1000):
    mask = cv2.inRange(image, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]
    centers = []
    for contour in large_contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append((cx, cy))
        else:
            centers.append(None)
    return large_contours, centers

def get_center_between_two_contours(center1, center2):
    if center1 and center2:
        return ((center1[0] + center2[0]) // 2, (center1[1] + center2[1]) // 2)
    return None

def move_to_point(center_between):
    global prev_error_x, prev_error_y, integral_x, integral_y, last_time
    
    current_x, current_y = 960, 540
    now = time.time()
    dt = now - last_time
    last_time = now
    
    if dt <= 0:
        dt = 0.01
    
    error_x = center_between[0] - current_x
    error_y = center_between[1] - current_y
    
    integral_x += error_x * dt
    integral_x = max(min(integral_x, 500), -500)
    integral_y += error_y * dt
    integral_y = max(min(integral_y, 500), -500)
    
    d_error_x = (error_x - prev_error_x) / dt
    d_error_y = (error_y - prev_error_y) / dt
    
    U_x = (Kp * error_x + Ki * integral_x + Kd * d_error_x)
    U_y = (Kp * error_y + Ki * integral_y + Kd * d_error_y)
    
    U_x = max(min(U_x, 30), -30)
    
    prev_error_x = error_x
    prev_error_y = error_y
    
    motor_left = base_speed + U_x
    motor_right = base_speed - U_x
    
    motor_left = max(-100, min(max_speed, motor_left))
    motor_right = max(-100, min(max_speed, motor_right))
    
    print(f"PID terms - P: {Kp*error_x:.2f}, I: {Ki*integral_x:.2f}, D: {Kd*d_error_x:.2f}")
    print(f"Left Motor: {motor_left}%, Right Motor: {motor_right}%")
    set_motor_speeds(motor_left, motor_right)

def process_frame(frame):
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    
    frame = adjust_brightness_contrast(frame, brightness=30, contrast=30)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow_lower = np.array([35, 100, 100])
    yellow_upper = np.array([85, 255, 255])
    green_lower = np.array([20, 100, 100])
    green_upper = np.array([30, 255, 255])
    blue_lower = np.array([100, 150, 0])
    blue_upper = np.array([140, 255, 255])

    green_contours, green_centers = find_contours_and_centers(hsv, green_lower, green_upper)
    yellow_contours, yellow_centers = find_contours_and_centers(hsv, yellow_lower, yellow_upper)
    blue_contours, blue_centers = find_contours_and_centers(hsv, blue_lower, blue_upper)
    
    global robot_state, blue_lost_time
    if robot_state == "tracking_blue":
        if blue_centers:
            print("Tracking blue object on the left")
            target_point = (0, blue_centers[0][1])
            move_to_point(target_point)
        else:
            print("Blue lost - stop and start forward timer")

            global prev_error_x, integral_x
            prev_error_x = 0
            integral_x = 0

            set_motor_speeds(0, 0)
            time.sleep(0.2)

            blue_lost_time = time.time()
            robot_state = "move_forward_after_lost"
    
    elif robot_state == "move_forward_after_lost":
        if time.time() - blue_lost_time < 1:
            print("Moving forward for 1 seconds")
            set_motor_speeds(base_speed, base_speed)
        else:
            print("Start turning left")
            robot_state = "turning_left"

    elif robot_state == "turning_left":
        set_motor_speeds(-base_speed, base_speed)
        if blue_centers:
            print("Blue found again - resuming tracking")
            robot_state = "tracking_blue"

    for contours, centers, color in zip(
        [green_contours, yellow_contours, blue_contours],
        [green_centers, yellow_centers, blue_centers],
        [(0, 255, 0), (0, 255, 255), (255, 0, 0)]
    ):
        for contour, center in zip(contours, centers):
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            if center:
                cv2.circle(frame, center, 5, color, -1)
                cv2.putText(frame, f"Center: {center}", (center[0] + 10, center[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                

    if len(yellow_centers) >= 2:
        largest_yellow_contours = sorted(yellow_contours, key=cv2.contourArea, reverse=True)[:2]
        largest_yellow_centers = [cv2.moments(c) for c in largest_yellow_contours]
        largest_yellow_centers = [(int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])) 
                                for m in largest_yellow_centers if m["m00"] != 0]
        if len(largest_yellow_centers) == 2:
            center_between = get_center_between_two_contours(largest_yellow_centers[0], largest_yellow_centers[1])
            if center_between:
                cv2.circle(frame, center_between, 5, (0, 0, 255), -1)
                move_to_point(center_between)
    elif len(yellow_centers) == 1:
        move_to_point(yellow_centers[0])
    else:
        print("No yellow markers found - stopping")
        set_motor_speeds(0, 0)

    return frame

try:
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = process_frame(frame)
        cv2.imshow('PID Controller', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()