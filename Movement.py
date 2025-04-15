import cv2
import numpy as np
import RPi.GPIO as GPIO

Kp = 0.05
Kd = 0.01

base_speed = 50
max_speed = 1.5

prev_error_x = 0
prev_error_y = 0

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

pwm_left = GPIO.PWM(ENA, max_speed)
pwm_right = GPIO.PWM(ENB, max_speed)
pwm_left.start(0)
pwm_right.start(0)

def set_motor_speeds(left_speed, right_speed):
    left_speed = left_speed // max_speed
    right_speed = right_speed // max_speed
    if left_speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        pwm_left.ChangeDutyCycle(min(left_speed, 100))
    elif left_speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        pwm_left.ChangeDutyCycle(min(-left_speed, 100))
    else:
        pwm_left.ChangeDutyCycle(0)

    if right_speed > 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_right.ChangeDutyCycle(min(right_speed, 100))
    elif right_speed < 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_right.ChangeDutyCycle(min(-right_speed, 100))
    else:
        pwm_right.ChangeDutyCycle(0)

def adjust_brightness_contrast(image, brightness=10, contrast=30):
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
    global prev_error_x
    current_x = 960
    center_threshold = 150
    error_x = center_between[0] - current_x
    if abs(error_x) <= center_threshold:
        set_motor_speeds(base_speed, base_speed)
        return
    d_error_x = error_x - prev_error_x
    U_x = Kp * error_x + Kd * d_error_x
    prev_error_x = error_x
    motor_left = base_speed + U_x
    motor_right = base_speed - U_x
    motor_left = max(0, min(100, motor_left))
    motor_right = max(0, min(100, motor_right))
    set_motor_speeds(motor_left, motor_right)

def get_distance_to_buoy(center):
    return 25

def rotate_around_buoy(center):
    set_motor_speeds(base_speed, -base_speed)

def is_buoy_on_right(center):
    image_center_x = frame.shape[1] // 2
    return center[0] > image_center_x

def process_frame(frame):
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    frame = adjust_brightness_contrast(frame, brightness=30, contrast=30)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow_lower = np.array([40, 40, 20])
    yellow_upper = np.array([91, 255, 140])
    green_lower = np.array([20, 100, 80])
    green_upper = np.array([30, 255, 255])
    blue_lower = np.array([100, 150, 0])
    blue_upper = np.array([140, 255, 255])

    green_contours, green_centers = find_contours_and_centers(hsv, green_lower, green_upper)
    yellow_contours, yellow_centers = find_contours_and_centers(hsv, yellow_lower, yellow_upper)
    blue_contours, blue_centers = find_contours_and_centers(hsv, blue_lower, blue_upper)

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
                cv2.putText(frame, f"Top-left: ({x}, {y})", (x, y - 10), 
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

    return frame

cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = process_frame(frame)
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()
