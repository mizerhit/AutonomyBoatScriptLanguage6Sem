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
        center_between = ((center1[0] + center2[0]) // 2, (center1[1] + center2[1]) // 2)
        return center_between
    return None

def move_to_point(center_between):
    global prev_error_x, prev_error_y

    current_x, current_y = 960, 1000

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

def process_frame(frame):
    frame = cv2.GaussianBlur(frame, (5, 5), 0)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_lower = np.array([40, 40, 20])
    green_upper = np.array([91, 255, 255])

    yellow_lower = np.array([20, 100, 80])
    yellow_upper = np.array([30, 255, 255])

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
                cv2.putText(frame, f"Center: {center}", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(frame, f"Top-left: ({x}, {y})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    if len(yellow_centers) >= 2:
        largest_yellow_contours = sorted(yellow_contours, key=cv2.contourArea, reverse=True)[:2]
        largest_yellow_centers = [cv2.moments(c) for c in largest_yellow_contours]
        largest_yellow_centers = [(int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])) for m in largest_yellow_centers if m["m00"] != 0]
        if len(largest_yellow_centers) == 2:
            center_between = get_center_between_two_contours(largest_yellow_centers[0], largest_yellow_centers[1])
            if center_between:
                cv2.circle(frame, center_between, 5, (0, 0, 255), -1)
                move_to_point(center_between)

    return frame

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = process_frame(frame)

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop_motors()
        break

cap.release()
cv2.destroyAllWindows()