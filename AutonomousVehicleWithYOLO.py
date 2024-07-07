import math
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from ultralytics import YOLO
from RobotMove import RobotMove
from RobotLight import RobotLight
from UltrasonicSensor import UltrasonicSensor
from ServoCtrl import ServoCtrl
import settings
import cv2

# Initialize the robot components
robot_move = RobotMove()
robot_light = RobotLight()
ultrasonic_sensor = UltrasonicSensor()
servo_ctrl = ServoCtrl()

# Start the components
robot_move.start()
robot_light.start()
ultrasonic_sensor.start()
servo_ctrl.start()

# Initialize the Pi Camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Load YOLO model
model = YOLO("yolo-Weights/yolov8n.pt")

# Object classes
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"]

# Known width of the object (in meters)
KNOWN_WIDTH = 0.0856  # Example width, use your own known object's width
# Focal length in pixels (calibrate your camera to find this value)
FOCAL_LENGTH = 700  # Example value, replace with your calculated focal length

def calculate_distance(known_width, focal_length, width_in_pixels):
    """
    Calculate the distance to an object using its known width and the width in pixels.

    :param known_width: The known width of the object in meters.
    :param focal_length: The focal length of the camera in pixels.
    :param width_in_pixels: The width of the object in the image in pixels.
    :return: The distance to the object in meters.
    """
    return (known_width * focal_length) / width_in_pixels

def move_forward():
    """
    Move the robot forward.
    """
    robot_move.move(settings.MOTOR_SPEED, 'forward')

def move_backward():
    """
    Move the robot backward.
    """
    robot_move.move(settings.MOTOR_SPEED, 'backward')

def move_left():
    """
    Move the robot to the left.
    """
    servo_ctrl.turnLeft(settings.ANGLE_RATE)
    robot_move.move(settings.MOTOR_SPEED, 'forward')

def move_right():
    """
    Move the robot to the right.
    """
    servo_ctrl.turnRight(settings.ANGLE_RATE)
    robot_move.move(settings.MOTOR_SPEED, 'forward')

def stop():
    """
    Stop the robot.
    """
    robot_move.move(0, 'none')

# Allow the camera to warm up
time.sleep(0.1)

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    results = model(img, stream=True)

    # Coordinates and obstacle detection
    distances = []
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to int values

            # Calculate the width of the detected object in pixels
            width_in_pixels = x2 - x1

            # Calculate the distance to the object
            distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, width_in_pixels)
            distances.append((distance, x1, y1, x2, y2))

            # Put bounding box in the image
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Confidence
            confidence = math.ceil((box.conf[0] * 100)) / 100

            # Class name
            cls = int(box.cls[0])
            class_name = classNames[cls]

            # Object details
            org = (x1, y1 - 10)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.5
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(img, f"{class_name} {confidence:.2f} {distance:.2f}m", org, font, fontScale, color, thickness)

    # Obstacle avoidance logic
    if distances:
        closest_object = min(distances, key=lambda x: x[0])  # Find the closest object
        distance, x1, y1, x2, y2 = closest_object

        if distance < 0.2:  # Example threshold distance in meters
            center_x = (x1 + x2) // 2

            if center_x < img.shape[1] // 3:
                move_right()
            elif center_x > 2 * img.shape[1] // 3:
                move_left()
            else:
                move_backward()
        else:
            move_forward()
    else:
        move_forward()

    cv2.imshow('Webcam', img)
    rawCapture.truncate(0)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()

# Clean up robot components
robot_move.terminate()
robot_light.pause()
ultrasonic_sensor.terminate()
servo_ctrl.terminate()

robot_move.join()
ultrasonic_sensor.join()
servo_ctrl.join()
