import cv2
import math
import numpy as np

# Start webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Provide absolute paths to the model files
weights_path = "yolov3.weights"
config_path = "yolov3.cfg"
names_path = "coco.names"

# Load YOLO model
net = cv2.dnn.readNet(weights_path, config_path)
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Load object classes
classNames = []
with open(names_path, "r") as f:
    classNames = [line.strip() for line in f.readlines()]

# Known width of the object (in meters)
KNOWN_WIDTH = 0.0856  # Example width, use your own known object's width
# Focal length in pixels (calibrate your camera to find this value)
FOCAL_LENGTH = 700  # Example value, replace with your calculated focal length

def calculate_distance(known_width, focal_length, width_in_pixels):
    return (known_width * focal_length) / width_in_pixels

def move_forward():
    print("Moving forward")

def move_backward():
    print("Moving backward")

def move_left():
    print("Moving left")

def move_right():
    print("Moving right")

def stop():
    print("Stopping")

while True:
    success, img = cap.read()
    height, width, channels = img.shape

    # Prepare the image for the model
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Coordinates and obstacle detection
    class_ids = []
    confidences = []
    boxes = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    distances = []

    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            class_id = class_ids[i]
            label = str(classNames[class_id])
            confidence = confidences[i]

            # Calculate the distance to the object
            distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, w)
            distances.append((distance, x, y, x + w, y + h))

            # Draw the bounding box and label
            color = (255, 0, 0)
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, f"{label} {confidence:.2f} {distance:.2f}m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

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
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
