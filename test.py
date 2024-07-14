import cv2
import numpy as np

# Define the lower and upper bounds for the red color in HSV
# Red color can have two ranges in HSV space
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])



def detect_cones(frame):
    """
    Detect cones in the given frame.
    
    Args:
        frame (numpy.ndarray): The frame from the camera.
    
    Returns:
        list: List of contours representing the cones.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for the red color
    mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
    mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
    mask = mask1 | mask2
    
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def get_center(contour):
    """
    Get the center of the given contour.
    
    Args:
        contour (numpy.ndarray): The contour of the cone.
    
    Returns:
        tuple: The (x, y) coordinates of the center of the contour.
    """
    M = cv2.moments(contour)
    if M["m00"] > 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    else:
        return None

def get_direction(cone_centers, frame_width):
    """
    Determine the direction based on the position of the cones.
    
    Args:
        cone_centers (list): List of (x, y) coordinates of the cone centers.
        frame_width (int): The width of the frame.
    
    Returns:
        str: The direction to move (forward, left, right, back).
    """
    if not cone_centers:
        return 'back'
    
    avg_x = sum([center[0] for center in cone_centers]) / len(cone_centers)
    
    if avg_x < frame_width * 0.4:
        return 'left'
    elif avg_x > frame_width * 0.6:
        return 'right'
    else:
        return 'forward'


def drive(direction):
    """
    Drive the robot in the given direction.
    
    Args:
        direction (str): The direction to move (forward, left, right, back).
    """
    if direction == 'forward':
        print("forward")
        # robot_move.move(speed_set, 'forward')
    elif direction == 'left':
        print("left")
        # robot_move.turn_left()
    elif direction == 'right':
        print("right")
        # robot_move.turn_right()
    elif direction == 'back':
        print("backward")
        # robot_move.move(speed_set, 'backward')

# if __name__ == "__main__":
#     main()
