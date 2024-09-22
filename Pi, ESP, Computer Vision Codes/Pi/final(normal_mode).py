import os
import cv2
import math
import numpy as np
from time import sleep
from itertools import combinations
from picamera2 import Picamera2
import random
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import serial

epsilon=1 #the minimum deviatu=ion that can be made by servo-motors

# Initialize a flag to check if data needs to be resent
resend = False

# Initialize the serial connection for communication with another device
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

# Set up GPIO pins for controlling hardware (e.g., servos)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(23, GPIO.OUT)  # Pin for sending data
GPIO.setup(24, GPIO.OUT)  # Pin for power control

def send(ang1, ang2, ang3):
    """
    Send the angles as a formatted message over serial communication.
    """
    message = f"{ang1},{ang2},{ang3},{ang1+2*ang2+3*ang3}\n"  # Create a message with angles
    GPIO.output(23, GPIO.HIGH)  # Signal the start of transmission
    ser.write(message.encode())  # Encode and send the message
    GPIO.output(23, GPIO.LOW)  # Signal the end of transmission
  
def Send(thetaX, thetaY, thetaZ):
    """
    Send the calculated angles and wait for acknowledgment.
    Resend if data is not valid.
    """
    print("Starting transmission")
    while True:
        try:
            line = ''
            send(int(thetaX), int(thetaY), int(thetaZ))  # Send the angles
            sleep(0.3)  # Wait for acknowledgment
            line = ser.readline().decode('utf-8').rstrip()  # Read the response
            print(line)
            if line == "received":
                print("Data received successfully")
                break  # Exit loop if data is received
            elif line == "resend":
                print("Data was not valid, resending...")
                ser.flush()  # Clear serial buffer
            else:
                print(f"Unexpected response: {line}")
                ser.flush()  # Clear serial buffer
        except UnicodeDecodeError:
            print("Failed to decode the received data. It might not be in UTF-8.")
            ser.flush()  # Clear serial buffer
    print("Transmission complete")

def find_central_pixel_of_objects(contours, image):
    """
    Find and return the central pixels of detected objects with 12 corners.
    """
    central_pixels = []
    cross_widths = []
    for contour in contours:
        # Approximate the contour for better shape representation
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the approximated contour has exactly 12 corners
        if len(approx) == 12:
            M = cv2.moments(contour)  # Calculate moments for central pixel
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                central_pixels.append((cX, cY))

                # Calculate bounding box dimensions
                x, y, w, h = cv2.boundingRect(contour)
                cross_widths.append(w)

                # Draw bounding rectangle and center point on the image
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)  # Center point in blue

    return image, central_pixels, cross_widths

def draw_perpendicular_lines(image, point):
    """
    Draw vertical and horizontal lines through a given point on the image.
    """
    height, width = image.shape[:2]
    cX, cY = point

    # Draw vertical and horizontal lines
    cv2.line(image, (cX, 0), (cX, height), (0, 255, 255), 2)  # Yellow vertical line
    cv2.line(image, (0, cY), (width, cY), (0, 255, 255), 2)  # Yellow horizontal line

def find_closest_corners(corners):
    """
    Find and return the two closest corners from a list of corners.
    """
    if len(corners) < 2:
        return None, None
    sorted_corners = sorted(corners, key=lambda corner: corner[1])  # Sort by y-coordinate

    min_distance = float('inf')
    x1, y1 = sorted_corners[0]
    closest_corner = None

    for (x2, y2) in sorted_corners[1:4]:  # Check only the next three corners
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  # Calculate distance
        if distance < min_distance:
            min_distance = distance
            closest_corner = (x2, y2)

    return sorted_corners[0], closest_corner

def find_corners_of_objects(contours):
    """
    Find and return the corners of detected objects with 12 corners.
    """
    corners = []
    for contour in contours:
        epsilon = 0.03 * cv2.arcLength(contour, True)  # Set approximation accuracy
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the approximated contour has exactly 12 corners
        if len(approx) == 12:
            for point in approx:
                corners.append(tuple(point[0]))  # Add corners to the list
    return corners
def process_image(image_path, plot=False):
    # Load the image in grayscale (assuming image_path is already a BGR image)
    image = cv2.cvtColor(image_path, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian Blur to reduce noise in the image
    blurred_image = cv2.GaussianBlur(image, (5, 5), 1.4)

    # Apply Canny edge detection to identify edges in the image
    edges = cv2.Canny(blurred_image, 50, 150)
    
    # Dilate the edges to close gaps and make contours more defined
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    
    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Identify the central pixel coordinates of detected objects with 12 corners
    image, central_pixels, cross_widths = find_central_pixel_of_objects(contours, image)

    # Create a color image from the grayscale image for drawing
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    
    # Calculate the center coordinates of the image
    height, width = image.shape
    center_image = (width // 2, height // 2)

    # Draw a red circle at the center of the image
    cv2.circle(color_image, center_image, 10, (0, 0, 255), -1)  
    
    # Find corners of the detected objects
    corners = find_corners_of_objects(contours)

    # Identify the two closest corners to each other
    corner1, corner2 = find_closest_corners(corners)

    # If two corners are found, draw a line between them and calculate the angle
    if corner1 is not None and corner2 is not None:
        cv2.line(color_image, corner1, corner2, (255, 0, 0), 2)  # Draw a line in blue

        # Calculate the angle of the line with respect to the horizontal axis
        dx = corner2[0] - corner1[0]
        dy = corner2[1] - corner1[1]
        angle_radians = np.arctan2(dy, dx)
        angle_degrees = np.abs(np.degrees(angle_radians))
        print("angle_degrees= ", angle_degrees)

        # Calculate ThetaY based on the angle
        if int(angle_degrees) in range(45, 135):
            ThetaY = 90 - angle_degrees
        elif int(angle_degrees) in range(135, 180):
            ThetaY = 180 - angle_degrees
        elif int(angle_degrees) in range(0, 45):
            ThetaY = -angle_degrees
    else:
        print("Not enough corners detected to calculate deviation.")

    # Draw perpendicular lines at the central pixels and mark them
    cross_pixels = []
    for pixel in central_pixels:
        draw_perpendicular_lines(color_image, pixel)
        cv2.circle(color_image, pixel, 7, (0, 255, 0), -1)  # Mark central pixel in green
        cross_pixels.append(pixel)
    
    if plot:
        # Display the images with detected features
        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.title('Original Image with Perpendicular Lines and Center')
        plt.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
        plt.axis('off')

        plt.subplot(1, 2, 2)
        plt.title('Canny Edges')
        plt.imshow(edges, cmap='gray')
        plt.axis('off')

        plt.tight_layout()
        plt.show()

    # Check if any cross pixels were found
    if not cross_pixels:
        print("No crosses detected.")
        return
    
    # Return deviations from the center along with ThetaY
    return [(cross[0] - center_image[0], cross[1] - center_image[1], float(ThetaY)) for cross in cross_pixels], cross_widths

def pixel_2_meter(value_in_pixel, ratio):
    # Convert pixel values to meters based on a given ratio
    return value_in_pixel * ratio

def compute_angle_difference(offset, distance_between_poles):
    # Calculate the angular difference based on the offset and distance
    return offset / distance_between_poles

def pixel_2_degree(value_in_pixel, ratio, distance_between_poles):
    # Convert pixel values to degrees using the ratio and distance
    return value_in_pixel * ratio * (180 / np.pi) / distance_between_poles

def display_results(cross_widths, diff_x, diff_y):
    # Calculate the ratio of real-world width to pixel width
    ratio = 175 / cross_widths[0]

    # Convert pixel differences to meters
    diff_x_in_meters = pixel_2_meter(diff_x, ratio)
    diff_y_in_meters = pixel_2_meter(diff_y, ratio)

    # Compute angle differences
    angle_x = compute_angle_difference(diff_x_in_meters, 1470)
    angle_y = compute_angle_difference(diff_y_in_meters, 1470)

    # Convert angles to degrees
    angle_x_degrees = angle_x * (180 / np.pi)
    angle_y_degrees = angle_y * (180 / np.pi)

    # Print the differences and calculated angles
    print(f"Difference: (dx: {diff_x}, dy: {diff_y})")
    print(f"Difference in meters: (dx: {diff_x_in_meters}, dy: {diff_y_in_meters})")
    print(f"Angles: (dx: {angle_x}, dy: {angle_y})")
    print(f"Angles in degrees: (ThetaZ: {angle_x_degrees}, ThetaX: {angle_y_degrees})")

# ________________________________________________________________________________________

# Constants for distance and width in the real world
DISTANCE_BETWEEN_POLES = 580  # Distance between poles in mm
CROSS_WIDTH_IN_METERS = 175    # Width of the cross in mm

# Flush serial buffer
ser.flush() 

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Capture a frame from the camera
frame = picam2.capture_array()
# Save the captured frame
cv2.imwrite('Before.jpg', frame)


while True:
    cross_devs, cross_widths = process_image(frame, False)  # Process the image for crosses

    # Check if any cross widths were detected
    if len(cross_widths) != 0:
        ratio = CROSS_WIDTH_IN_METERS / cross_widths[0]

    # Convert detected cross deviations to degrees
    cross_devs_degree = [
        (
            pixel_2_degree(cross_dev[0], ratio, DISTANCE_BETWEEN_POLES), 
            pixel_2_degree(cross_dev[1], ratio, DISTANCE_BETWEEN_POLES),
            cross_dev[2] 
        )
        for cross_dev in cross_devs
    ]

    # Unpack the angles
    thetaZ, thetaX, thetaY = cross_devs_degree[0]

    print(thetaX, thetaY, thetaZ)  # Print the angles

    # Send the angles to the appropriate output (e.g., serial)
    Send(round(thetaX), round(thetaY), round(thetaZ))
    print("Sent data", round(thetaX), round(thetaY), round(thetaZ))
    if(round(thetaX)<=epsilon and round(thetaY)<=epsilon and round(thetaZ)<=epsilon):
        break

# Wait before capturing the next frame
sleep(1)
frame = picam2.capture_array()  # Capture another frame
cv2.imwrite('After.jpg', frame)  # Save the captured frame

# Close the serial connection
ser.flush() 
ser.close()  
print("Power off sequence")

# Signal to power off
GPIO.output(24, GPIO.HIGH)
sleep(0.5)
GPIO.output(24, GPIO.LOW)
print("Shutting down")
os.system("sudo poweroff")  # Shut down the system
