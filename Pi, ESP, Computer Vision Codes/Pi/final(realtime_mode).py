
import cv2
import math
import numpy as np
from time import sleep, time
from itertools import combinations
from picamera2 import Picamera2
import random
import RPi.GPIO as GPIO
import serial

# Flag for resend status
resend = False

# Initialize the serial connection for communication with another device
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

# Set up GPIO pins for control signals
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(23, GPIO.OUT)  # Output pin for transmission signal
GPIO.setup(24, GPIO.OUT)  # Not used in this code

def send_one(ang1, ang2, ang3):
    # Construct the message string with angles and a checksum
    message = f"{ang1},{ang2},{ang3},{ang1 + ang2 + ang3}\n"
    print("Starting transmission")
    GPIO.output(23, GPIO.HIGH)  # Set transmission pin high
    ser.write(message.encode())  # Send the message as bytes
    GPIO.output(23, GPIO.LOW)  # Set transmission pin low
    print("Transmission complete")

def send(ang1, ang2, ang3):
    # Send the angles and wait for a response
    send_one(ang1, ang2, ang3)
    while True:
        try:
            line = ser.readline().decode('utf-8').rstrip()  # Read the response
            print(line)
            if line == "received":
                print("Data received successfully")
                break  # Exit loop if data was received
            elif line == "resend":
                ser.flush()  # Clear the serial buffer
                print("Data was not valid, resending...")
                send_one(int(ang1), int(ang2), int(ang3))  # Resend data
                sleep(0.1)
            else:
                ser.flush()  # Clear the buffer for unexpected responses
                print(f"Unexpected response: {line}")
        except UnicodeDecodeError:
            print("Failed to decode the received data. It might not be in UTF-8.")
            line = ''  # Reset line if decode fails

# Function to find the central pixel of detected objects
def find_central_pixel_of_objects(contours, image):
    central_pixels = []  # List to hold central pixel coordinates
    cross_widths = []   # List to hold widths of detected objects
    corners = []        # List to hold corner points of contours
    for contour in contours:
        # Approximate the contour to get straight edges
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 12:  # Check for 12-cornered objects
            # Calculate moments for center coordinates
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                central_pixels.append((cX, cY))

                # Get bounding box dimensions
                x, y, w, h = cv2.boundingRect(contour)
                cross_widths.append(w)

                # Draw bounding rectangle on the image
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Draw the center point
                cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
            for point in approx:
                corners.append(tuple(point[0]))  # Collect corners

    return image, central_pixels, cross_widths, corners

# Function to draw perpendicular lines through a point
def draw_perpendicular_lines(image, point):
    height, width = image.shape[:2]
    cX, cY = point
    # Draw vertical and horizontal lines
    cv2.line(image, (cX, 0), (cX, height), (0, 255, 255), 2)  # Yellow vertical line
    cv2.line(image, (0, cY), (width, cY), (0, 255, 255), 2)  # Yellow horizontal line

def find_closest_corners(corners):
    if len(corners) < 2:
        return None, None  # Not enough corners to compare

    # Sort corners based on their y-coordinates
    sorted_corners = sorted(corners, key=lambda corner: corner[1])
    min_distance = float('inf')
    x1, y1 = sorted_corners[0]
    closest_corner = None

    # Check distances between the first corner and the next three corners
    for (x2, y2) in sorted_corners[1:4]:
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if distance < min_distance:
            min_distance = distance
            closest_corner = (x2, y2)

    return sorted_corners[0], closest_corner

def process_image(image_path, plot=False):
    # Load the image in grayscale
    image = cv2.cvtColor(image_path, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(image, (5, 5), 1.4)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred_image, 50, 150)

    # Dilate edges to strengthen the contours
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find central pixels and other properties of detected objects
    image, central_pixels, cross_widths, corners = find_central_pixel_of_objects(contours, image)

    # Create a color copy of the grayscale image to draw on
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    
    # Calculate the center of the image
    center_image = (640 // 2, 480 // 2)

    # Draw the center of the image
    cv2.circle(color_image, center_image, 10, (0, 0, 255), -1)  # Red circle at image center
    
    # Find the two closest corners
    corner1, corner2 = find_closest_corners(corners)

    # If two corners were found, draw a line and calculate the angle
    if corner1 is not None and corner2 is not None:
        cv2.line(color_image, corner1, corner2, (255, 0, 0), 2)  # Red line between corners

        # Calculate angle with respect to the horizontal axis
        dx = corner2[0] - corner1[0]
        dy = corner2[1] - corner1[1]
        angle_radians = np.arctan2(dy, dx)
        angle_degrees = np.degrees(angle_radians)
        print("angle_degrees= ", angle_degrees)
        ThetaY = 0
        # Determine the angle ThetaY based on the calculated angle
        if int(angle_degrees) in range(45, 135):
            ThetaY = 90 - angle_degrees
        elif int(angle_degrees) in range(135, 180):
            ThetaY = 180 - angle_degrees
        elif int(angle_degrees) in range(0, 45):
            ThetaY = -angle_degrees
    else:
        print("Pas assez de coins détectés pour calculer la déviation.")  # Not enough corners

    # Draw perpendicular lines for detected central pixels
    cross_pixels = []
    for pixel in central_pixels:
        draw_perpendicular_lines(color_image, pixel)
        cv2.circle(color_image, pixel, 7, (0, 255, 0), -1)  # Green circle for detected central pixel
        cross_pixels.append(pixel)

    # If plotting is enabled, display the images
    if plot:
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
        for (x, y) in corners:
            plt.scatter(x, y, color='red')
        plt.show()

    # Handle the case when no contours are found
    if not cross_pixels:
        print("Aucune croix détectée.")  # No crosses detected
        return False, False
    
    return [(cross[0] - center_image[0], cross[1] - center_image[1], float(ThetaY)) for cross in cross_pixels], cross_widths

def pixel_2_meter(value_in_pixel, ratio):
    # Convert pixel value to meters based on a given ratio
    return value_in_pixel * ratio
def compute_angle_difference(offset, distance_between_poles):
    # Calculate the angle difference based on the offset and distance between poles
    return offset / distance_between_poles

def pixel_2_degree(value_in_pixel, ratio, distance_between_poles):
    # Convert pixel value to degrees using the specified ratio and distance
    return value_in_pixel * ratio * (180 / np.pi) / distance_between_poles

# ________________________________________________________________________________________
tic = time()  # Start timing
DISTANCE_BETWEEN_POLES = 765  # Distance between poles in meters
CROSS_WIDTH_IN_METERS = 175  # Width of the cross in meters

ser.flush()  # Clear the serial buffer

# Initialize the camera
picam2 = Picamera2()  # Create a camera object
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))  # Set the camera resolution
picam2.start()  # Start the camera

# Capture the initial frame from the camera
frame = picam2.capture_array()
cv2.imwrite('Before.jpg', frame)  # Save the captured frame

# Process the captured image to find cross deviations and widths
cross_devs, cross_widths = process_image(frame, False)

# Loop until valid cross widths and deviations are detected
while not cross_widths or not cross_devs:
    # If no valid data, send a default signal
    send(0, 0, 5)
    sleep(0.1)  # Wait briefly
    frame = picam2.capture_array()  # Capture another frame
    cross_devs, cross_widths = process_image(frame, False)  # Process the new frame

# Save the last captured frame after processing
cv2.imwrite('afterbalayage.jpg', frame)

# Calculate the ratio based on the first detected cross width
ratio = CROSS_WIDTH_IN_METERS / cross_widths[0]

# Continuous loop to adjust angles until convergence is achieved
while True:
    thetaZ, thetaX, thetaY = 0, 0, 0  # Initialize angles to zero

    # Check if any cross deviations were found
    if cross_devs:
        # Convert pixel deviations to degrees
        cross_devs_degree = [ 
            (
                pixel_2_degree(cross_dev[0], ratio, DISTANCE_BETWEEN_POLES), 
                pixel_2_degree(cross_dev[1], ratio, DISTANCE_BETWEEN_POLES),
                cross_dev[2]  # Retain original angle
            )
            for cross_dev in cross_devs
        ]

        # Get angles from the first detected cross deviation
        thetaZ, thetaX, thetaY = cross_devs_degree[0]

    print(thetaZ, thetaX, thetaY)  # Output the angles

    # Send adjusted angles to the system
    send(int(thetaX) + np.sign(int(thetaX)), int(thetaY) + np.sign(int(thetaY)), int(thetaZ) + np.sign(int(thetaZ)))

    print(int(thetaX), ",", int(thetaY), ",", int(thetaZ))  # Log sent angles

    sleep(0.1)  # Brief pause
    ser.flush()  # Clear the serial buffer
    sleep(0.1)  # Brief pause

    # Capture a new frame for processing
    frame = picam2.capture_array()
    cross_devs, cross_widths = process_image(frame, False)  # Process the new frame
    
    # Check for convergence of the angles
    if abs(thetaZ) < 1 and abs(thetaX) < 1 and abs(thetaY) < 1:
        print("CONVERGED!")  # Log if converged
        break  # Exit the loop

try:
    # Continuous monitoring loop
    while True:
        frame = picam2.capture_array()  # Capture a new frame
        cross_devs, cross_widths = process_image(frame, False)  # Process the frame
        thetaZ, thetaX, thetaY = 0, 0, 0  # Reset angles

        # Check if any cross deviations were found
        if cross_devs:
            cross_devs_degree = [ 
                (
                    pixel_2_degree(cross_dev[0], ratio, DISTANCE_BETWEEN_POLES), 
                    pixel_2_degree(cross_dev[1], ratio, DISTANCE_BETWEEN_POLES),
                    cross_dev[2]  # Retain original angle
                )
                for cross_dev in cross_devs
            ]

            # Get angles from the first detected cross deviation
            thetaZ, thetaX, thetaY = cross_devs_degree[0]
        print(thetaZ, thetaX, thetaY)  # Output the angles

        # Check if adjustments are needed based on the angle thresholds
        if abs(thetaZ) > 1 or abs(thetaX) > 1 or abs(thetaY) > 2:
            send(int(thetaX) + np.sign(int(thetaX)), int(thetaY) + np.sign(int(thetaY)), int(thetaZ) + np.sign(int(thetaZ)))
            print(int(thetaX), ",", int(thetaY), ",", int(thetaZ))  # Log sent angles
except KeyboardInterrupt:
    # Handle the exit signal
    print('INTERRUPTED!')

# Final operations after exiting the loop
sleep(1)  # Brief pause before final capture
frame = picam2.capture_array()  # Capture the final frame
cv2.imwrite('After.jpg', frame)  # Save the final frame

# Close the serial connection and clean up
ser.flush()  # Clear the serial buffer
ser.close()  # Close the serial port
