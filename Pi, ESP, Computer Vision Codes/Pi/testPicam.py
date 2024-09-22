import cv2
from picamera2 import Picamera2

# Initialize and configure the Raspberry Pi Camera
picam2 = Picamera2()  # Create a Picamera2 object
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))  # Set resolution to 640x480
picam2.start()  # Start the camera

while True:
    # Capture the current frame from the camera as an array
    frame = picam2.capture_array()

    # Display the captured frame in a window titled 'Night Vision Camera Stream'
    cv2.imshow('Night Vision Camera Stream', frame)

    # Check if 'q' key is pressed, and exit the loop if it is
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Close all OpenCV windows after exiting the loop
cv2.destroyAllWindows()
