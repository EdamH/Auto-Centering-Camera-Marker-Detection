import cv2

def main():
    # Initialize the USB camera (0 selects the default camera)
    cap = cv2.VideoCapture(0)

    # Check if the camera is successfully opened
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Capture a single frame from the camera
        ret, frame = cap.read()
        
        # If frame capture failed, exit the loop
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Display the captured frame in a window
        cv2.imshow('USB Camera Stream', frame)

        # Wait for 1ms between frames and check if 'q' key is pressed to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera resource and close the display window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
