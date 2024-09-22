from time import sleep
import serial
import random

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

for i in range(10):
    X=random.randint(10, 50)
    Send(X,3*X-5,5*X-30)
# Close the serial connection
ser.flush() 
ser.close() 

