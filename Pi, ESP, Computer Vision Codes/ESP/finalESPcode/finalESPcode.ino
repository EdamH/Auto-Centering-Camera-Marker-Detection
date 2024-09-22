#include <HardwareSerial.h>  // Library to use hardware serial communication
#include <ESP32Servo.h>      // Library to control servos on ESP32
#include "BluetoothSerial.h" // Library for Bluetooth communication

BluetoothSerial SerialBT;    // Bluetooth object
String device_name = "ESP32-BT"; // Bluetooth device name

int Sample_number = 0;       // Counter for the number of samples processed
int t_cycle = 900000;        // Time cycle (15 minutes) in milliseconds

long tic = millis();         // Start time of the cycle

String dataset;              // String to store dataset (servo positions)

bool a = 1, b = 0, c = 0;    // Flags for the power-off sequence
long st;                     // Start time for power-off
int d;                       // Time remaining after power-off

// Pin definitions
#define relay 33             // Relay control pin (used to power Pi4)
#define s_interrupt 25       // Interrupt pin for servo control
#define p_interrupt 14       // Interrupt pin for power control

// Pins for hardware serial communication
#define RX 9
#define TX 10

Servo servo1, servo2, servo3; // Servo objects

// Initial servo positions
int X = 100, Y = 90, Z = 80;  

bool calculating = 1;        // Flag to indicate if calculations are ongoing
int delta = 0;               // Time spent in a cycle

HardwareSerial HSerial(1);   // Hardware serial object (for UART1)

volatile bool servo = false; // Flag set by the interrupt for servo control
volatile bool power = false; // Flag set by the interrupt for power control

// Interrupt handler for servo control
void IRAM_ATTR servo_write() {
  servo = true;  // Set a flag to indicate the servo interrupt event
}

// Interrupt handler for power control
void IRAM_ATTR power_off() {
  power = true;  // Set a flag to indicate the power interrupt event
}

void setup() {
  SerialBT.begin(device_name);         // Initialize Bluetooth with the device name
  pinMode(relay, OUTPUT);              // Set relay pin as output
  attachInterrupt(p_interrupt, power_off, RISING); // Attach interrupt for power control
  attachInterrupt(s_interrupt, servo_write, RISING); // Attach interrupt for servo control
  Serial.begin(115200);                // Initialize serial communication
  HSerial.begin(9600, SERIAL_8N1, RX, TX); // Initialize hardware serial for communication

  // Setup servos with a 50 Hz PWM signal and attach them to corresponding pins
  servo1.setPeriodHertz(50);
  servo1.attach(17, 550, 2500);        // Attach servo1 to pin 17
  servo2.setPeriodHertz(50);
  servo2.attach(16, 550, 2500);        // Attach servo2 to pin 16
  servo3.setPeriodHertz(50);
  servo3.attach(5, 550, 2500);         // Attach servo3 to pin 5

  // Write initial servo positions
  servo1.write(X);
  servo2.write(Y);
  servo3.write(Z);

  Serial.println(millis() - tic);      // Print time since the start of the program
}

void loop() {
  // If the system is in calculating mode, process servo adjustments
  if (calculating) {
    digitalWrite(relay, HIGH);         // Turn on the relay to power external devices

    // Check if the servo interrupt flag was set
    if (servo) {
      // Check if data is available on the hardware serial
      if (HSerial.available()) {
        Serial.println("receiving");
        String data = HSerial.readStringUntil('\n');  // Read incoming data until newline character
        int ang1 = 0, ang2 = 0, ang3 = 0, s = 0;      // Variables for servo angles and checksum
        Serial.println(data);

        // Parse the angles and checksum from the received data
        sscanf(data.c_str(), "%d,%d,%d,%d", &ang1, &ang2, &ang3, &s);

        Serial.print("servoN°1=");
        Serial.println(ang1);
        Serial.print("servoN°2=");
        Serial.println(ang2);
        Serial.print("servoN°3=");
        Serial.println(ang3);

        // Check the integrity of the received data
        int h = ang1 + 2 * ang2 + 3 * ang3;
        if (h - s == 0) { // If checksum is correct
          // Adjust servo positions
          X = X - ang1;
          Y = Y - ang2;
          Z = Z - ang3;

          Serial.print("x=");
          Serial.println(X);
          Serial.print("Y=");
          Serial.println(Y);
          Serial.print("Z=");
          Serial.println(Z);

          // Update servo positions if within valid range
          if (X > 60) {
            servo1.write(X);
          } else {
            Serial.println("x<60");
          }

          servo2.write(Y);

          if (Z > 60) {
            servo3.write(Z);
          } else {
            Serial.println("z<60");
          }

          // Send acknowledgment to the sender
          HSerial.println("received");
          Sample_number++;

          // Update dataset with the new positions
          String rotation = "";
          if (dataset.length() == 0) {
            rotation = String(X) + "," + String(Y) + "," + String(Z) + "\r";
          } else {
            rotation = "|" + String(X) + "," + String(Y) + "," + String(Z) + "\r";
          }
          dataset = dataset + rotation;

          // Clear the input buffer
          HSerial.readStringUntil('\n');
        } else {  // If checksum is incorrect
          HSerial.println("resend");
        }
      } else {  // If no serial data is available
        Serial.println("serial not available");
        HSerial.println("resend");
      }

      servo = false;  // Reset the servo interrupt flag
      HSerial.readStringUntil('\n');  // Clear any remaining data
    }

    // Check if the power interrupt flag was set
    if (power) {
      Serial.println("shutting down");
      calculating = 0;  // Stop calculations
      if (a) {          // First stage of power-off sequence
        st = millis();
        a = 0;
        b = 1;
      }
      if (b) {          // Wait for 4 seconds before powering off
        if (millis() - st >= 4000) {
          digitalWrite(relay, LOW);  // Turn off the relay to cut power
          delta = millis() - tic;    // Calculate the time spent in this cycle
          Serial.print("delta =");
          Serial.println(delta);
          d = t_cycle - delta;       // Calculate the remaining time for the cycle
          b = 0;
          c = 1;
          st = millis();
        }
      }
      if (c) {          // Wait for the remaining cycle time before restarting
        if (millis() - st >= d) {
          calculating = 1;  // Resume calculations after the cycle
          power = false;    // Reset the power interrupt flag
        }
      }
    }

    // Bluetooth communication: check if data is received via Bluetooth
    if (SerialBT.available()) {
      String incomingData = SerialBT.readStringUntil('\n');
      incomingData.trim();
      Serial.println("Received: " + incomingData);

      // If the received data is "a", send the elapsed time and dataset
      if (incomingData == "a") {
        long t = millis() - tic;  // Calculate the elapsed time
        Serial.println(t);
        String msg = String(t) + "#";  // Send the time

        int len = msg.length();
        char byteArray[len + 1];
        msg.getBytes((unsigned char*)byteArray, len + 1);
        SerialBT.write((uint8_t*)byteArray, len);
        delay(50);

        // Send the dataset of servo rotations
        msg = dataset + "\r";
        len = msg.length();
        char byteArray1[len + 1];
        msg.getBytes((unsigned char*)byteArray1, len + 1);
        SerialBT.write((uint8_t*)byteArray1, len);
      }
    }
  }
}
