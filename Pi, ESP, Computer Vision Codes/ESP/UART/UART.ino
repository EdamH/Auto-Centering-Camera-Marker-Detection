#include <HardwareSerial.h>  // Library to use hardware serial communication

#define RX 9
#define TX 10
#define s_interrupt 25  // Interrupt pin for servo control

HardwareSerial HSerial(1);   // Hardware serial object (for UART1)
volatile bool servo = false;  // Flag set by the interrupt for servo control

// Interrupt handler for servo control
void IRAM_ATTR servo_write() {
  servo = true;  // Set a flag to indicate the servo interrupt event
}

void setup() {
  Serial.begin(115200);                // Initialize serial communication
  HSerial.begin(9600, SERIAL_8N1, RX, TX); // Initialize hardware serial for communication
  attachInterrupt(s_interrupt, servo_write, RISING); // Attach interrupt for servo control
}

void loop() {
  // Check if the servo interrupt flag was set
  if (servo) {
    // Check if data is available on the hardware serial
    if (HSerial.available()) {
      String data = HSerial.readStringUntil('\n');  // Read incoming data until newline character
      int ang1 = 0, ang2 = 0, ang3 = 0, s = 0;      // Variables for servo angles and checksum

      // Parse the angles and checksum from the received data
      sscanf(data.c_str(), "%d,%d,%d,%d", &ang1, &ang2, &ang3, &s);

      // Check the integrity of the received data
      int h = ang1 + 2 * ang2 + 3 * ang3;
      if (h==s) { // If checksum is correct
        // Send acknowledgment to the sender
        HSerial.println("received");
        Serial.println(String(ang1) + "  " + String(ang2) + "  " + String(ang3));
      } else {  // If checksum is incorrect
        HSerial.println("resend");
      }
    } else {  // If no serial data is available
      HSerial.println("resend");
    }

    servo = false;  // Reset the servo interrupt flag
  }
}
