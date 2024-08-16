#include <ODriveArduino.h>

// ODrive serial port
#define SERIAL_PORT Serial1

// ODrive object
ODriveArduino odrive(SERIAL_PORT);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Connect to ODrive
  if (!odrive.begin()) {
    Serial.println("Failed to connect to ODrive");
    while (1) {
      delay(1000);
    }
  }

  // Set motor velocity
  odrive.SetVelocity(0, 1000); // Motor 0, velocity 1000 counts/s
}

void loop() {
  // Read motor velocity
  float velocity = odrive.GetVelocity(0); // Motor 0

  // Print motor velocity
  Serial.print("Motor velocity: ");
  Serial.println(velocity);

  // Delay for 1 second
  delay(1000);
}
