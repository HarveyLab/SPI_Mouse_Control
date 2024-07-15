void setup() {
  // Initialize the serial communication at a baud rate of 9600
  Serial.begin(9600);

  // Wait for the serial port to connect (useful for Leonardo, Teensy, etc.)
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // Static variable to keep track of the counter
  static unsigned long counter = 1;

  // Send the counter value over the serial port
  Serial.println(counter);

  // Increment the counter
  counter++;

  // Add a small delay to avoid flooding the serial port
  delay(10); // 1 second delay
}
