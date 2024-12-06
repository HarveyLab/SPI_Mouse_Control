/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html
 
   This example code is in the public domain.
*/

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;
const int valve1 = 22;
const int valve2 = 23;

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  pinMode(valve1, OUTPUT);
  pinMode(valve2, OUTPUT);
}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {
  digitalWrite(ledPin, HIGH);   // set the LED on
  digitalWrite(valve1, HIGH);   // set the LED on
  digitalWrite(valve2, HIGH);   // set the LED on

  delay(100);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  digitalWrite(valve1, LOW);   // set the LED on
  digitalWrite(valve2, LOW);   // set the LED on
  delay(1000);                  // wait for a second
}

