// Thanks to Rachel De Barros and this video for inspiration: How to Control a Servo with an Ultrasonic Sensor and Arduino + Code ( https://www.youtube.com/watch?v=ybhMIy9LWFg )
#include <Servo.h>

// Pin Connections
const int   servoPin  = 7; 
const int   trigPin = 9; 
const int   echoPin= 10; 

// Values for calculation
const float speedOfSound = 0.034; // In Microseconds
const float cm2in = 2.54; // CMs in an Inch
const unsigned long msPerTurn = 2000; // The number of milliseconds needed to complete a full turn (360 degrees)

// Calculating distance to subject
float duration; // Variable to store pulse duration
float distanceCM; // Variable to store distance in CM
float distanceIN; // Variable to store distance in IN
float distanceIN_prev; // Variable to store previously calculated distance; 


// Position of servo
const float open_pt; // The number of turns needed to be fully open, calculate by spinning the curtain rod from fully closed to fully open
const float close_pt = 0.0; // Fully closed position
float current_pos = 0.0;

Servo curtainRodServo; 

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach servoPin to our Curtain Rod Servo 
  curtainRodServo.attach(servoPin);
}

void loop() {
  // Start with a clean signal
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send trigger signal to activate ultrasound transmitter
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Must be high for 10 seconds to active transmission
  digitalWrite(trigPin, LOW);

  // Grab pulse duration in microseconds
  // If set to HIGH, pulseIn() waits for the pin to go from LOW to HIGH
  // Timing stops when pin goes back to LOW
  duration = pulseIn(echoPin, HIGH);

  // Distance in centimeters. Calculate how long it took sound to travel
  // Divide by 2 so that we only get the distance coming back from source
  distanceCM = (duration * speedOfSound) /2;

  // Convert to inches
  distanceIN = distanceCM * cm2in;

  // Print Distance
  Serial.print("Distance: ");
  Serial.print(distanceCM);
  Serial.print("cm |  ");
  Serial.print(distanceIN);
  Serial.print(" in");

  // Calculate 

  // Map distances in inches to servo position in degrees
  // This essentially neatly maps 1 to 12 inches => 0 to 180 degrees
  // int servoPos = map(distanceIN, 1, 12, 0, 180);
  
  //A continuous servo instead moves at a certain speed
  // 0 being full speed in one direction, 180 being full speed in the other direction, 90 being motionless, with gradients in between
  curtainRodServo.write(servoPos);
  distanceIN_prev = distanceIN;

  // Wait the required milliseconds to make a full turn
  delay(msPerTurn);
}
