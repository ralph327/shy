// Thanks to Rachel De Barros and this video for inspiration: How to Control a Servo with an Ultrasonic Sensor and Arduino + Code ( https://www.youtube.com/watch?v=ybhMIy9LWFg )
#include <Servo.h>

// Debugging
const bool Debug = true;

// Pin Connections
const int servoPin = 7; 
const int  trigPin = 9; 
const int  echoPin = 10; 

// Turn indexes for turnsToMake Array
enum idx_turns {WholeTurns, QuarterTurns, END};

// Direction subject is going
enum subj_dir {STILL, AWAY, TOWARDS};

// Direction to spin
enum servo_dir {OPEN, CLOSE, NOWHERE};

// Speed to spin
enum servo_speed {FULL, HALF};

/* Speeds for moving the servo
** A continuous servo moves at a certain speed in one direction
** 0 being full speed in one direction, 180 being full speed in the other direction, 90 being motionless, with gradients in between
** NOTE: Switch these depending on which direction the angle goes
*/
enum servo_move {FullSpeedOpen = 0, HalfSpeedOpen = 45, NoMovement = 90, HalfSpeedClose = 135, FullSpeedClose = 180};

/*
** Values for calculation
*/
// In Microseconds
const float speedOfSound = 0.034; 
// CMs in an Inch
const float cm2in = 2.54; 
// The number of milliseconds needed to complete a full turn (360 degrees) at full speed
const unsigned long msPerTurn = 2000; 
// The number of milliseconds needed to complete a quarter turn (90 degrees) at full speed
const unsigned long msPerQtrTurn = msPerTurn/4;  

/*
** Variables for position of servo
** NOTE: shy should start fully closed
*/ 
// The number of turns needed to be fully open, calculate by spinning the curtain rod from fully closed to fully open
const float open_pt = -1.1; 
// Fully closed position
const float close_pt = 0.0; 
// Number of milliseconds to go from fully open to fully closed or vice versa
const unsigned long total_ms_to_switch_states =  (unsigned long) (open_pt * msPerTurn);
// The current position of the blinds
float current_pos = -1;

/*
** Globals
*/ 
// Variable to store previously calculated distance
float prev_d_inches; 
// The Servo!
Servo curtainRodServo; 

/* Use ultasonic device and math to get distance
** NOTE: Max of 13ft measuring distance
*/
float GetDistance() {
  // Calculating distance to subject
  float duration; // Variable to store pulse duration
  float distanceCM; // Variable to store distance in CM
  float distanceIN; // Variable to store distance in IN

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

  /* WARNING WARNING WARNING
  ?????????????????????????????
  How is distance reported when nothing is being detected?
  Does that flip to zero or is that registered as 13ft
  ?????????????????????????????
  ** WARNING WARNING WARNING
  */

  if (Debug)
  {
    // Print Distance
    Serial.print("Distance: ");
    Serial.print(distanceCM);
    Serial.print(" cm | ");
    Serial.print(distanceIN);
    Serial.print(" in \n");
  }

  return distanceIN;
}

// Sets all turns in array to be 0
void ResetTurns(int *turnsToMake[]) {
  // Reset Turns
  for (idx_turns x = WholeTurns; x < END ; x++)
  {
    turnsToMake[x] = 0;

    if(Debug)
    {
      Serial.print("Setting index ")
      Serial.print(x);
      Serial.print(" in turnsToMake to 0 ");
    }
  }
}

// Performs an action based on system state
void Perform(float distance) {
  // To tell how subject is moving
  subj_dir going; 

  // Is subject moving closer or further away?
  float diff = prev_d_inches - distance;
  if(diff == 0)
  {
    going = STILL;
    if(Debug)
    {
      Serial.println("Subject is staying still");
    }
  }
  else if(diff < 0)
  {
    going = AWAY;
    if(Debug)
    {
      Serial.println("Subject is going away");
    }
  }
  else if (diff > 0)
  {
    going = TOWARDS;
    if(Debug)
    {
      Serial.println("Subject is coming towards");
    }
  }
  else
  {
    if(Debug)
    {
      Serial.println("ERROR: Impossible behavior detected");
    }
  }
  
  // Do something depending on where the subject is going
  switch (going) {
    case STILL:
      break;
    case AWAY:
      break;
    case TOWARDS:
      break;
    default:
      Serial.println("ERROR: Impossible behavior detected");
      if(Debug)
      {
        Serial.println(__func__);
        Serial.println("Giiiiirrrrrlllll this should not be happening");  
        Serial.print("The calculated distance is ");
        Serial.println(diff);
      }
      break;
  }
} 

// Calculate how much time it will take to do turns
void Turn(int *turnsToMake[], servo_dir dir, servo_speed spd) {
  unsigned long time_to_wait = 0;
  servo_move move = NoMovement;

   // Determine which angle to use to move in the right direction
  switch (dir)
  {
    case OPEN:
      if(spd == FULL)
      {
        move = FullSpeedOpen;
      }
      else 
      {
        move = HalfSpeedOpen;
      }
      break;
    case CLOSE:
      if(spd == FULL)
      {
        move = FullSpeedClose;
      }
      else 
      {
        move = HalfSpeedClose;
      }
      break;
    case NOWHERE:
      move = NoMovement;
      break;
    default:
        Serial.println("ERROR: Impossible behavior detected");
        if(Debug)
        {
          Serial.println(__func__);
          Serial.print("The direction is ");
          Serial.println(direction);
        }
        break;
  }

  /* Calculate how much time it will take to perform this series of turns
  ** and calculate what our current position will be at once this is performed
  */
  for (idx_turns x = WholeTurns; x < END; x++)
  {
    switch (x)
    {
      case WholeTurns:
        if(spd == FULL)
        {
          time_to_wait += turnsToMake[x] * msPerTurn;
        }
        else
        {
          time_to_wait += turnsToMake[x] * (msPerTurn / 2);
        }
        switch (dir) {
          case OPEN:
            current_pos += turnsToMake[x];
            break;
          case CLOSE:
            current_pos -= turnsToMake[x];
            break;
          case NOWHERE:
          default:
            current_pos = current_pos;
            break;
        }
        break;
      case QuarterTurns: 
        if(spd == FULL)
        {
          time_to_wait += turnsToMake[x] * msPerQtrTurn;
        }
        else
        {
          time_to_wait += turnsToMake[x] * (msPerQtrTurn / 2);
        }  
        switch (dir) {
          case OPEN:
            current_pos += (turnsToMake[x] * .25);
            break;
          case CLOSE:
            current_pos -= (turnsToMake[x] * .25);
            break;
          case NOWHERE:
          default:
            current_pos = current_pos;
            break;
        }
        break;
      default:
        Serial.println("ERROR: Impossible behavior detected");
        if(Debug)
        {
          Serial.println(__func__);
          Serial.print("The loop index is ");
          Serial.println(x);
        }
        break;
    }
  }

  // TODO: Check if turn would be illegal meaning we go past the fully open or fully closed point


  // Get the servo moving
  curtainRodServo.write(move);

  // Wait the required milliseconds to make a full turn
  delay(time_to_wait);
  
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach servoPin to our Curtain Rod Servo 
  curtainRodServo.attach(servoPin);

  // Tell servo not to move
  servo_move move = NoMovement;
  curtainRodServo.write(move);

  // Set current position
  // start from closed point
  current_pos = close_pt;

  /* Set the previous distance inches for the start
  ** Just in case someone is standing within the 13ft 
  ** boundary as the arduino is booting up
  */
  prev_d_inches = GetDistance();
}

void loop() {
  float d_inches; // Variable to store distance in inches

  /* Use ultrasonic device to get distance from subject
  ** NOTE: Max of 13ft measuring distance
  */
  d_inches = GetDistance();
  
  /* Will perform an action based on state of blinds and subject 
  ** This also does the necessary waiting to hold the loop
  */
  Perform(d_inches);

  // Save distance for next loop
  prev_d_inches = d_inches;
}

