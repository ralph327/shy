// Thanks to Rachel De Barros and this video for inspiration: How to Control a Servo with an Ultrasonic Sensor and Arduino + Code ( https://www.youtube.com/watch?v=ybhMIy9LWFg )
#include <Servo.h>

// Bools
const bool Debug = true;
const bool Measure_Mode = false;
bool subj_is_detected = false;

// Pin Connections
const int servoPin = 7; 
const int  trigPin = 9; 
const int  echoPin = 10; 

// Turn indexes for turnsToMake Array
enum idx_turns {WholeTurns, QuarterTurns, END};

// Direction subject is going
enum subj_dir {STILL, AWAY, TOWARDS};

/* Direction to spin
** NOTE: Blinds need to be turned to the right/clockwise to close
**       Turn left/counter-clockwise to open
*/
enum servo_dir {OPEN, CLOSE, NOWHERE};

// Speed to spin
enum servo_speed {FULL, HALF};

/* Speeds for moving the servo
** A continuous servo moves at a certain speed in one direction
** 0 being full speed in one direction, 180 being full speed in the other direction, 90 being motionless, with gradients in between
** NOTE: Switch these depending on which direction the angle goes
*/
enum servo_move {FullSpeedOpen = 180, HalfSpeedOpen = 135, NoMovement = 90, HalfSpeedClose = 45, FullSpeedClose = 0};

/*
** Values for calculations
*/
// In Microseconds
const float speedOfSound = 0.034; 
// CMs in an Inch
const float cm2in = 2.54; 
// The number of milliseconds needed to complete a full turn (360 degrees) at full speed
const unsigned long msPerTurn = 626; 
// The number of milliseconds needed to complete a quarter turn (90 degrees) at full speed
const unsigned long msPerQtrTurn = msPerTurn/4;  
// 30 seconds -  1000 ms = 1 s
const long thirty_sec_interval = 30000;
// One Minute -  1000 ms = 1 s
const long one_minute_interval = 60000;
// Two Minutes -  1000 ms = 1 s
const long two_minute_interval = 120000;


/*
** Variables for position of servo
** NOTE: shy should start fully open
*/ 
// The number of turns needed to be fully open, calculate by spinning the curtain rod from fully closed to fully open
const float open_pt = 5.5; 
// Fully closed position
const float close_pt = 0.0; 
// Number of milliseconds to go from fully open to fully closed or vice versa
const unsigned long total_ms_to_switch_states =  (unsigned long) (open_pt * msPerTurn);
// The current position of the blinds
float current_pos = -1;
// THe previous position of the blinds
float prev_pos = -1;

/*
** Globals
*/ 
// Store previously calculated distance
float prev_d_inches; 
// Store current runtime in milliseconds
// NOTE: millis() will overflow back to 0 after 50 days
unsigned long current_runtime = 0;
// Store previously queried runtime in milliseconds
unsigned long prev_runtime = 0; 
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
  delayMicroseconds(10); // Must be high for 10 seconds to activate transmission
  digitalWrite(trigPin, LOW);

  // Grab pulse duration in microseconds
  // If set to HIGH, pulseIn() waits for the pin to go from LOW to HIGH
  // Timing stops when pin goes back to LOW
  duration = pulseIn(echoPin, HIGH);

  if (Debug)
  {
    // Print Distance
    Serial.print("Duration: ");
    Serial.println(duration);
  }

  // Distance in centimeters. Calculate how long it took sound to travel
  // Divide by 2 so that we only get the distance coming back from source
  distanceCM = (duration * speedOfSound) / 2;

  // Convert to inches
  distanceIN = distanceCM / cm2in;

  /* WARNING WARNING WARNING
  ?????????????????????????????
  How is distance reported when nothing is being detected?
  Does that flip to zero or is that registered as 13ft

  So far behavior we have observed is that it reports a length of something
  within the 13ft limit. So we may not see this the above behavior
  during exhibition
  ?????????????????????????????
  ** WARNING WARNING WARNING
  */

  /* TODO: set subject is detected 
  if(distanceIN > 0 && distanceIN < 13)
  {
    subj_is_detected = true;
  }
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
void ResetTurns(unsigned long int *turnsToMake[]) {
  // Reset Turns
  for (int x = 0; x < static_cast<int>(sizeof(*turnsToMake)) / sizeof(turnsToMake[0]) ; x++)
  {
    turnsToMake[x] = 0;

    if(Debug)
    {
      Serial.print("Setting index ");
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
  if(diff >= -1 && diff <= 1)
  {
    going = STILL;
    if(Debug)
    {
      Serial.println("Subject is staying still");
    }
  }
  else if(diff < -1)
  {
    going = AWAY;
    if(Debug)
    {
      Serial.println("Subject is going away");
    }
  }
  else if (diff > 1)
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
  
  // Calculate the time between the last action performed and this one
  unsigned long interval = current_runtime - prev_runtime; 

  if(Debug)
  {
    Serial.println("Interval between last action and the start of this one is: ");
    Serial.println(interval);
  }

  // Do something depending on where the subject is going
  switch (going) {
    case STILL:
      if (interval > thirty_sec_interval)
      {

      }
      break;
    case AWAY:
      break;
    case TOWARDS:
      if(distance <= 120)
      {
        // Start closing, keep doing distance checks until subject moves away to 10 ft and then start reopening
      }
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
void Turn(unsigned long int *turnsToMake[], servo_dir dir, servo_speed spd) {
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
          Serial.println(dir);
        }
      break;
  }

  /* Calculate how much time it will take to perform this series of turns
  ** and calculate what our current position will be at once this is performed
  */
  for (unsigned long int x = 0; x < static_cast<int>(sizeof(*turnsToMake)) / sizeof(turnsToMake[0]) ; x++)
  {
    switch (x)
    {
      case WholeTurns:
        if(spd == FULL)
        {
          time_to_wait += static_cast<int>(*turnsToMake[x]) * msPerTurn;
        }
        else
        {
          time_to_wait += static_cast<int>(*turnsToMake[x]) * (msPerTurn / 2);
        }
        switch (dir) {
          case OPEN:
            current_pos += static_cast<float>(*turnsToMake[x]);
            break;
          case CLOSE:
            current_pos -= static_cast<float>(*turnsToMake[x]);
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
          time_to_wait += static_cast<int>(*turnsToMake[x]) * msPerQtrTurn;
        }
        else
        {
          time_to_wait += static_cast<int>(*turnsToMake[x]) * (msPerQtrTurn / 2);
        }  
        switch (dir) {
          case OPEN:
            current_pos += static_cast<float>( (static_cast<double>(*turnsToMake[x]) * .25) );
            break;
          case CLOSE:
            current_pos -= static_cast<float>( (static_cast<double>(*turnsToMake[x]) * .25) );
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

  // Check if turn would be illegal meaning we go past the fully open or fully closed point
  if (current_pos > open_pt || current_pos < close_pt)
  {
    float pos_diff = 0.0;

    if (current_pos > open_pt)
    {
      pos_diff = open_pt - prev_pos;

      current_pos = open_pt;
      
      if(Debug)
      {
        Serial.println("We are about to overturn past the open point. Make the correction.")
      }
    }
    else if (current_pos < close_pt)
    {
      pos_diff = prev_pos;

      current_pos = close_pt;

      if(Debug)
      {
        Serial.println("We are about to overturn past the close point. Make the correction.")
      }
    }

    time_to_wait = msPerTurn * pos_diff;

    if(Debug)
    {
      Serial.print("The amount that needs to be moved in order to each open or close points: ");
      Serial.print(pos_diff);
      Serial.print("The time needed to make that movement: "));
      Serial.println(time_to_wait);
    }
  }

  // Get the servo moving
  curtainRodServo.write(move);

  // Wait the required milliseconds to make the desired turns
  delay(time_to_wait);
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if(Debug)
  {
    Serial.println("STARTING");
  }

  // Attach servoPin to our Curtain Rod Servo 
  curtainRodServo.attach(servoPin);

  // Tell servo not to move
  servo_move move = NoMovement;
  curtainRodServo.write(move);

  // Set current position
  // start from open point
  current_pos = open_pt;

  // Set previous position
  prev_pos = open_pt;

  /* Set the previous distance inches for the start
  ** Just in case someone is standing within the 13ft 
  ** boundary as the arduino is booting up
  */
  prev_d_inches = GetDistance();

  // Set runtime trackers
  current_runtime = millis();
  prev_runtime = current_runtime;
}

void loop() {
  // Variable to store distance in inches
  float d_inches; 

  // Update current runtime
  current_runtime = millis();

  if(Debug)
  {
    Serial.println("LOOPING");
  }

  /* Measure mode simply runs the servo back and forth 
  ** so that we can manually change the delay time to
  ** get an accurate time for a 360 degree rotation
  **
  */
  if(Measure_Mode == true) 
  {
    // Move the servo in the open direction
    servo_move move = FullSpeedOpen;
    curtainRodServo.write(move);
    delay(626); // Wait 626 milliseconds

    move = NoMovement;
    curtainRodServo.write(move);
    delay(3000); // Wait 3 seconds

    // Move the servo in the close direction
    move = FullSpeedClose;
    curtainRodServo.write(move);
    delay(626); // Wait 626 milliseconds

    move = NoMovement;
    curtainRodServo.write(move);
    delay(3000); // Wait 3 seconds
  }

  /* Normal Mode
  ** Checks Distance and performs an action
  */
  else
  {
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

    // Set previous position
    prev_pos = current_pos;
  }

  // Save runtime for next loop
  prev_runtime = current_runtime;
}

