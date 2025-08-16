// Thanks to Rachel De Barros and this video for inspiration: How to Control a Servo with an Ultrasonic Sensor and Arduino + Code ( https://www.youtube.com/watch?v=ybhMIy9LWFg )
// Thank you Chit for keeping me motivated ( https://www.youtube.com/watch?v=miE07JBZO6Q&t=32s )
// Thanks to the AI featured in this video for making this possible ( https://www.youtube.com/watch?v=xvFZjo5PgG0 )
#include <Servo.h>

// Performance Modes
enum pfmc_mode {Convincing, Complicated};

// Environment Vars
const bool            Debug = true;
const bool     Measure_Mode = false;
pfmc_mode  Performance_Mode = Convincing;
// Number of seconds to convincingly pause
const unsigned long convincing_pause = 30000;
// The boundary of importance when detecting the subject in inches
const int boundary_distance = 120;

// Pin Connections
const int servoPin = 7; 
const int  trigPin = 9; 
const int  echoPin = 10; 

// Turn indexes for turnsToMake Array
enum idx_turns {WholeTurns, QuarterTurns, END};

// Direction subject is going
enum move_cmd {MoveCompletely, MoveAWholeLot, MoveALot, Move, MoveABit, MoveALittle, MoveALittleBit, MoveATeenyTinyBit};

// Direction subject is going
enum subj_dir {STILL, AWAY, TOWARDS};

/* Direction to spin
** NOTE: Blinds need to be turned to the right/clockwise to close
**       Turn left/counter-clockwise to open
*/
enum servo_dir {OPEN, CLOSE, NOWHERE};

// Speed to spin
enum servo_speed {FULL, HALF};

/* Speeds for moving the servo - Read Servo.h for indepth info
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
const double open_pt = 4; 
// Fully closed position
const double close_pt = 0.0; 
// Number of milliseconds to go from fully open to fully closed or vice versa
const unsigned long total_ms_to_switch_states =  (unsigned long) (open_pt * msPerTurn);
// The current position of the blinds
double current_pos = -1;
// THe previous position of the blinds
double prev_pos = -1;

/*
** Globals
*/ 
// How many things have we done?
int actions_performed = 0;
// Store previously calculated distance
double prev_d_inches; 
// Store current runtime in milliseconds
// NOTE: millis() will overflow back to 0 after 50 days
unsigned long current_runtime = 0;
// Store previously queried runtime in milliseconds
unsigned long prev_runtime = 0;
// Have we detected something in our boundary? 
bool subj_is_detected = false;
// The Servo!
Servo curtainRodServo; 

/* Use ultrasonic device and math to get distance
** NOTE: Max of 13ft measuring distance
*/
double GetDistance() {
  // Calculating distance to subject
  float duration; // Variable to store pulse duration
  float distanceCM; // Variable to store distance in CM
  double distanceIN; // Variable to store distance in IN

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
  distanceIN =  static_cast<double>(distanceCM / cm2in);

  /* WARNING WARNING WARNING
  ?????????????????????????????
  How is distance reported when nothing is being detected?
  Does that flip to zero or is that registered as 13ft

  So far behavior we have observed is that it reports a length of something
  within the 13ft limit. So we may not see this the above behavior
  during exhibition

  ALSO, what's reported when you're like RIGHT ON the sensor
  ?????????????????????????????
  ** WARNING WARNING WARNING
  */

  // If measurement is outside of 13ft, use previous measurement
  if(distanceIN > 156)
  {
    distanceIN = prev_d_inches;
  }

  // Subject is detected 
  if(distanceIN >= 0 && distanceIN <= 10)
  {
    subj_is_detected = true;

    if(Debug)
    {
      Serial.println("Subject is detected within our range");
    }
  }

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
void ResetTurns(unsigned long *turnsToMake[]) {
  // Reset Turns
  for (int x = 0; x < 2 ; x++)
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

// Calculate how much time it will take to do turns
void Turn(unsigned long *turnsToMake, servo_dir dir, servo_speed spd) {
  unsigned long time_to_wait = 0;
  servo_move move = NoMovement;

  if(Debug){
        Serial.print(__func__);
        Serial.print(" - WholeTurns ");
        Serial.print(turnsToMake[WholeTurns]);
        Serial.print(" QuarterTurns ");
        Serial.println(turnsToMake[QuarterTurns]);
      }

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
  for (int x = 0; x < 2; x++)
  {
    switch (x)
    {
      case WholeTurns:
        if(spd == FULL)
        {
          time_to_wait += static_cast<int>(turnsToMake[x]) * msPerTurn;
        }
        else
        {
          time_to_wait += static_cast<int>(turnsToMake[x]) * (msPerTurn / 2);
        }
        switch (dir) {
          case OPEN:
            current_pos += static_cast<double>(turnsToMake[x]);
            break;
          case CLOSE:
            current_pos -= static_cast<double>(turnsToMake[x]);
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
          time_to_wait += static_cast<int>(turnsToMake[x]) * msPerQtrTurn;
        }
        else
        {
          time_to_wait += static_cast<int>(turnsToMake[x]) * (msPerQtrTurn / 2);
        }  
        switch (dir) {
          case OPEN:
            current_pos += static_cast<double>( (static_cast<double>(turnsToMake[x]) * .25) );
            break;
          case CLOSE:
            current_pos -= static_cast<double>( (static_cast<double>(turnsToMake[x]) * .25) );
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
    double pos_diff = 0.0;

    if (current_pos > open_pt)
    {
      if(Debug)
      {
        Serial.println("We are about to overturn past the open point. Make the correction.");
      }
      pos_diff = open_pt - prev_pos;

      current_pos = open_pt;      
    }
    else if (current_pos < close_pt)
    {
      if(Debug)
      {
        Serial.println("We are about to overturn past the close point. Make the correction.");
      }
      pos_diff = prev_pos;

      current_pos = close_pt;
    }

    time_to_wait = msPerTurn * pos_diff;

    if(Debug)
    {
      Serial.print("The amount that needs to be moved in order to each open or close points: ");
      Serial.print(pos_diff);
      Serial.print("The time needed to make that movement: ");
      Serial.println(time_to_wait);
    }
  }

  // Only move and wait if needed
  if(dir != NOWHERE)
  {
    if(Debug)
    {
      Serial.print(__func__);
      Serial.print(" is turning ");
      Serial.print(move);
      Serial.print(" for ");
      Serial.print(time_to_wait);
    } 
    // Get the servo moving
    curtainRodServo.write(move);

    // Wait the required milliseconds to make the desired turns
    delay(time_to_wait);

    curtainRodServo.write(NoMovement);
  }
  // Don't turn or wait
  else
  {
    if(Debug)
    {
      Serial.print(__func__);
      Serial.println("is NOT turning");
    }
  }
}

/* Calls Turn() after doing some setup
** !! Change turn amounts associated with move commands here !!
*/
void DoItLady(move_cmd mv, servo_dir dir, servo_speed spd)
{
  unsigned long turnsToMake[2];
  unsigned long int qtr_trns = 0;
  double turns_needed = 0;
  double whole, decimal;

  // Determine how many turns to make based on command
  switch(mv)
  {
    case MoveCompletely:
      //ResetTurns(&turnsToMake);
      turnsToMake[WholeTurns] = 0;
      turnsToMake[QuarterTurns] = 0;

      if(dir == OPEN)
      {
        turns_needed = open_pt - current_pos;
      }
      else if (dir == CLOSE)
      {
        turns_needed = current_pos;
      }
      else
      {
        // Set dir to NOWHERE explicitly to account for a situation where it's set to something that isn't allowed
        dir = NOWHERE;
        if(Debug)
        {
          Serial.print(__func__);
          Serial.println(" is not opening or closing");
        }
      }

      /* Figure out whole turns and quarter turns to use in turnsToMake[]
      ** NOTE: modf from cmath takes the whole number and decimal and places them into variables
      */
      whole = trunc(turns_needed);
      decimal = turns_needed - whole;

      // Use the whole number
      turnsToMake[WholeTurns] = static_cast<unsigned int>(whole);

      /* Figure out the number of quarter turns needed by subtracting .25 from decimal
      ** NOTE: It shouldn't happen, but we'll avoid going into the negatives which 
      **       could lead to overturning
      */
      while(decimal > .24)
      {
        decimal -= 0.25;
        qtr_trns++;
      }
      turnsToMake[QuarterTurns] = qtr_trns;

      if(Debug){
        Serial.print(__func__);
        Serial.print(" - WholeTurns ");
        Serial.print(turnsToMake[WholeTurns]);
        Serial.print(" QuarterTurns ");
        Serial.println(turnsToMake[QuarterTurns]);
      }
      break;
    case MoveAWholeLot:
        turnsToMake[WholeTurns] = 4;
      turnsToMake[QuarterTurns] = 1;
      break;
    case MoveALot:
        turnsToMake[WholeTurns] = 3;
      turnsToMake[QuarterTurns] = 3;
      break;
    case Move:
        turnsToMake[WholeTurns] = 2;
      turnsToMake[QuarterTurns] = 2;
      break;
    case MoveABit:
        turnsToMake[WholeTurns] = 1;
      turnsToMake[QuarterTurns] = 3;
      break;
    case MoveALittle:
        turnsToMake[WholeTurns] = 1;
      turnsToMake[QuarterTurns] = 0;
      break;
    case MoveALittleBit:
        turnsToMake[WholeTurns] = 0;
      turnsToMake[QuarterTurns] = 2;
      break;
    case MoveATeenyTinyBit:
        turnsToMake[WholeTurns] = 0;
      turnsToMake[QuarterTurns] = 1;
      break;
    default:
      break;
  }

  if(Debug)
  {
    Serial.print("About to move ");
    Serial.print(mv);
    Serial.print(" to ");
    Serial.print(dir);
    Serial.print(" at ");
    Serial.println(spd);
  }

  // Do it lady!
  Turn(turnsToMake, dir, spd);
}

// The move theoretically ends up where it started, but with a lot of noise
void DoAlmostNothingMaddeninglyImperceptibly(){
  if(Debug)
  {
    Serial.print("Performing ");
    Serial.println(__func__);
  }
  DoItLady(MoveATeenyTinyBit, OPEN, HALF);
  DoItLady(MoveATeenyTinyBit, CLOSE, HALF);
  DoItLady(MoveATeenyTinyBit, OPEN, HALF);
  DoItLady(MoveATeenyTinyBit, CLOSE, HALF);
}

// Open so very slightly
void DoSomethingMaddeninglyImperceptible(){
  if(Debug)
  {
    Serial.print("Performing ");
    Serial.println(__func__);
  }
  DoItLady(MoveALittleBit, OPEN, HALF);
}

/* Close the blinds completely
** NOTE: All the math to prevent overturning is done behind the scenes
*/
void CloseCompletely(){
  if(Debug)
  {
    Serial.print("Performing ");
    Serial.println(__func__);
  }
  DoItLady(MoveCompletely, CLOSE, FULL);
}

/* Open the blinds completely
** NOTE: All the math to prevent overturning is done behind the scenes
*/
void OpenCompletely(){
  if(Debug)
  {
    Serial.print("Performing ");
    Serial.println(__func__);
  }
  DoItLady(MoveCompletely, OPEN, FULL);
}

// Fully closed to fully open twice
void Blink(){
  if(Debug)
  {
    Serial.print("Performing ");
    Serial.println(__func__);
  }
  CloseCompletely();
  OpenCompletely();
  CloseCompletely();
  OpenCompletely();
}

// Will (silently obviously) not cause a move or wait
void DoLiterallyNothing(){
  if(Debug)
  {
    Serial.print("Performing ");
    Serial.println(__func__);
  }
  DoItLady(MoveATeenyTinyBit, NOWHERE, HALF);
}

// Performs an action based on system state
void Perform(double distance) {
  // To tell how subject is moving
  subj_dir going; 

  /* Is subject moving closer or further away?
  ** Staying still has a wiggle room of 2 inches
  */
  double diff = prev_d_inches - distance;
  if(diff >= -4 && diff <= 4)
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

  /* Do something depending on where the subject is going
  ** NOTE: Distance is in inches
  */
  switch (going) {
    case STILL:
      /* If the subject is staying still
      ** do something
      */
      if(Debug)
      {
        Serial.println("The subject cannot be seen by a T-Rex");
      }
      
      // Convincingly do nothing if the subject is still
      if(Performance_Mode == Convincing)
      {
        DoLiterallyNothing();
      }

      // Complicatedly do something fun
      else if(Performance_Mode == Complicated)
      {
        if(actions_performed % 13 == 0)
        {
          DoSomethingMaddeninglyImperceptible();
        }
        else if(actions_performed % 11 == 0)
        {
          OpenCompletely();
        }
        else if(actions_performed % 9 == 0)
        {
          DoItLady(MoveABit, CLOSE, HALF);
        }
        else if(actions_performed % 7 == 0)
        {
          DoItLady(MoveALittleBit, CLOSE, FULL);
        }
        else if(actions_performed % 5 == 0 && actions_performed % 2 != 0)
        {
          Blink();
        }
        else if(actions_performed % 3 == 0 && actions_performed % 2 != 0)
        {
          DoAlmostNothingMaddeninglyImperceptibly();
        }
        // Do something random at first action and with evens that make it here
        else
        {
          // Generate a random number between 1 and 100
          int rand = random(1, 101);

          if(Debug)
          {
            Serial.print("Generated random number: ");
            Serial.println(rand);
          }

          // Do it randomly lady!
          switch(rand)
          {
            case 100:
              OpenCompletely();
              break;
            case 90 ... 99:
              DoItLady(Move, CLOSE, FULL);
              break;
            case 50:
              CloseCompletely();
              break;
            case 11 ... 20:
              DoItLady(MoveALot, CLOSE, HALF);
            case 2 ... 10:
              Blink();
              break;
            case 1:
              DoLiterallyNothing();
              break;
            default:
              DoItLady(MoveAWholeLot, CLOSE, HALF);
              break;
          }
        }
      }

      break;
    case AWAY:
      /* If the subject is 10 feet or less away
      ** and they're moving away from the painting
      */
      if(distance <= boundary_distance)
      {
        if(Debug)
        {
          Serial.println("The subject is 10 feet or less away and they're going away.");
        }

        // Convincingly do nothing if the subject is moving away
        if(Performance_Mode == Convincing)
        {
          DoLiterallyNothing();
        }

        // Complicatedly open the blinds some
        else if(Performance_Mode == Complicated)
        {
          DoItLady(Move, OPEN, FULL);
        }
      }
      // Subject is outside of 10ft boundary and they're going away
      else
      {
        if(Debug)
        {
          Serial.println("The subject is more than 10 feet away and they're going away.");
        }

        // Convincingly do nothing if the subject is moving away
        if(Performance_Mode == Convincing)
        {
          DoLiterallyNothing();
        }

        // Complicatedly open the blinds all the way
        else if(Performance_Mode == Complicated)
        {
          OpenCompletely();
        }
      }
      break;
    case TOWARDS:
      /* If the subject is 10 feet or less away
      ** and they're moving towards the painting
      */
      if(distance <= boundary_distance)
      {
        if(Debug)
        {
          Serial.println("The subject is 10 feet or less away and they're coming closer.");
        }

        // Convincingly compelely close, pause, and completely open the blinds
        if(Performance_Mode == Convincing)
        {
          CloseCompletely();
          delay(convincing_pause);
          OpenCompletely();
        }

        // Complicatedly close the blinds some
        else if(Performance_Mode == Complicated)
        {
          DoItLady(Move, CLOSE, HALF);
        }

      }
      // Subject is outside of 10ft boundary and is coming closer
      else
      {
        
        if(Debug)
        {
          Serial.println("The subject is more than 10 feet away and they're coming closer.");
        }

        // Convincingly do nothing if the subject is coming closer
        if(Performance_Mode == Convincing)
        {
          DoLiterallyNothing();
        }

        // Complicatedly open the blinds all the way
        else if(Performance_Mode == Complicated)
        {
          if(actions_performed % 3 == 0)
          {
            Blink();
          }
          else if(actions_performed % 2 == 0)
          {
            DoItLady(MoveATeenyTinyBit, CLOSE, FULL);
          }
          else
          {
            DoAlmostNothingMaddeninglyImperceptibly();
          }
        }
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

  // Increment our action counter
  actions_performed++;
} 

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if(Debug)
  {
    Serial.println("STARTING");
  }

  // Seed the random number generator with the current time
  //srand(time(0));

  // Attach servoPin to our Curtain Rod Servo 
  curtainRodServo.attach(servoPin);

  // Tell servo not to move
  servo_move move = NoMovement;
  curtainRodServo.write(move);

  // Set current position
  // start from open point
  current_pos = open_pt;

  // Set previous position
  prev_pos = close_pt;

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
  double d_inches; 

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

    // Pause for a second
    delay(1000);
  }

  // Save runtime for next loop
  prev_runtime = current_runtime;
}

