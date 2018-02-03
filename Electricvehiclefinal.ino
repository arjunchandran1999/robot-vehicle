/* 
 * Version 1.0
 */

/*WOOHOO LETS INITALIZE SOME VARIABLES FOR THE PROGRAM TO USE*/

//Initializations for the motor/H-Bridge to send PWM to spin the motor
int RPWM = 5; //for spinning right
int LPWM = 6; //for spinning left

//DO NOT TOUCH, FOR ENABLING SETUP
int R_EN = 8;
int L_EN = 7;

//STUFF FOR ROTARY ENCODERS IF YOU NEED TO CHANGE THE ROTARY ENCODERS CHANGE THESE AS WELL
volatile long encoderPos = 0; //counter for the dial
static boolean rotating = false;
unsigned int debounceDelay = 1;
boolean A_set = false;
boolean B_set = false;


//Inputs for rotary encoders
int encoderPinA = 2; //For the right wire
int encoderPinB = 3; //For the left left wire

//Values for distances, for different distances LOOK HERE AND CHANGE HERE
float metersNeeded = 9.8; //if you need to change the target distance CHANGE ME!!!
float targetDistMeters = -0.3084516968+(0.9747757676)*metersNeeded; //Target Distance in meters
float accelerateDistMeters = targetDistMeters/2.4; //How far it accelerates, accelerates 1/4 the distance full throttle
float distanceTolerance = 0.1; //Tolerance for distance from desired point
int teethPerGear = 20; //20 teeth on the gear


//Variables for car dimensions
float wheelCircum = 2.00 * 3.14159265 * 2.00; //The wheels circumfrence, in inches

//Conversion to inches for distances
float targetDist = targetDistMeters * 39.3701;
float accelerateDist = accelerateDistMeters * 39.3701;

//Other important variables
volatile float rotation = 0.00; //Number of rotations for the rotary encoders and to keep track of number of rotations for the print statement
volatile float distToTarget = targetDist; //Initialized as the original target distance
volatile float distanceTraveled = 0.000000;
boolean ranLoop = false;
volatile float currVelocity = 0.00; //Current velocity of the car, changes in the checkSpeed() function.
int currentAccelDirection = 0; //Current direction that the car/motors are accelerating, 1 is right, 0 is nothing, and -1 is left. This is currently not used but we may use it later.
int fullThrottleForwardSpeed = 255; //Speed for full throttle
int seekSpeed = 200; //Speed for seeking
int stopSpeed = 255; //Speed for stopping



void setup() {
  
  Serial.begin(250000);
  
  //Sets up all outputs for the motor/H-Bridge
  for (int i = 5; i < 7; i++)
  {
    pinMode(i, OUTPUT); //Set up all motor outputs as outputs
    digitalWrite(i,LOW); //Set up all motor as default low
  }
  
  for (int i = 7; i < 9; i++)
  {
    pinMode(i, OUTPUT); //Set up all enablers ports as outputs
    digitalWrite(i, HIGH); //Set up all enablers as HIGH
  }
  

  //Setting up pins for rotary encoders
  pinMode(encoderPinA, INPUT); //Setting up for right wire 
  pinMode(encoderPinB, INPUT); //Setting up for left wire
  digitalWrite(encoderPinA, HIGH); //Turn on pullup resistors for right wire
  digitalWrite(encoderPinB, HIGH); //Turn on pullup resistors for left wire

  //encoder pin on interrupt 1 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  //encoder pin on interrupt 2 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
  
  delay(2000);

}

void loop() 
{
  /*while (distance < (goal - error) || distance > (goal + error))
  {
    seek();
  }*/

  //If the program just started/first loop, then full throttle forward.
  if (!ranLoop)
  {
    fullThrottleForward();
  }

  //Stop the car after going full throttle forward
  stopCar();

  //start to seek for the target
  if(abs(distToTarget) > distanceTolerance)
  seekTarget();

  ranLoop = true;

  //Crash the program *this is temporary and is just to make sure it doesn't run again lol*
  while(true) {};
  

}

void fullThrottleForward()
{
  //Keep on going with the full throttle forward until the car is at the max distance for acceleration.
  while (distanceTraveled < accelerateDist)
  {
    motorPowerChange(RPWM, fullThrottleForwardSpeed); //call method to make it go full speed ahead
  };
}

void motorPowerChange(int direction, int power) {
  //Set the direction that the car is accelerating in
  if (direction == RPWM)
    currentAccelDirection = 1;
  else if (direction == LPWM)
    currentAccelDirection = -1;

  //Send the motor in the desired direction with the desired power 
  analogWrite(direction, power);
}


void checkSpeed()
{
  float initialTime = 0.00; //initial time that the car has already traveled 
  float initialDistance = 0.00; //Initial distance that the car has already traveled when checking speed
  float finalTime = 0.00; //Time after waiting 10 miliseconds
  float finalDistance = 0.00; //Distance after waiting 10 miliseconds
  float deltaTime = 0.00; //Change in time
  float deltaDistance = 0.00; //Change in distance

  initialTime = millis(); //Set the value
  initialDistance = distanceTraveled; //Set the value

  delay(30); //Wait 15 miliseconds before rechecking values to calculate change in position over change in time
    
  finalTime = millis(); //Set the value
  finalDistance = distanceTraveled; //Set the value

    
  deltaTime = (finalTime-initialTime)/1000; //put the change in time in seconds 
  deltaDistance = finalDistance-initialDistance; //Calculate the change in distance

  currVelocity = deltaDistance/deltaTime; //Calculate the current velocty
    
}

void stopCar()
{
  //Shut off the motors and check the current speed
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  checkSpeed();

  //If it's going right run motors left, a little bit of tolerance here that can be tweaked
  while(currVelocity > 0.01)
  {
    motorPowerChange(LPWM, stopSpeed);
    checkSpeed();
  }
  //If it's going left run it right, a little bit of tolerance here that can be tweaked
  while(currVelocity < -.01)
  {
    motorPowerChange(RPWM, stopSpeed);
    checkSpeed();
  }

  //Shut off the motors and check the speed one more time. Also set the current acceleration direction to zero (It is not speeding up because of a motor anymore).
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  currentAccelDirection = 0;
  checkSpeed();

  //If the current speed is not zero try to stop the car again. This is pretty much just a failsafe and should not be needed.
  if (abs(currVelocity) > 0.01)
    stopCar();
}

//This can be tweaked to change motor values and stuff, this is just the basic idea
void seekTarget()
{
  //While the distance to the target is beyond the distance tolerance seek the target
  while (abs(distToTarget) > distanceTolerance)
  {
    /*
    //While the distance traveled is too far spin the motor backwards
    while(distanceTraveled > targetDist + distanceTolerance)
    {
      motorPowerChange(LPWM, 255);
    }

    //stop the car after going in this direction
    stopCar();

    //Give rotary encoders time to check out the distances
    delay(300);

    */
    //While the distance traveled is too short spin the motor forwards
    while(distanceTraveled < targetDist - distanceTolerance)
    {
      motorPowerChange(RPWM, seekSpeed);
      delay(70);
      stopCar();
      delay(600);
    }
    
    //Stop the car after going in this direction
    stopCar();

    //Give rotary encoders time to check out the distances
    delay(100);
  }
  //Stop the car, this is just a failsafe and can probably be removed
  stopCar();
}


/*WARNING! PROCEED WITH CAUTION WHEN CHANGING ANYTHING BELOW
!
!
!
*/
// Interrupt on A changing state
void doEncoderA(){
  // debounce
  if ( rotating ) delay (5);  // wait a little until the bouncing is done

  // Test transition, did things really change? 
  if( digitalRead(encoderPinA) != A_set ) {  // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set ) 
      encoderPos += 1;

    rotating = false;  // no more debouncing until loop() hits again
  }
  Serial.println(encoderPos);
  rotation=(encoderPos/teethPerGear)/2.00;
  distanceTraveled=rotation*wheelCircum*1.00;
  distToTarget=targetDist-distanceTraveled;
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) delay (5);
  if( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) 
      encoderPos -= 1;

    rotating = false;
  }
  Serial.println(encoderPos);
  rotation=(encoderPos/teethPerGear)/2.00;
  distanceTraveled=rotation*wheelCircum*1.00;
  distToTarget = targetDist-distanceTraveled;
}
