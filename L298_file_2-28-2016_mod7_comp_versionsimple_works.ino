//#include <TimerOne.h>

// Set variables for encoder (from sketch feb14a)
enum PinAssignments {
  encoderPinA = 2,   // rigth
  encoderPinB = 3   // left
  //clearButton = 4    // another two pins
};

volatile long encoderPos = 0;  // a counter for the dial
volatile long pencoderPos = 0;  // a counter for the dial
//long lastReportedPos = 1;   // change management
static boolean rotating=false;      // debounce management
static boolean error=false;

// interrupt service routine vars
boolean A_set = false;              
boolean B_set = false;

unsigned int loopcount=0;
unsigned int motorspeed=125;
unsigned int reversespeed=100;
unsigned int setfwdspeed=100;
unsigned int setrevspeed=100;
unsigned int debouncedelay=1;
unsigned int safetytime =3000;
unsigned int safetyloop = 500;
unsigned int motordirection=1; //1 = forward, 0= reverse

float time=0.000000;
float ptime=0.000000;
volatile float rotation = 0.00;
float wheelcircum=2*3.141592*4/2; //circumference of wheel in inches
float targetdistance=1*39.37; //target distance in inches, for 39.37 inches per meter
float dtolerance=1.00;
volatile float dist = 0.000000;
volatile float pdist=0.000000;
volatile float acceleration =0.000000;
volatile float velocity =0.000000;
volatile float pvelocity =0.000000;

void setup() 
{
  Serial.begin(9600);

  //encoder Setup
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  //pinMode(clearButton, INPUT);
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  //digitalWrite(clearButton, HIGH);

  // encoder pin on interrupt 1 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 2 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  //Motor Setup
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(13,OUTPUT);

}
void loop()
{
   //Serial.print("Loop:  Entered loop routine ");  
   ptime=time;
   time = millis();
   time = time;
   loopcount += 1;
   
   if(time=ptime){
    acceleration=0.001;
    velocity=0.001;
   }
   else {
   pvelocity=velocity;
   velocity=((dist-pdist)/(time-ptime));
   acceleration = ((velocity-pvelocity)/(time-ptime));
    }
   
   if((acceleration>0) and (motordirection=0)) {
    error=true;
   }
  
   Serial.print("Encoderpos is ");
   Serial.println(encoderPos);
   Serial.print("Rotation is ");
   Serial.println(rotation);
   Serial.print("Distance is ");
   Serial.println(dist);
   Serial.print("Loopcount is ");
   Serial.println(loopcount);
   Serial.print("Time is ");
   Serial.println(time);
   Serial.print("Motordirection is ");
   Serial.println(motordirection);
   Serial.print("Motorspeed is ");
   Serial.println(motorspeed);
   Serial.print("Reversespeed is ");
   Serial.println(reversespeed);
   Serial.print("Acceleration is ");
   Serial.println(acceleration);
   Serial.println("");
 
   
    if (dist >= targetdistance*0.75)
    {
      motorspeed = 90;
    }
   if ((dist > targetdistance-dtolerance)and(dist<targetdistance+dtolerance))
      {
      reversespeed=0;
      motorspeed=0;
      }
   else {
      reversespeed=setrevspeed;
      motorspeed=setfwdspeed;
      }
   if (time >= safetytime )
      {
      reversespeed=0;
      motorspeed=0;
      }
   if (loopcount>safetyloop)
      {
      reversespeed=0;
      motorspeed=0;
      }
   motor2();
   pencoderPos=encoderPos;
}


void motor2()
{
  rotating = true;  // reset the debouncer
  
   //Motor settings for cw is LOW/HIGH
   //Motor settings for ccw is HIGH/LOW
   //Motor setting for brake is LOW/LOW
     
  if (dist<targetdistance-dtolerance){
    Serial.println("Motor2: Forward.");
    motordirection=1;  
     
     digitalWrite(8, LOW);
     digitalWrite(7, HIGH);
     analogWrite(13, motorspeed);
    
     //Serial.print("Motor2: Rotation is ");
     //Serial.println(rotation);
  }
 else if (dist>targetdistance+dtolerance) {
     Serial.println("Motor2: In Reverse."); 
     motordirection=0; 
     delay(500);
     digitalWrite(8, HIGH);
     digitalWrite(7, LOW);
     analogWrite(13, reversespeed-60);
     delay(100);
     analogWrite(13, reversespeed-40);
     delay(100);
     analogWrite(13, reversespeed);
     delay (1000);
     analogWrite(13, 0);
     delay(300);
     
     
     //Serial.print("Motor2: Rotation is ");
     //Serial.println(rotation);
  }
 else {
  Serial.println("Motor2: Stop.");  
  analogWrite(13, 0);
 }  
}

void doEncoderA(){
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change? 
  if( digitalRead(encoderPinA) != A_set ) {  // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set ) {
      encoderPos += 1;
      rotation=(encoderPos/96.00)/2;
      pdist=dist;
      dist=rotation*wheelcircum;
      ptime=time;
      time = millis();
      //time = time/1000.000000;
    }
  }
    rotating = false;  // no more debouncing until loop() hits again
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) delay (1);
  if( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) 
      encoderPos -= 1;
      rotation=(encoderPos/96.00)/2;
      pdist=dist;
      dist=(rotation*wheelcircum);
      ptime=time;
      time = millis();
      //time = time/1000.000000;
  }
    rotating = false;
}

   

