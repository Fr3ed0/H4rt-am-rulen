//Copyright Frederic "Fr3ed0" Wolf
//2015-2016

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>

//Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4 the motors go.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

//we will use the #define stuff. ask me about it! DEACTIVATE BEFORE THE RACE STARTS!
//#define DEBUG // if active we are in the DEBUGGING mode, firing the Serial.println guns
//#define ZEIT
//#define SENSORADJUST // use this one FIRST:  *SensorAdjust to make all Sensors equal
//#define MIDSENSOR // use midTarget to make midError = 0 if Sensor is placed on grey
//#define LEFTSENSOR //
//#define RIGHTSENSOR //
#define MOTOR

//Sensor pins
int leftIRPin = A1;
int rightIRPin = A2;
int midIRPin = A0;

// Initialize variables
const float kP = 1.2;
const float kI = 0.003;
const float kD = 0.;
int leftSensorAdjust = -14; //use those to calibrate sensors
int rightSensorAdjust = 0;
int midSensorAdjust = 0;

//TargetSpeed is the power the motors will get if we want the robot to drive straight.
const byte leftTargetSpeed = 60; // we are using a byte here, because the motorshield
const byte rightTargetSpeed = 60; // goes from 0-255.

//the target values for the sensors. if those are unchanged the robot will go straight
//place sensors between black and weight and insert proper readings
int leftTarget = 80;
int rightTarget = 60;
int midTarget = 580; //the darker the sourroundings the higer tis value should be

//leftError is (the reading - the target)
int leftError;
int rightError;
int midError;

int turn;

long integral;

//derivative stuff
int derivative;
byte i;
int bPrevError = 0;
int aPrevError = 0;

//values for motorspeed
int leftSpeed;
int rightSpeed;

//lightSensorStuff
int trashL; //used to trash the first reading 
int trashR;
int trashMid;
//left Sensor
int leftSensor0; // we will meassure 5 times each sensore and only use the average value
int leftSensor1;
int leftSensor2;
int leftSensor3;
int leftSensor4;
int leftSensorAvg; //the average value of the raw data
int leftSensor; //the value which we will actually use avg-offset
//right Sensor
int rightSensor0;
int rightSensor1;
int rightSensor2;
int rightSensor3;
int rightSensor4;
int rightSensorAvg;
int rightSensor;
//This is the only actual PID-Sensor
int midSensor0;
int midSensor1;
int midSensor2;
int midSensor3;
int midSensor4;
int trashM;
int midSensorAvg;
int midSensor;

#ifdef SENSORADJUST
int ml;
int mr; 
#endif


//Interrupt motor adjust stuff

volatile  uint8_t lastB = 0;   
volatile  uint16_t counterA;
volatile  uint16_t counterB;
volatile int8_t altAB = 0;
volatile long encoderWert = 0;
 

#define LINKERMOTORPIN 2
#define RECHTERMOTORPIN 3

#define GREIFARMPINA 4
#define GREIFARMPINB 5

//tacho stuff
int altwertA;
int altwert;
int neuwert;
const byte deltaTacho=100; // chose your value according to your prefered max range 

//greifarm stuff
int8_t schrittTab[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; 



//delta t time loop
long jetzt = 0;
long altZeit, altZeitb;
int deltaT = 15; //we have to see if this is to high or to low

void setup() {

  //Interrupt stuff
PCICR |= (1 << PCIE2); //arduino will react on interrupts on port D
PCMSK2 |= (1 << LINKERMOTORPIN); // 2 defines the pin which listens for motor encoder (must be on portD)
PCMSK2 |= (1 << RECHTERMOTORPIN); // for the second motor ** 00001100
PCMSK2 |= (1 << GREIFARMPINA);
PCMSK2 |= (1 << GREIFARMPINB);
DDRD&=~PCMSK2; //if DDRD is 0 on the specific bit = input


  
  Serial.begin(9600);           // set up Serial baud to 115200 bps
  AFMS.begin();  // start motorshield with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
    //We are reading both values and trash them. the first readings are just that.
    trashL = analogRead(leftIRPin);
    trashR = analogRead(rightIRPin);
    trashM = analogRead(midIRPin);
    // Set motor direction
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      leftMotor->run(RELEASE);
//  leftMotor->run(BACKWARD);
    //  rightMotor->run(BACKWARD);
     integral = 0 ;
  
  //TODO make tests for all them hardware!
//    Serial.println("Adafruit Motorshield v2 - ready!");
//    Serial.println("All Systems GO!");

interrupts();
}

void loop() {
static  uint16_t altwertA;
uint16_t SpeedA;
  //a super time-get-right loop which is fueled by pure magic
  //pure magic means our prozessor has a sense of time which he will tell us with "millis"
  //in this loop we get our n-1 readings (the first one always gets trashed) 
  jetzt = millis();
  
  if((jetzt - altZeitb) > deltaTacho){
    altZeitb = jetzt;    
    SpeedA=counterA-altwertA;
    altwertA=counterA;
    
 #ifdef MOTOR
  Serial.println(SpeedA);
  
  #endif
  }  

  
  if((jetzt - altZeit) > deltaT){
   #ifdef ZEIT 
    Serial.println((jetzt - altZeit));
    #endif
    altZeit = jetzt;
      //TODO FIX OVERFLOW for jetzt
    //getting new values - since i dont know how to proper use an array it looks like that
   
    
    
    
  trashL = analogRead(leftIRPin);  
  leftSensor0 = analogRead(leftIRPin);
  leftSensor1 = analogRead(leftIRPin);
  leftSensor2 = analogRead(leftIRPin);
  leftSensor3 = analogRead(leftIRPin);
  leftSensor4 = analogRead(leftIRPin);
  
  trashR = analogRead(rightIRPin);
  rightSensor0 = analogRead(rightIRPin);
  rightSensor1 = analogRead(rightIRPin);
  rightSensor2 = analogRead(rightIRPin);
  rightSensor3 = analogRead(rightIRPin);
  rightSensor4 = analogRead(rightIRPin);
  
  trashMid = analogRead(midIRPin);
  midSensor0 = analogRead(midIRPin);
  midSensor1 = analogRead(midIRPin);
  midSensor2 = analogRead(midIRPin);
  midSensor3 = analogRead(midIRPin);
  midSensor4 = analogRead(midIRPin);

    
    //calculate the avg out of the raw sensor data ... again an array would be superbé
    leftSensorAvg = ((leftSensor0 + leftSensor1 + leftSensor2 +
                      leftSensor3 + leftSensor4)/5);
    
    rightSensorAvg = ((rightSensor0 + rightSensor1 + rightSensor2 +
                       rightSensor3 + rightSensor4)/5);

    midSensorAvg = ((midSensor0 + midSensor1 + midSensor2 + midSensor3 + midSensor4)/5);
                       
    //finally we get our good sensor data
    leftSensor = leftSensorAvg - leftSensorAdjust;
    rightSensor = rightSensorAvg - rightSensorAdjust;
    midSensor = midSensorAvg - midSensorAdjust;

    
    leftError = leftTarget - leftSensor;
    rightError = rightTarget - rightSensor;
    midError = midTarget - midSensor;

    //IntegralStuff
    //FIXME: maybe +-10 isnt big enough
    integral = integral + midError;
    if(midError<=10 && midError>=-10){
      integral = 0;
    }
      
    //derivate stuff
    //adding the nessesary things for the derivative -Whoppa FlipFlop Style
    i++;
    if((i % 2) == 0){
      aPrevError = midError;
      derivative = midError - bPrevError;
      } else {
      bPrevError = midError ;
      derivative = midError - aPrevError;
        }
      
#ifdef SENSORADJUST
int ml = (midSensorAvg - leftSensorAvg);
int mr = (midSensorAvg - rightSensorAvg); 
#endif       
    }

  

    //Heres how we adjust the motorspeeds!
    //from now on on we totally ignore our secondary sensors
    //
    turn = midError * kP + kI * integral + kD * derivative;

    leftSpeed = leftTargetSpeed + turn;
    rightSpeed = rightTargetSpeed - turn;
    
    // HACKED: If speed is > 255 we might want to use the overflow instead of trash it.
    // e.G turn == 300 -> 300-255 = 45 => Motor 1: 255 Motor2: leftSpeed - 45 (instead of just leftSpeed)
    
    if (leftSpeed >= 255){
      leftSpeed = 255;
    }
    if (rightSpeed >= 255){
      rightSpeed = 255;
    }
    if (leftSpeed <= 0) {
      leftSpeed = 0;
    }
    if (rightSpeed <= 0){
      rightSpeed = 0;
    }
    
    //Adjusting the Motorspeeds to get the turn!(via i2c)
  //  leftMotor->setSpeed(leftSpeed);
  //  rightMotor->setSpeed(rightSpeed);

    leftMotor->setSpeed(255);
    rightMotor->setSpeed(255);

    #ifdef DEBUG
    Serial.println(String(midError) + "\t " + String(integral) + "\t" + int(derivative) + "\t" + String(turn) + "\t" + String(leftSpeed) + "\t" + String(rightSpeed));
    #endif

    #ifdef SENSORADJUST
    Serial.println(String("Mavg: ") + int(midSensorAvg) + String("Lavg: ") + int(leftSensorAvg) + String("Ravg: ") + int(rightSensorAvg) +
    String("ML = 0?: ") + int(ml) + String("MR = 0?: ") + int(mr));
    #endif

    #ifdef MIDSENSOR
    Serial.println(String("Mavg: ") + int(midSensorAvg)  + String(" on grey? 0=>?: ") + int(midError));
    #endif

    
    #ifdef LEFTSENSOR
    Serial.println(String("Lavg: ") + int(leftSensorAvg) + String(" on grey? 0=>?: ") + int(leftError));
    #endif

    
    #ifdef RIGHTSENSOR
    Serial.println(String("Ravg: ") + int(rightSensorAvg) + String(" on grey? 0=>?: ") + int(rightError));
    #endif
    
  }





ISR(PCINT2_vect){ //speciifc routine

  uint8_t newB = PIND;                  // aktuellen Port speichern
  uint8_t chgB = newB ^ lastB;          // geänderte Bits zum letzten Port
  lastB = newB;                         // neuer Port wird zum alten für die nächste Runde

  if (chgB) {                             // wenn mindestens 1 Bit geändert   
    
    if (chgB & (1 << LINKERMOTORPIN)) {
     counterA++;
    }
     if (chgB & (1 << RECHTERMOTORPIN)) {
     counterB++;
    }
    if (chgB & ((1 <<GREIFARMPINA) | (1 << GREIFARMPINB)) ){
        altAB <<= 2; 
        altAB &= B00001100;
   //   altAB |= ((0b00000100 & newB) << 1) | (0b00001000 & newB); 
   //   altAB |= (((1 << GREIFARMPINA) & newB) << 1) | ((1 << GREIFARMPINB) & newB);
        altAB |= (digitalRead(GREIFARMPINA) << 1) | digitalRead(GREIFARMPINB); 
        encoderWert += schrittTab[altAB];
    }

  }
}
  
