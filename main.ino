#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>

//Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4 the motors go.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

//we will use the #define stuff. ask me about it! DEACTIVATE BEFORE THE RACE STARTS!
#define DEBUG // if active we are in the DEBUGGING mode, firing all the println guns
//#define DEBUGSENSOR

//Sensor pins
int leftIRPin = A0;
int rightIRPin = A1;

// Initialize variables
const float kP = 0.5;
const float kI = 0;
const float kD = 0;
int leftSensorAdjust = -14; //use those to calibrate sensors
int rightSensorAdjust = 0;

//TargetSpeed is the power the motors will get if we want the robot to drive straight.
const byte leftTargetSpeed = 80; // we are using a byte here, because the motorshield
const byte rightTargetSpeed = 80; // goes from 0-255.

//the target values for the sensors. if those are unchanged the robot will go straight
//place sensors between black and weight and insert proper readings
int leftTarget = 560;
int rightTarget = 560;

//leftError is the reading - the target
int leftError;
int rightError;

int turn;

//values for motorspeed
byte leftSpeed;
byte rightSpeed;

//lightSensorStuff
int trashL; //used to trash the first reading 
int trashR;
int leftSensor0; // we will meassure 5 times each sensore and only use the average value
int rightSensor0;
int leftSensor1;
int rightSensor1;
int leftSensor2;
int rightSensor2;
int leftSensor3;
int rightSensor3;
int leftSensor4;
int rightSensor4;
int leftSensorAvg; //the average value of the raw data
int rightSensorAvg;
int leftSensor; //the value which we will actually use avg-offset
int rightSensor;


//delta t time loop
long jetzt = 0;
int deltaT = 30; //we have to see if this is to high or to low

void setup() {
  Serial.begin(9600);           // set up Serial baud to 115200 bps
  AFMS.begin();  // start motorshield with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
    //We are reading both values and trash them. the first readings are just that.
    trashL = analogRead(leftIRPin);
    trashR = analogRead(rightIRPin);
  
    // Set motor direction
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    //  leftMotor->run(BACKWARD);
    //  rightMotor->run(BACKWARD);
  
  //TODO make tests for all them hardware!
    Serial.println("Adafruit Motorshield v2 - ready!");
    Serial.println("All Systems GO!");
}

void loop() {
  //a super time-get-right loop which is fueled by pure magic
  //pure magic means our prozessor has a sense of time which he will tell us with "millis"
  //in this loop we get our n-1 readings (the first one always gets trashed) 
  if((millis() - jetzt) > deltaT){
    jetzt = millis(); 
    //getting new values - since i dont know how to proper use an array it looks like that
    trashL = analogRead(leftIRPin);
    trashR = analogRead(rightIRPin);
  leftSensor0 = analogRead(leftIRPin);
  rightSensor0 = analogRead(rightIRPin);
  leftSensor1 = analogRead(leftIRPin);
  rightSensor1 = analogRead(rightIRPin);
  leftSensor2 = analogRead(leftIRPin);
  rightSensor2 = analogRead(rightIRPin);
  leftSensor3 = analogRead(leftIRPin);
  rightSensor3 = analogRead(rightIRPin);
  leftSensor4 = analogRead(leftIRPin);
  rightSensor4 = analogRead(rightIRPin);
    
    //calculate the avg out of the raw sensor data ... again an array would be superbé
    leftSensorAvg = ((leftSensor0 + leftSensor1 + leftSensor2 +
                      leftSensor3 + leftSensor4)/5);
    
    rightSensorAvg = ((rightSensor0 + rightSensor1 + rightSensor2 +
                       rightSensor3 + rightSensor4)/5);
                       
    //finally we get our good sensor data
    leftSensor = leftSensorAvg - leftSensorAdjust;
    rightSensor = rightSensorAvg - rightSensorAdjust;

    
    leftError = leftTarget - leftSensor;
    rightError = rightTarget - rightSensor;
    
    #ifdef DEBUGSENSOR
    Serial.println(String(leftSensor) + " " + String(rightSensor)
    + " " + String(leftSensor2) + " " + String(rightSensor3) + " " + String(jetzt));
    #endif
    }


    //Heres how we adjust the motorspeeds!
    //TODO adding the other parts of our PID! 
    //from now on on we totally ignore our second sensor
    
    turn = leftError * kP;

    leftSpeed = leftTargetSpeed + turn;
    rightSpeed = rightTargetSpeed - turn;
    
    //Adjusting the Motorspeeds to get the turn!
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);

#ifdef DEBUG
    Serial.println(String(leftError) + " " + String(rightError)
    + " " + String(leftSpeed) + " " + String(rightSpeed));
#endif
    
  }
