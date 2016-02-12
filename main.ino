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
//#define GREIFARM
//#define TACHO
#define NEU
//#define DEBUG // if active we are in the DEBUGGING mode, firing the Serial.println guns
//#define ZEIT
//#define SENSORADJUST // use this one FIRST:  *SensorAdjust to make all Sensors equal
//#define MIDSENSOR // use midTarget to make midError = 0 if Sensor is placed on grey
//#define LEFTSENSOR //
//#define RIGHTSENSOR //
//#define MOTOR

const byte probeCount = 5;

int poti = A3; 

//Sensor pins
int leftIRPin = A0;
int rightIRPin = A2;
int midIRPin = A1;

// Racedriver PID
const float kP = 0.2;
const float kI = 0.003;
const float kD = 0.;
int leftSensorAdjust = -14; //use those to calibrate sensors
int rightSensorAdjust = 0;
int midSensorAdjust = 0;

//TargetSpeed is the power the motors will get if we want the robot to drive straight.
const byte leftTargetSpeed = 20; // we are using a byte here, because the motorshield
const byte rightTargetSpeed = 20; // goes from 0-255.
const byte lessBack = 2;

//the target values for the sensors. if those are unchanged the robot will go straight
//place sensors between black and weight and insert proper readings
int leftTarget = 680;
int rightTarget = 680;
int midTarget = 690; //the darker the sourroundings the higer tis value should be

int leftSensorAvg;
int rightSensorAvg;
int midSensorAvg;
int leftSensor;
int rightSensor;
int midSensor;

//leftError is (the reading - the target)
int leftError;
int rightError;
int midError;

int pidError;

int turn;

long integral;

int rightPower;
int leftPower;

//derivative stuff
int derivative;
byte flip;
int bPrevError = 0;
int aPrevError = 0;

//values for motorspeed
int leftSpeed;
int rightSpeed;

//lightSensorStuff


#ifdef SENSORADJUST
int ml;
int mr; 
int lr;
#endif

//motorPID
int actualSpeedA;//our tacho
int actualSpeedB;
int wantedSpeedB;

int wantedSpeedAoK;
int wantedSpeedBoK;

int errorSpeedB;
int wantedSpeedA; //the value set by our pid sensor cycle
int errorSpeedA; //the value used to adjust motor power
int lgear;
int rgear;

//Interrupt motor adjust stuff

volatile  uint8_t lastB = 0;   
volatile  int16_t counterA;
volatile  int16_t counterB;
volatile int8_t altAB = 0;
volatile int8_t altCD = 0;
volatile long encoderWert = 0;
 

#define LINKERMOTORPINA 2
#define LINKERMOTORPINB 3

#define RECHTERMOTORPINA 4
#define RECHTERMOTORPINB 5

#define GREIFARMPINA 6
#define GREIFARMPINB 7

//tacho stuff
int altwert;
int neuwert;
int altwertA;
int SpeedA;
int altwertB;
int SpeedB;
const byte deltaTacho=100; // chose your value according to your prefered max range 

//greifarm stuff
int8_t schrittTab[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; 

//motorregelung

const float kM =1;
const float kMI = 0.8;
const float kMD = 0;
float errorIntegral;
float errorIntegralB;
int aSA;
int der;


//delta t time loop
long jetzt = 0;
long altZeit, altZeitb;
const byte deltaT = 15; //we have to see if this is to high or to low

void setup() {

  //Interrupt stuff
PCICR |= (1 << PCIE2); //arduino will react on interrupts on port D
PCMSK2 |= (1 << LINKERMOTORPINA);
PCMSK2 |= (1 << LINKERMOTORPINB); // 2 defines the pin which listens for motor encoder (must be on portD)
PCMSK2 |= (1 << RECHTERMOTORPINA);
PCMSK2 |= (1 << RECHTERMOTORPINB);// for the second motor ** 00001100
PCMSK2 |= (1 << GREIFARMPINA);
PCMSK2 |= (1 << GREIFARMPINB);
DDRD&=~PCMSK2; //if DDRD is 0 on the specific bit = input
PORTD |= PCMSK2; //arduino intern pullup resistor


  
  Serial.begin(9600);           // set up Serial baud to 115200 bps
  AFMS.begin();  // start motorshield with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
    //We are reading both values and trash them. the first readings are just that.
    // Set motor direction
 //     leftMotor->run(FORWARD);
 //     rightMotor->run(FORWARD);
 //     leftMotor->run(RELEASE);
 //     rightMotor->run(RELEASE);
 //    leftMotor->run(BACKWARD);
 //   rightMotor->run(BACKWARD);
     integral = 0 ;
     errorIntegral = 0;
  //TODO make tests for all them hardware!
//    Serial.println("Adafruit Motorshield v2 - ready!");
//    Serial.println("All Systems GO!");

interrupts();
}


void loop() {

  jetzt = millis();
  
  if((jetzt - altZeitb) > deltaTacho){


//      wantedSpeedA =map(analogRead(poti),0,1024,0,140);
//      wantedSpeedB = wantedSpeedA; //Insert proper pid regulated wanted speeds here
    altZeitb = jetzt;    
    actualSpeedA=(counterA-altwertA)*100/deltaTacho;
    altwertA=counterA;


    actualSpeedB=(counterB-altwertB)*100/deltaTacho;
    altwertB=counterB;


    

      //tempomat
      errorSpeedA = wantedSpeedA - abs(actualSpeedA);
      errorIntegral += errorSpeedA;

      errorSpeedB = wantedSpeedB - abs(actualSpeedB);
      errorIntegralB += errorSpeedB;


    der=errorSpeedA - aSA;
    aSA=errorSpeedA;


      leftPower = errorSpeedA * kM + errorIntegral * kMI + wantedSpeedAoK *2.5 + der*kMD; 
      rightPower = errorSpeedB * kM + errorIntegralB * kMI + wantedSpeedBoK *2.5 + der*kMD;
      

         leftSpeed = leftPowerControl(leftPower);
         rightSpeed = rightPowerControl(rightPower);
       
    leftMotor->setSpeed(leftSpeed); 
    rightMotor->setSpeed(rightSpeed);


 
  }  

  
  if((jetzt - altZeit) > deltaT){
    
   #ifdef ZEIT 
    Serial.println((jetzt - altZeit));
    #endif
    altZeit = jetzt;
    
      //TODO FIX OVERFLOW for jetzt
    //getting new values - since i dont know how to proper use an array it looks like that
   
    sensorAbfrage();
    
    //IntegralStuff
    //FIXME: maybe +-10 isnt big enough
    integral = integral + pidError;
    if(pidError<=10 && pidError>=-10){
      integral = 0;
    }
      
    //derivate stuff
    //adding the nessesary things for the derivative -Whoppa FlipFlop Style
    flip++;
    if((flip % 2) == 0){
      aPrevError = pidError;
      derivative = pidError - bPrevError;
      } else {
      bPrevError = pidError ;
      derivative = pidError - aPrevError;
        }
      
    //Heres how we adjust the motorspeeds!
    //from now on on we totally ignore our secondary sensor
    pidError = leftError - rightError;
    turn = pidError * kP + kI * integral + kD * derivative;

    wantedSpeedA = leftTargetSpeed + turn;
    wantedSpeedB = rightTargetSpeed - turn;
//      rightPower = leftTargetSpeed + turn;
//      leftPower = rightTargetSpeed - turn;


    
    
    //Adjusting the Motorspeeds to get the turn!(via i2c)
  //  leftMotor->setSpeed(leftSpeed);
  //  ->setSpeed(rightSpeed);



      debugInfos();

      
    } //end of the timed loop


    
    if(wantedSpeedA <= 0){
      leftMotor->run(BACKWARD);
      lgear = -1;
      wantedSpeedAoK = -1 * wantedSpeedA;
    }if(wantedSpeedA > 0) {
      leftMotor->run(FORWARD);
      wantedSpeedAoK = wantedSpeedA;
      lgear = 1;
    }
    
    if(wantedSpeedB <= 0){
      rightMotor->run(BACKWARD);
      rgear = -1;
      wantedSpeedBoK = -1* wantedSpeedB;
    }if(wantedSpeedB > 0) {
      rightMotor->run(FORWARD);
      wantedSpeedBoK = wantedSpeedB;
      rgear = 1;
    }
    
  
    
      
  }// end of the void loop



    byte leftPowerControl(int x){
          if (x >= 255){
          return 255;
          }
          if (x <= 0){
            return 0;
          }
              else {
              return byte(x);
            }
          }
    byte rightPowerControl(int y){
          if (y >= 255){
          return 255;
          }
          if (y <= 0){
            return 0;
          }
              else {
              return byte(y);
            }
          }
        
inline void debugInfos(){
    #ifdef GREIFARM
    Serial.println(String(actualSpeedB));
    #endif
  
    #ifdef TACHO
    Serial.println(String(actualSpeedA) + "\t" + String(actualSpeedB));
    #endif
    
    #ifdef NEU
    Serial.println(String(wantedSpeedA) + "\t" + String(wantedSpeedAoK) + "\t" + String(lgear) + "\t" + String(wantedSpeedB) + "\t" + String(wantedSpeedBoK) + "\t" + String(rgear));
    #endif

    #ifdef MOTOR
    Serial.println(String(errorSpeedA) + "\t" + String(leftPower) + "\t" + String(actualSpeedA) + "\t" + String(errorIntegral) + "\t" + "\t" + String(wantedSpeedA) + "\t" + String(actualSpeedB) );
    #endif
  
    #ifdef DEBUG
    Serial.println(String(pidError) + "\t " + String(integral) + "\t" + String(derivative) + "\t" + String(turn) + "\t" + String(wantedSpeedA) + "\t" + String(wantedSpeedB));
    #endif

    #ifdef SENSORADJUST
    Serial.println(String("Lavg: ") + String(leftSensorAvg) + String(" on grey? 0=>?: ") + String(leftError) + String("Ravg: ") + String(rightSensorAvg) + String(" on grey? 0=>?: ") + String(rightError) + "\t" + String(pidError));
    #endif

    #ifdef MIDSENSOR
    Serial.println(String("Mavg: ") + String(midSensorAvg)  + String(" on grey? 0=>?: ") + String(midError));
    #endif

    
    #ifdef LEFTSENSOR
    Serial.println(String("Lavg: ") + String(leftSensorAvg) + String(" on grey? 0=>?: ") + String(leftError));
    #endif

    
    #ifdef RIGHTSENSOR
    Serial.println(String("Ravg: ") + String(rightSensorAvg) + String(" on grey? 0=>?: ") + String(rightError));
    #endif
     
  
}


inline int sensorMittelWert(byte sensorPin, byte probeCountHorst){

    int trash;
    long sum = 0;
    
    trash = analogRead(sensorPin);
    for(byte i = 0; i<probeCountHorst ;i++){
      sum += analogRead(sensorPin);
      }
      
      return sum/probeCountHorst;
    }


inline void sensorAbfrage(){
  
    //calculate the avg out of the raw sensor data ... again an array would be superbé
    leftSensorAvg = sensorMittelWert(leftIRPin, probeCount);
    rightSensorAvg = sensorMittelWert(rightIRPin, probeCount);
    midSensorAvg = sensorMittelWert(midIRPin, probeCount);
                       
    //finally we get our good sensor data
    leftSensor = leftSensorAvg - leftSensorAdjust;
    rightSensor = rightSensorAvg - rightSensorAdjust;
    midSensor = midSensorAvg - midSensorAdjust;
    
    leftError = leftTarget - leftSensor;
    rightError = rightTarget - rightSensor;
    midError = midTarget - midSensor;
  
 }


ISR(PCINT2_vect){ //speciifc routine

  uint8_t newB = PIND;                  // aktuellen Port speichern
  uint8_t chgB = newB ^ lastB;          // geänderte Bits zum letzten Port
  lastB = newB;                         // neuer Port wird zum alten für die nächste Runde

if (chgB & ((1 <<LINKERMOTORPINA) | (1 << LINKERMOTORPINB)) ){
        altCD <<= 2; 
        altCD &= B00001100;

  altCD |= (((1 << LINKERMOTORPINA) & newB) >> (LINKERMOTORPINA-1)) | ((1 << LINKERMOTORPINB) & newB)>>LINKERMOTORPINB;
//        altCD |= (digitalRead(LINKERMOTORPINA) << 1) | digitalRead(LINKERMOTORPINB); 
        counterA -= schrittTab[altCD];
    }

if (chgB & ((1 <<RECHTERMOTORPINA) | (1 << RECHTERMOTORPINB)) ){
        altAB <<= 2; 
        altAB &= B00001100;

  altAB |= (((1 << RECHTERMOTORPINA) & newB) >> (RECHTERMOTORPINA-1)) | ((1 << RECHTERMOTORPINB) & newB)>>RECHTERMOTORPINB;
         
        counterB -= schrittTab[altAB];
    }

    
//    if (chgB & (1 << LINKERMOTORPIN)) {
//     counterA++;
//    }
//     if (chgB & (1 << RECHTERMOTORPIN)) {
//     counterB++;
//    }
    if (chgB & ((1 <<GREIFARMPINA) | (1 << GREIFARMPINB)) ){
        altAB <<= 2; 
        altAB &= B00001100;

  altAB |= (((1 << GREIFARMPINA) & newB) >> (GREIFARMPINA-1)) | ((1 << GREIFARMPINB) & newB)>>GREIFARMPINB;
  //    altAB |= (digitalRead(GREIFARMPINA) << 1) | digitalRead(GREIFARMPINB); 
         
        encoderWert -= schrittTab[altAB];
    }
}
  
