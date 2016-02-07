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

const byte probeCount = 5;

int poti = A3; 

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
#endif

//motorPID
int actualSpeedA; //our tacho
int wantedSpeedB;
int wantedSpeedA; //the value set by our pid sensor cycle
int errorSpeedA; //the value used to adjust motor power


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
int altwert;
int neuwert;
uint16_t altwertA;
uint16_t SpeedA;
const byte deltaTacho=100; // chose your value according to your prefered max range 

//greifarm stuff
int8_t schrittTab[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; 

//motorregelung

const float kM =1;
const float kMI = 0.8;
const float kMD = 0;
float errorIntegral;
int aSA;
int der;


//delta t time loop
long jetzt = 0;
long altZeit, altZeitb;
const byte deltaT = 15; //we have to see if this is to high or to low

void setup() {

  //Interrupt stuff
PCICR |= (1 << PCIE2); //arduino will react on interrupts on port D
PCMSK2 |= (1 << LINKERMOTORPIN); // 2 defines the pin which listens for motor encoder (must be on portD)
PCMSK2 |= (1 << RECHTERMOTORPIN); // for the second motor ** 00001100
PCMSK2 |= (1 << GREIFARMPINA);
PCMSK2 |= (1 << GREIFARMPINB);
DDRD&=~PCMSK2; //if DDRD is 0 on the specific bit = input
PORTD |= PCMSK2; //arduino intern pullup resistor


  
  Serial.begin(9600);           // set up Serial baud to 115200 bps
  AFMS.begin();  // start motorshield with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
    //We are reading both values and trash them. the first readings are just that.
    // Set motor direction
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
 //     leftMotor->run(RELEASE);
//  leftMotor->run(BACKWARD);
    //  rightMotor->run(BACKWARD);
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


      wantedSpeedA =map(analogRead(poti),0,1024,0,140);
    altZeitb = jetzt;    
    actualSpeedA=(counterA-altwertA)*100/deltaTacho;
    altwertA=counterA;


    

      //tempomat
      errorSpeedA = wantedSpeedA - actualSpeedA;
      errorIntegral += errorSpeedA;


    der=errorSpeedA - aSA;
    aSA=errorSpeedA;

      leftPower = errorSpeedA * kM + errorIntegral * kMI + wantedSpeedA *2.5 + der*kMD; 
//wantedSpeedA *2
         leftPower = powerOverflowProtection(leftPower);
         rightPower = powerOverflowProtection(rightPower);
         
    leftMotor->setSpeed(leftPower);
    rightMotor->setSpeed(rightPower);


 
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
    integral = integral + midError;
    if(midError<=10 && midError>=-10){
      integral = 0;
    }
      
    //derivate stuff
    //adding the nessesary things for the derivative -Whoppa FlipFlop Style
    flip++;
    if((flip % 2) == 0){
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
      
    //Heres how we adjust the motorspeeds!
    //from now on on we totally ignore our secondary sensor
    turn = midError * kP + kI * integral + kD * derivative;

//    wantedSpeedA = leftTargetSpeed + turn;
//    wantedSpeedB = rightTargetSpeed - turn;
//      rightPower = leftTargetSpeed + turn;
//      leftPower = rightTargetSpeed - turn;


    
    
    //Adjusting the Motorspeeds to get the turn!(via i2c)
  //  leftMotor->setSpeed(leftSpeed);
  //  rightMotor->setSpeed(rightSpeed);



      debugInfos();

      
    } //end of the timed loop

    
  
    
      
  }// end of the void loop

      byte powerOverflowProtection(int x){
        if (x >= 255){
          return 255;
          } else if(x <=0) {
            return 0;
            }  else {
              return byte(x);
            }
        
      }
inline void debugInfos(){

    #ifdef MOTOR
    Serial.println(String(errorSpeedA) + "\t" + String(leftPower) + "\t" + String(actualSpeedA) + "\t" + String(errorIntegral) + "\t" + "\t" + String(wantedSpeedA) );
    #endif
  
    #ifdef DEBUG
    Serial.println(String(midError) + "\t " + String(integral) + "\t" + String(derivative) + "\t" + String(turn) + "\t" + String(leftSpeed) + "\t" + String(rightSpeed));
    #endif

    #ifdef SENSORADJUST
    Serial.println(String("Mavg: ") + String(midSensorAvg) + String("Lavg: ") + String(leftSensorAvg) + String("Ravg: ") + String(rightSensorAvg) +
    String("ML = 0?: ") + String(ml) + String("MR = 0?: ") + String(mr));
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
  
