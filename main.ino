//=======================================================
//This Programm is made by Danfre wolfstoll and his army of minions
//System Design Project University Freiburg WS15/16
//Insert all Contributors here
//Made 2015/2016
//Insert any free software license here
//Thanks to J. Sluka http://www.inpharmix.com/
//Thanks to the ginger guy from github
//=======================================================


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>

//Initilise MotorShield with standard i2c port
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//assign motors to ports
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(1);

//Define Light Sensor Pins
int leftSensor = A0;
int rightSensor = A1;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
