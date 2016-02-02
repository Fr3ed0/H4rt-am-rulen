//Copyright Frederic "Fre3d0" Wolf
//2015-2016

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <math.h>


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);

//#define DEBUG
byte a = digitalRead(19);
byte b = digitalRead(20);
int now;
int zeit = 100;
int k;
int g;
int h;
int tempoa;
int tempob;
int tempoaMax;
int tempobMax;
int tempo;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  
  rightMotor->setSpeed(120);
  leftMotor->setSpeed(120);
  
  #ifdef DEBUG
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  #endif
}

void loop() {
 if((millis() - now) > zeit){
    now = millis();
    h++;
 }
  g = h % 2;
  if(g == 0){
    if(a > 0){
      tempob = 0;
      tempoa ++;
      }
  
  }
  if(g == 1){
    tempoaMax = tempoa;
    tempoa = 0;
    tempob ++;
  }
  
    tempo = tempoaMax + tempobMax;
    Serial.println(int(tempo));
}









