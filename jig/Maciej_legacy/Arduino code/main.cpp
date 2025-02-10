#include <Servo.h>
#include <Arduino.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11

#define gripperOpen 50
#define gripperClose 140

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Starting Joint Angles
int Joint1Min = 90;
int Joint1Max = 180;
int Joint2Min = 0;
int Joint2Max = 100;
int Joint3Min = 180;
int Joint3Max = 60;
int GripperAngle = gripperClose;

void setup()
{
  Serial.begin(9600);
  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Gripper.attach(GripperPin);
}

void extend(int delayTime = 0){
  Joint3.write(Joint3Min);
  Joint2.write(Joint2Min);
  delay(delayTime);
}

void retract(int delayTime = 0){
  Joint2.write(Joint2Max);
  Joint3.write(Joint3Max);
  delay(delayTime);
}

void grab(int delayTime = 0){
  Gripper.write(gripperClose);
  delay(delayTime);
  
}

void letGo(int delayTime = 0){
  Gripper.write(gripperOpen);
  delay(delayTime);
}

void outPosition(int delayTime = 0){
  Joint1.write(Joint1Max);
  delay(delayTime);
}

void inPosition(int delayTime = 0){
  Joint1.write(Joint1Min);
  delay(delayTime);
}

void loop()
{
  letGo(1000);
  inPosition(1000);
  extend(1000);
  grab(1000);
  retract(1000);
  outPosition(1000);
}