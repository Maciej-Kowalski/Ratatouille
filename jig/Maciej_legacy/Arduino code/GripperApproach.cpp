#include <Servo.h>
#include <Arduino.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11

#define gripperOpen 15
#define gripperClose 140



// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Starting Joint Angles
int Joint1Val = 90;
int Joint2Val = 0;
int Joint3Val = 180;
int GripperCurrent = gripperOpen;
bool opening = false;
int gripperSpeed = 10;
int baseSpeed = 1;
int setPoint = gripperClose - 50;

void setup()
{
    Serial.begin(9600);
    Joint1.attach(Joint1Pin);
    Joint2.attach(Joint2Pin);
    Joint3.attach(Joint3Pin);
    Gripper.attach(GripperPin);
    Joint1.write(Joint1Val);
    Joint2.write(Joint2Val);
    Joint3.write(Joint3Val);
    Gripper.write(GripperCurrent);
}

void loop()
{
    //increment gripper by an amount proportional to how close it is to gripper close
    if(opening){
        GripperCurrent -= gripperSpeed;
    }else if(GripperCurrent < setPoint){
        GripperCurrent += (gripperSpeed - baseSpeed) * (setPoint - GripperCurrent) / (setPoint - gripperOpen) + baseSpeed;
    }else{
        GripperCurrent += baseSpeed;
    }

    bool pause = false;
    if(GripperCurrent > gripperClose && !opening)
    {
        GripperCurrent = gripperClose;
        opening = true;
        pause = true;
    }
    else if(GripperCurrent < gripperOpen && opening)
    {
        GripperCurrent = gripperOpen;
        opening = false;
        pause = true;
    }


    Gripper.write(GripperCurrent);
    Serial.println(GripperCurrent);
    int delayTime = pause ? 3000 : 20;
    delay(delayTime);
}