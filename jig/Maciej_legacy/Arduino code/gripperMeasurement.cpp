#include <Servo.h>
#include <Arduino.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11

#define gripperOpen 50
#define gripperClose 140

int potentiometerUsage = A1; 

int gripperInput = A4;
int gripperInputValue = 0;
int stepWidth = 5;

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Starting Joint Angles
int Joint1Min = 90;
int Joint2Min = 0;
int Joint3Min = 180;
int GripperMin = 0;
int GripperMax = 180;
int GripperCurrent = GripperMax;
bool positive = false;
int GripperIncrement = 5;

void setup()
{
    Serial.begin(9600);
    Joint1.attach(Joint1Pin);
    Joint2.attach(Joint2Pin);
    Joint3.attach(Joint3Pin);
    Gripper.attach(GripperPin);
    Joint1.write(Joint1Min);
    Joint2.write(Joint2Min);
    Joint3.write(Joint3Min);
    Gripper.write(GripperCurrent);

    //read from potentiometer
}

void loop()
{
    bool usePotentiometer = analogRead(potentiometerUsage) > 512;

    if(usePotentiometer){
        int gripperValue = analogRead(gripperInput);
        GripperCurrent = map(gripperValue, 0, 1023, 0, 180/stepWidth) * stepWidth;
    }else{
        if (GripperCurrent < GripperMin || GripperCurrent > GripperMax)
        {
            positive = !positive;
        }
        if (positive)
        {
            GripperCurrent+= GripperIncrement;
        }
        else
        {
            GripperCurrent-= GripperIncrement;
        }
    }

    Gripper.write(GripperCurrent);
    Serial.println(GripperCurrent);
    int delayTime = usePotentiometer ? 100 : 3000;
    delay(delayTime);
}