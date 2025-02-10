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
Servo joint1;
Servo joint2;
Servo joint3;
Servo gripper;
// Starting Joint Angles at 0 the robot is straight and looking forward
int joint1Val = 90;
int joint2Val = 90;
int joint3Val = 90;
int joint1Offset = 0;
int joint2Offset = 0;
int joint3Offset = 0;
int gripperCurrent = gripperOpen;

 
//measurements in cm
//point start frin center of joint 2
float pointOffset[3] = {0, 0, 0};

float points[1][3] = {
    {0, 10, -5},
};
int pointCount = 1;
int pointIndex = 0;

float l2 = 9.6;
float l3 = 16;
float error_range = 0.1;

void updateServos(){
    joint1.write(joint1Val + joint1Offset);
    joint2.write(joint2Val + joint2Offset);
    joint3.write(joint3Val + joint3Offset);
}

void setup()
{
    Serial.begin(9600);
    joint1.attach(Joint1Pin);
    joint2.attach(Joint2Pin);
    joint3.attach(Joint3Pin);
    gripper.attach(GripperPin);
    updateServos();
    gripper.write(gripperCurrent);
    delay(3000);
}

int toDegrees(float radians){
    return round(radians * 180 / PI);
}


//scale point to max length of arm if it is too long
void scaleToMax(float* point){
    float magnitude = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]) + error_range;
    if(magnitude >= l2 + l3){
        point[0] = point[0] * (l2 + l3) / magnitude;
        point[1] = point[1] * (l2 + l3) / magnitude;
        point[2] = point[2] * (l2 + l3) / magnitude;
    }
}

void inverseKinematics(float x, float y, float z){
    float w = sqrt(x*x + y*y);
    float beta = atan2(-z, w);
    float psi = acos((w*w + z*z + l2*l2 - l3*l3) / (2*l2*sqrt(w*w + z*z)));
    float theta1 = atan2(y, x);
    float theta2 = beta + psi;
    float theta3 = PI - acos((w*w + z*z - l2*l2 - l3*l3) / (2*l2*l3));

    int angle1 = toDegrees(theta1);
    int angle2 = toDegrees(theta2);
    int angle3 = toDegrees(theta3);

    joint1Val = angle1;
    joint2Val = angle2;
    joint3Val = angle3;

    Serial.print(" theta1: ");
    Serial.print(angle1);
    Serial.print(" theta2: ");
    Serial.print(angle2);
    Serial.print(" theta3: ");
    Serial.println(angle3);

}

void loop()
{
    pointIndex++;
    if (pointIndex >= pointCount){
        pointIndex = 0;
    }
    
    float offsetedPoint[3] = { 
        points[pointIndex][0] + pointOffset[0], 
        points[pointIndex][1] + pointOffset[1],
        points[pointIndex][2] + pointOffset[2]
    };
    scaleToMax(offsetedPoint);
    inverseKinematics(offsetedPoint[0], offsetedPoint[1], offsetedPoint[2]);
    updateServos();

    delay(3000);
}