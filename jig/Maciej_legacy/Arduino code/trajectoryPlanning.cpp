#include <Servo.h>
#include <Arduino.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11

#define gripperOpen 15
#define gripperClose 140

#define pointCount 3

// Servo Objects
Servo joint1;
Servo joint2;
Servo joint3;
Servo gripper;
// Starting Joint Angles at 0 the robot is straight and looking forward
int joint1Val = 90;
int joint2Val = 90;
int joint3Val = 90;
int joint1Offset = 5;
int joint2Offset = -10;
int joint3Offset = 5;
int gripperCurrent = gripperOpen;

float timeStep = 0.1;
float travelTime = 5; 
 
//measurements in cm
//point start frin center of joint 2
float pointOffset[3] = {-7.6, 6.5, 8.8};

float points[pointCount][3] = {
    {10, 5, 15},
    {4.8, -2.8, 5},
    {12.2, 17.2, 5}
};

int pointIndex = 0;

float l2 = 9.6;
float l3 = 16;
float error_range = 0.1;

void updateServos(){
    joint1.write(joint1Val + joint1Offset);
    joint2.write(joint2Val + joint2Offset);
    joint3.write(joint3Val + joint3Offset);
}

//convert user friendly frame to robot frame
void frameConversion(float* point){
    float tempX = point[1] + pointOffset[0];
    float tempY = point[0] + pointOffset[1];
    float tempZ = -point[2] + pointOffset[2];
    point[0] = tempX;
    point[1] = tempY;
    point[2] = tempZ;
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

//recomputes the points to align with the new offset and scale them to the max length of the arm
void computePoints(){
    for(int i = 0; i < pointCount; i++){
        frameConversion(points[i]);
        scaleToMax(points[i]);
    }
}

void inverseKinematics(float*);

void setup()
{
    computePoints();
    Serial.begin(9600);
    joint1.attach(Joint1Pin);
    joint2.attach(Joint2Pin);
    joint3.attach(Joint3Pin);
    gripper.attach(GripperPin);
    inverseKinematics(points[pointIndex]);
    updateServos();
    gripper.write(gripperCurrent);
    delay(3000);
}

int toDegrees(float radians){
    return round(radians * 180 / PI);
}


void inverseKinematics(float* point){
    float x = point[0];
    float y = point[1];
    float z = point[2];
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
}

float valueInterpolation(float from, float to, float t){
    return from + 3*t*t*(to-from) - 2*t*t*t*(to-from);
}

float* pointInterpolation(float* from, float* to, float t){
    float* result = (float*)malloc(sizeof(float) * 3);
    result[0] = valueInterpolation(from[0], to[0], t);
    result[1] = valueInterpolation(from[1], to[1], t);
    result[2] = valueInterpolation(from[2], to[2], t);
    return result;
}

void setTrajectory(float* from, float* to){
    for (float t = 0; t < travelTime; t += timeStep)
    {
        float *point = pointInterpolation(from, to, t / travelTime);
        inverseKinematics(point);
        free(point);
        updateServos();
        delay(long(timeStep * 1000));
    }
}

void loop()
{
    float* prevPoint = points[pointIndex];
    pointIndex++;
    if (pointIndex >= pointCount){
        pointIndex = 0;
    }

    setTrajectory(prevPoint, points[pointIndex]);
    updateServos();

    delay(3000);
}