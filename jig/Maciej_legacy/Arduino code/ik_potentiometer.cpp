#include <Servo.h>
#include <Arduino.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11

#define gripperOpen 15
#define gripperClose 140
#define averagingSize 10


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
 
int inputPin1 = A1;
int inputPin2 = A2;
int inputPin3 = A3;
int inputPin4 = A4;

float point[3] = {0, 0, 0};
int averagedReadings[averagingSize][4] = {0};
int memoryIndex = 0;
//measurements in cm
//point start frin center of joint 2
float Workbench_frame[3] = {-7.6, 6.5, 8.8};


float l2 = 9.6;
float l3 = 16;
float error_range = 0.1;

void updateServos(){
    joint1.write(joint1Val + joint1Offset);
    joint2.write(joint2Val + joint2Offset);
    joint3.write(joint3Val + joint3Offset);
    gripper.write(gripperCurrent);
}

//convert workbench frame to robot frame for inverse kinematics
void frameConversion(){
    float tempX = point[1] + Workbench_frame[0];
    float tempY = point[0] + Workbench_frame[1];
    float tempZ = -point[2] + Workbench_frame[2];
    point[0] = tempX;
    point[1] = tempY;
    point[2] = tempZ;
}

int* averageAnalogRead();

void setup()
{
    Serial.begin(9600);
    joint1.attach(Joint1Pin);
    joint2.attach(Joint2Pin);
    joint3.attach(Joint3Pin);
    gripper.attach(GripperPin);
    updateServos();
    for(int i = 0; i < averagingSize; i++){
        averageAnalogRead();
    }
    
}

int toDegrees(float radians){
    return round(radians * 180 / PI);
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

void scaleToMax(){
    float magnitude = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]) + error_range;
    if(magnitude >= l2 + l3){
        point[0] = point[0] * (l2 + l3) / magnitude;
        point[1] = point[1] * (l2 + l3) / magnitude;
        point[2] = point[2] * (l2 + l3) / magnitude;
    }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int* averageAnalogRead(){
    averagedReadings[memoryIndex][0] = analogRead(inputPin1);
    averagedReadings[memoryIndex][1] = analogRead(inputPin2);
    averagedReadings[memoryIndex][2] = analogRead(inputPin3);
    averagedReadings[memoryIndex][3] = analogRead(inputPin4);

    int* averagePoint = (int*)malloc(4 * sizeof(int));
    averagePoint[0] = 0;
    averagePoint[1] = 0;
    averagePoint[2] = 0;
    averagePoint[3] = 0;

    for (int i = 0; i < averagingSize; i++){
        averagePoint[0] += averagedReadings[i][0];
        averagePoint[1] += averagedReadings[i][1];
        averagePoint[2] += averagedReadings[i][2];
        averagePoint[3] += averagedReadings[i][3];
    }
    averagePoint[0] /= averagingSize;
    averagePoint[1] /= averagingSize;
    averagePoint[2] /= averagingSize;
    averagePoint[3] /= averagingSize;

    memoryIndex = (memoryIndex + 1) % averagingSize;
    return averagePoint;
}

void readInput(){
    int input1, input2, input3, input4;
    int* inputs = averageAnalogRead();
    //destructur inputs into variables
    input1 = inputs[0];
    input2 = inputs[1];
    input3 = inputs[2];
    input4 = inputs[3];
    free(inputs);

    point[0] = mapFloat(input1, 0, 1023, 0, 20);
    point[1] = mapFloat(input2, 0, 1023, -20, 20);
    point[2] = mapFloat(input3, 0, 1023, 0, 40);
    gripperCurrent = mapFloat(input4, 0, 1023, gripperOpen, gripperClose);
    //gripperCurrent = map(input4, 0, 1023, gripperOpen, gripperClose);

    Serial.print(" x: ");
    Serial.print(point[0]);
    Serial.print(" y: ");
    Serial.print(point[1]);
    Serial.print(" z: ");
    Serial.println(point[2]);
}

void loop()
{
    readInput();
    frameConversion();
    scaleToMax();
    inverseKinematics(point[0], point[1], point[2]);
    updateServos();

    delay(10);
}