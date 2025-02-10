#include <Servo.h>
#include <Arduino.h>

// Arduino Pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11
#define DiodePin 13

// Gripper States
#define gripperOpen 15
#define gripperClose 140


// Servos
Servo joint1;
Servo joint2;
Servo joint3;
Servo gripper;

// Servo joint angles
int joint1Val = 90;
int joint2Val = 90;
int joint3Val = 90;

// Servo joint offsets
int joint1Offset = 5;
int joint2Offset = -10;
int joint3Offset = 5;

// trajectory planning
float timeStep = 0.02; 
float travelTime = 3; //Tf
 

//measurements in cm

//coodinate of workspace frame (user friendly frame) compared to origin in robot frame
float pointOffset[3] = {-7.6, 6.5, 8.8};

//coordinates of start and relay points in workspace frame
float startPoint[3] = {9.5, 7.6, 18.4};

// Number of points in the path
#define pointCount 3

float points[pointCount][3] = {
    {3, -2.5, 10},
    {6, -5, 6},
    {9.5, 7.6, 18.4}
};

//gripper state for each realy point
float gripperState[pointCount] = {
    gripperOpen,
    gripperClose,
    gripperOpen
};

//flags for if the point is out of range (lights up diode)
bool outOfRange[pointCount] = {false};


float* prevPoint = startPoint;
int pointIndex = -1;

//lengths of the arm segments
float l2 = 9.6;
float l3 = 16;

//error range for the scaleToMax function to account for floating point errors
float error_range = 0.1; 

//
//servo control section
//

//update the servos to the current joint angles
void updateServos(){
    joint1.write(joint1Val + joint1Offset);
    joint2.write(joint2Val + joint2Offset);
    joint3.write(joint3Val + joint3Offset);
}

//update the gripper to the given state
void updateGripper(int state){
    gripper.write(state);
}

//update the diode to the given state
void updateDiode(bool state){
    digitalWrite(DiodePin, state);
}

//
// precomputation for frame conversion and scaling section
//

//convert user friendly frame to robot frame
void frameConversion(float* point){
    float tempX = point[1] + pointOffset[0];
    float tempY = point[0] + pointOffset[1];
    float tempZ = -point[2] + pointOffset[2];
    point[0] = tempX;
    point[1] = tempY;
    point[2] = tempZ;
}

//scale point to max length of arm if it is out of range
//returns true if scaled and falsed if untouched
bool scaleToMax(float* point){
    float magnitude = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]) + error_range;
    if(magnitude >= l2 + l3){
        point[0] = point[0] * (l2 + l3) / magnitude;
        point[1] = point[1] * (l2 + l3) / magnitude;
        point[2] = point[2] * (l2 + l3) / magnitude;
        return true;
    }
    return false;
}

//recomputes the points to align with robot frame and scale them to the max length of the arm
void computePoints(){
    frameConversion(startPoint);
    for(int i = 0; i < pointCount; i++){
        frameConversion(points[i]);
        outOfRange[i] = scaleToMax(points[i]);
    }
}

//
//inverse kinematics section
//

//convert radians to degrees
int toDegrees(float radians){
    return round(radians * 180 / PI);
}

//update the joint angles to point to the given point 
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

//
//trajectory planning section 
//

//apply a cubic interpolation between the two values, with t as the interpolation factor
float valueInterpolation(float from, float to, float t){
    return from + 3*t*t*(to-from) - 2*t*t*t*(to-from);
}

//apply a cubic interpolation between the two points, with t as the interpolation factor
float* pointInterpolation(float* from, float* to, float t){
    float* result = (float*)malloc(sizeof(float) * 3);
    result[0] = valueInterpolation(from[0], to[0], t);
    result[1] = valueInterpolation(from[1], to[1], t);
    result[2] = valueInterpolation(from[2], to[2], t);
    return result;
}

//move the from one point to another in a smooth trajectory
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

//main section
void setup()
{
    computePoints();
    Serial.begin(9600);

    //initialize pins
    joint1.attach(Joint1Pin);
    joint2.attach(Joint2Pin);
    joint3.attach(Joint3Pin);
    gripper.attach(GripperPin);

    //set initial position
    inverseKinematics(startPoint);
    updateServos();
    updateGripper(gripperOpen);

    delay(1000);
}

void loop()
{
    //increment point index
    pointIndex = (pointIndex + 1) % pointCount;

    //update position
    setTrajectory(prevPoint, points[pointIndex]);
    updateDiode(outOfRange[pointIndex]);
    delay(1000);

    //update gripper
    updateGripper(gripperState[pointIndex]);
    delay(1000);

    //update previous point
    prevPoint = points[pointIndex];
}