#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 5
// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Starting Joint Angles
int Joint1Angle = 90;
int Joint2Angle = 90;
int Joint3Angle = 90;
int GripperOpen = 30; // Open gripper; Need to tune value
int GripperClose = 120; // Close gripper; Need to tune value
// Joint Angle Offsets
int Joint1Offset = 10; // Your value may be different
int Joint2Offset = 30; // Your value may be different
int Joint3Offset = 0; // Your value may be different


void setup()
{
  Serial.begin(9600);
  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Gripper.attach(GripperPin);
  
  // Set initial positions
  Joint1.write(Joint1Angle + Joint1Offset);
  Joint2.write(Joint2Angle + Joint2Offset);
  Joint3.write(Joint3Angle + Joint3Offset);
  Gripper.write(GripperOpen); // Open gripper

  delay(5000); // Wait 5 seconds before moving Joint 1
}

void loop()
{
  // Move Joint 1 to 90 degrees, then back to original position
  Joint2.write(Joint2Offset); // Move Joint 1 to 90 degrees
  delay(500); // Wait for 1 second to allow it to move

  Joint2.write(30+Joint2Offset); // Move Joint 1 to 90 degrees
  delay(500); // Wait for 1 second to allow it to move

  Joint2.write(60+Joint2Offset); // Move Joint 1 to 90 degrees
  delay(500); // Wait for 1 second to allow it to move

  Joint2.write(90 + Joint2Offset); // Return to the starting position
  delay(500); // Wait for 1 second to allow it to return
  
  delay(500); // Small delay before next iteration (adjustable)
}
