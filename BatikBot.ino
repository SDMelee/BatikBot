/* 
This file coordinates two stepper motors on separate Adafruit V2.3 motor shields.
The file translates polar coordinates into simultaneous movement of the motors
to the specified positions. Positions are specified in the arrays radius[] and 
angle[] with length numPos. Positions are specified in inches and degrees. 
The maximum radius is 7.25", and the angle is limited to 0<theta<360 deg. 
If the angle is given outside of this range, it is automatically adjusted
to within [0, 360] *(needs to be implemented). Radius motor is automatically 
calibrated by a range finder at initialization. Radii r<0.25 inches are unavailable, 
and so the radius motor is positioned at r=0.25". Safety mechanisms need to be included 
in the software to prevent movement to 0.25<r<7.25.
To activate debugging, use the debug[] variable (see comments in GLOBAL VARIABLES
for a description of available debugging modes).
Note that the range finder does not accurately retrieve distances once the motor
objects have been initialized in setup. Disabling the motor output does not fix
this issue and it remains unresolved.
TODO:
  1. Check software safety mechanisms for 0.25<r<7.25
  2. Add code to deal with radial offest on y-axis if necessary
  3. Why doesn't the range-finder work after motor initialization?
  4. Update for newest AccelStepper library
  5. Setup plot modes
*/

#include <AccelStepper.h>                     // V1.30, For running motors simultaneously
#include <Wire.h>                             // Allows communication with I2C and TWI devices --> do we need this?
#include <Adafruit_MotorShield.h>             // For interfacing with Motor Shields
#include "utility/Adafruit_MS_PWMServoDriver.h"
//#include "Math.h"                           // For asin, only necessary if y_offset!=0
#include <avr/pgmspace.h>                     // Store data in flash (program) memory instead of SRAM

// CONSTANTS
#define numPos 80                             // Number of Positions to plot
// NOTE: If you run the motors too fast, the motor shield will OVERHEAT
#define radialSpeed 200                       // Radius Motor Speed in RPM
#define angularSpeed 120                      // Angular Speed in RPM
#define extruderSpeed 200                          // Resist applicator speed
#define calibrationDistance 8.92              // This is the distance from the ultrasonic range-finder for the 
                                              // radial arm to be positioned correctly (in inches)
#define plotMode 1                            // 0: Dots
                                              // 1: Continuous

//PINS
const int pingPin = 7;                        // Range-Finder Pin

// GLOBAL VARIABLES
float y_Offset = 0;                           // offset in y-direction of radial arm in inches (e.g., if plotter is positioned at (x,y)=(0,1.25)
                                              // instead of (x,y)=(0,0)). y_Offset will account calculate x-movement to produce r, and the corresponding
                                              // change in angle.
float x_Offset=0.25;                          // Starting position of the radial arm from origin in inches
int inputVal;                                 // used to store serial input
int currPos = 0;                              // Current index of position arrays
bool debug[5] = {0, 0, 0, 0, 0};              // Debug enable/disable flags, will output data to monitor if enabled
                                              // debug[0]: enable all debug messages
                                              // debug[1]: calibration debug
                                              // debug[2]: positioning debug
                                              // debug[3]: y_Offset debug
                                              // debug[4]: print position at each step
// POSITIONS
// Current coordinates plot a spiral pattern
const float PROGMEM radius[numPos] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2.75, 2.75, 2.75, 2.75, 2.75, 2.75, 2.75, 2.75, 2.75, 2.75, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5};            // inches
const float PROGMEM angle[numPos] =  {0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0, 18, 36, 54, 72, 90, 108, 126, 144, 162, 180, 198, 216, 234, 252, 270, 288, 306, 324, 360};      // degrees

// Create a motor shield object with the default I2C address, 0x60
Adafruit_MotorShield AFMSbot = Adafruit_MotorShield(0x60);
// Create a motor shield object with the I2C address 0x61 
Adafruit_MotorShield AFMStop = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *Angle       = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *Extruder    = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *Radius      = AFMStop.getStepper(200, 1);

// Create wrapper functions for use with the AccelStepper Library
// Can change to MICROSTEP for more accuracy, or DOUBLE for more torque
// Angular Motor
void forwardstepA() {  
  Angle->onestep(FORWARD, SINGLE);
}
void backwardstepA() {  
  Angle->onestep(BACKWARD, SINGLE);
}
// Radial Motor
void forwardstepR() {  
  Radius->onestep(FORWARD, SINGLE);
}
void backwardstepR() {  
  Radius->onestep(BACKWARD, SINGLE);
}
// Glue  Gun Press Motor
void forwardstepE() {
  Extruder->onestep(FORWARD, DOUBLE);  
}
void backwardstepE()  {
  Extruder->onestep(BACKWARD, DOUBLE);  
}
// Create Astepper and Rstepper objects from AccelStepper library
AccelStepper Astepper(forwardstepA, backwardstepA); // use functions to step
AccelStepper Rstepper(forwardstepR, backwardstepR); // use functions to step
AccelStepper Estepper(forwardstepE, backwardstepE); // use functions to step

// Reset function for program restart in case of failure
void(* resetFunc) (void) = 0;//declare reset function at address 0

void waitForInput(){
  Serial.println("Waiting for user input to start... Please make sure motor power is ON.");
  inputVal = 0;
  while (inputVal == 0){
    if(Serial.available()== 0){
      inputVal = Serial.parseInt(); //read int or parselong for ..long...
    }
  }
}

// Setup initialized motors, sets ther speeds, and calibrates position to (0.25", 0deg)
void setup() {
  Serial.begin(9600);               // set up Serial library at 9600 bps
  Serial.println("Plotting Program Start!");
  delay(5000);                      // Delay to allow rangers time to steup otherwise initial reading will be wrong
  float distance_=distance();       // distance for calibration needs to be grabbed before motors are initialized
  
  AFMStop.begin();                  // create with the default frequency 1.6KHz
  AFMSbot.begin();
  
  // Set speed in rpm
  Astepper.setSpeed(angularSpeed);
  Rstepper.setSpeed(radialSpeed);
  Estepper.setSpeed(extruderSpeed);

  // Wait for user input to start
  waitForInput();
  Serial.println("Beginning system calibration...");
  calibrate(distance_);
  moveToPolarCoord(pgm_read_float_near(&radius[currPos]), pgm_read_float_near(&angle[currPos]));
}

float distance(){
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);
  float distance_=(float) duration;
  distance_ = distance_ / 74.0 / 2.0; // convert microseconds to inches
  
  if(debug[0] || debug[1]){
    Serial.println("distance():");
    Serial.print("  Duration of ping in us: ");
    Serial.println(duration);
    Serial.print("  Converted distance from us to in: ");
    Serial.println(distance_);
  }
  
  return distance_;
}

void calibrate(float distance_){
  if(debug[0] || debug[1]){
    Serial.println("calibrate(float distance_):");
    Serial.print("  Calibration Distance is: ");
    Serial.print(calibrationDistance);
    Serial.print(", current position is: ");
    Serial.println(distance_);
  }
  float dist = distance_-calibrationDistance;
  if(dist<0){
    dist=abs(dist);
    long steps = (long) (dist/0.0015);
    Rstepper.move(steps);
    if(debug[0] || debug [1]){
      Serial.print("  Moving "); Serial.print(dist); Serial.println("inches inward");
      Serial.print("  From position "); Serial.print(Rstepper.currentPosition());
      Serial.print(" to"); Serial.println(Rstepper.targetPosition());
    }
    Rstepper.setSpeed(radialSpeed/2);
  }
  else{
    long steps = (long) (dist/0.0015);
    Rstepper.setCurrentPosition(steps);
    Rstepper.moveTo(0);
    if(debug[0] || debug [1]){
      Serial.print("  Moving "); Serial.print(dist); Serial.println("inches outward");
      Serial.print("  From position "); Serial.print(Rstepper.currentPosition());
      Serial.print(" to"); Serial.println(Rstepper.targetPosition());
    }
    Rstepper.setSpeed(-0.5*radialSpeed);
  }
  Rstepper.runToPosition();  
  // Set current position
  Astepper.setCurrentPosition(0); 
  long initialDist = (long) (x_Offset/0.0015);
  Rstepper.setCurrentPosition(initialDist);  // Initial radius  
}

// There are 0.186 deg/step for the angle motor set to 200 steps per rotation
// convertAngleToSteps() takes angle in degrees and returns number of steps
long convertAngleToSteps(float degrees_){
  long steps=(long) (degrees_/0.186);
  return steps;
}

float convertStepsToAngle(long steps){
  float degree = (float) (0.186*steps);
  return degree;
}

// There are 0.0015"/step for the radius motor set to 200 steps per rotation
long convertRadiusToSteps(float inches){
  // THIS WILL REMOVE RADIAL SIGN
  float x=sqrt(inches*inches-y_Offset*y_Offset);// If there is an offset of the radial arm
                                                // then lateral movement corresponds to the
                                                // projection of the desired radius with y=a
                                                // onto the x-axis
                                                // if the radial arm is perfectly aligned with
                                                // the x-axis, offset=0
 long steps= (long) (x/0.0015);
 return steps;
}

// convertStepsToRadius() not currently used
float convertStepsToRadius(long steps){
  float inches = (float) (steps)*0.0015;
  inches = inches-x_Offset;
  inches = abs(inches);
  inches = sqrt(inches*inches+y_Offset*y_Offset);
  return inches;
}

// move to Polar Coordinate position (r, theta) given in degrees and inches
void moveToPolarCoord(float r, float theta){
  // if there is a y-axis offset, then the change in angle due to a change
  // in radius must be accounted for. See documentation for derivation.
  /*if(y_Offset!=0 && convertStepsToRadius(Rstepper.currentPosition())!=r){
    float dtheta = asin(y_Offset/convertStepsToRadius(Astepper.currentPosition()))-asin(y_Offset/r); // returns value in radians
    dtheta = 57.3*dtheta; // Convert radians to degrees
    float thetaNew = convertStepsToAngle(Astepper.currentPosition())-dtheta;
    Serial.print("New angle is: ");
    Serial.println(thetaNew);
    Astepper.setCurrentPosition(convertAngleToSteps(thetaNew));
  }*/
  if(debug[0] || debug[2]){
    Serial.println("Moving to r/step; deg/step: ");
    Serial.print(r); Serial.print(", ");
    Serial.print(convertRadiusToSteps(r));
    Serial.print("; ");
    Serial.print(theta); Serial.print(", ");
    Serial.println(convertAngleToSteps(theta));
    Serial.print("From (r,theta): "); Serial.print(Rstepper.currentPosition()); 
    Serial.print(", "); Serial.println(Astepper.currentPosition());
  }
  if(convertRadiusToSteps(r)<Rstepper.currentPosition()){
    Rstepper.setSpeed(-1*radialSpeed);  
  }
  else{
    Rstepper.setSpeed(radialSpeed);  
  }
  if(convertAngleToSteps(theta)<Astepper.currentPosition()){
    Astepper.setSpeed(-1*angularSpeed);  
  }
  else{
    Astepper.setSpeed(angularSpeed);  
  }
  Astepper.moveTo(convertAngleToSteps(theta));
  Rstepper.moveTo(convertRadiusToSteps(r));
  // Pauses and waits for user input before each movement in debug mode
  if(debug[0] || debug [2]){
    waitForInput();
    // Input of '2' moves back a step from current
    if(inputVal == 2){
      currPos=currPos-2;  
    }
  }
}

void applyWax(){
  //Estepper.setSpeed(extruderSpeed);
  Estepper.move(50); // Move 2.5 rotations or 0.5cm down
  // Use non-blocked command in while-loop instead of blocked command
  // because blocked command does not work for some reason...
  Serial.println("Pressing syringe down");
  while(Estepper.currentPosition()!=Estepper.targetPosition()){
    Estepper.runSpeedToPosition();
  }
  Serial.println("Waiting for user input to move to next position!");
  inputVal = 0;
  waitForInput();
}

void loop() {
  // If we are at target position, set next position.
  if(Rstepper.currentPosition()==Rstepper.targetPosition() && Astepper.currentPosition()==Astepper.targetPosition() && currPos<numPos){
    applyWax();
    if(debug[0] || debug[4]){
      Serial.println("Setting next Position");
      Serial.print("currPos: ");
      Serial.println(currPos);
    }
    currPos++;
    moveToPolarCoord(pgm_read_float_near(&radius[currPos]), pgm_read_float_near(&angle[currPos]));  
  }

  if(currPos==numPos){
    Serial.println("Plotting Complete!");
    Serial.println("Disabling Motors...");
    Rstepper.disableOutputs();
    Astepper.disableOutputs();
    Estepper.disableOutputs();
    Serial.println("Resetting Program... Check for overheating on motors, shields, and supplies.");
    delay(1000);  // Delay to allow above message to finish printing to monitor
    resetFunc(); //call reset 
  }
  // Enabling this debug section will significantly slow motor movement!
  // Continuously prints motor position in steps in debug mode
  if(debug[0] || debug[2]){
    Serial.print("R Position: ");
    Serial.println(Rstepper.currentPosition());
  }
  if(debug[0] || debug[2]){
    Serial.print("A Position: ");
    Serial.println(Astepper.currentPosition());
  }
 //runSpeedToPosition() is not blocked! That is, it will be called multiple times throughout the loop()
  // to get to position. This is done to run both motors simultaneously. See AccelStepper.h for more information.
  Rstepper.runSpeedToPosition();    
  Astepper.runSpeedToPosition();
}
