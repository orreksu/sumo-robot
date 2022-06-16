/* Title: Prototype 5
 * Date: October 30, 2018
 * Project_KY is designed to control a sumo bot with variation of strategies in mind and ability to easily contribute with your own fight modes
 */

// LIBRARIES--------------------------------------------------------------------
#include <Servo.h>
#include <time.h>


// SETTING UP SERVOS, SENSORS & BUTTON------------------------------------------
Servo leftServo;
Servo rightServo;

const int downLeftFrontSensor = 11;
const int downRightFrontSensor = 10;

const int downLeftBackSensor = 12;
const int downRightBackSensor = 13;

const int trigPinFrontSensor = 8;
const int echoPinFrontSensor = 9;

const int onOfButtonPin = 7;

void setup() {
  leftServo.attach (5);
  rightServo.attach (6);

  pinMode (downLeftFrontSensor, INPUT);
  pinMode (downRightFrontSensor, INPUT);
  pinMode (downLeftBackSensor, INPUT);
  pinMode (downRightBackSensor, INPUT);

  pinMode (trigPinFrontSensor, OUTPUT);
  pinMode (echoPinFrontSensor, INPUT);

  pinMode(onOfButtonPin, INPUT);

  //Serial.begin (9600);                          // Setting serial output to 9600 speed
}


// DECLARING MODES SWITCHES-----------------------------------------------------
bool fightModeSwitch = false;                   // Setting the bool for Fight Mode to "false" at the start of the robot
bool calibrationModeSwitch = false;             // Change to "true" in case need to calibrate sensors and servos

bool detectionModeSwitch = true;                // Switch that shows if Detection Mode is turned "on", for the start of Fight Mode the robot goes straight to detection, so value is "true"
bool attackModeSwitch = false;                  // Switch that shows if Attack Mode turned "on"


// DECLARING MODES FUNCTIONS----------------------------------------------------
void fightModeFunction();                       // Function for the Fight Mode, a controller with if statements to turn on and off other modes
void waitModeFunction();                        // Function that is on, when Fight Mode is "false"

void detectionModeFunction(double frontSensorReadingInches);                   // Function for the process of detecting the opponent on the ring
void attackModeFunction(double frontSensorReadingInches);                      // Function fro attack of the opponent


// DECLARING GLOBAL CONSTANTS---------------------------------------------------
const double robotVelocity = 7.5;               // Constant velocity of the robot measured with 4AA Energizer batteries (In/sec)
const double secondsFor360Turn = 4;             // Constant amount of time it takes robot to turn 360 degrees in seconds measured with 4AA Energizer batteries


// DECLARING MAIN FUNCTIONS-----------------------------------------------------
void mainFunction();


// DECLARING MOVEMENT FUNCTIONS-------------------------------------------------
void basicMovementSec(int powerLeft, int powerRight, double moveSeconds);
void basicMovementForDegree(int powerLeft, int powerRight, double turnAngle);

void stopMovement();

void turnRightForSec(double moveSeconds);
void turnRightForDegree(double turnAngle);

void turnLeftForSec(double moveSeconds);
void turnLeftForDegree(double turnAngle);

void driveForwardForSec(double moveSeconds);
void driveBackwardForSec(double moveSeconds);


// DECLARING SENSORS FUNCTIONS--------------------------------------------------
bool leftFrontSensorDetectWhite();                  // Function for the left front ground sensor (outputs 1 for white and 0 for black)
bool rightFrontSensorDetectWhite();                 // Function for the right front ground sensor (outputs 1 for white and 0 for black)
bool leftBackSensorDetectWhite();                   // Function for the left back ground sensor (outputs 1 for white and 0 for black)
bool rightBackSensorDetectWhite();                  // Function for the right back ground sensor (outputs 1 for white and 0 for black)
float frontSensorDistanceInches ();                 // Function for the front sensor (outputs distance to the object in front of the robot)


// DECLARING ADDITIONAL FUNCTIONS-----------------------------------------------
void pressOnOffButton();                        // Function detects the press of the OnOff Button and changes the value of the Fight Mode
double angleToSeconds (double turnAngle);       // Function transforms desired turning angle to seconds for movement functions
void calibrationModeFunction();                 // Function that stops the servos, and creates a Serial Outputs for sensors to calibrate


// STANDARD LOOP----------------------------------------------------------------
void loop(void) {
  if (calibrationModeSwitch) {                  // If statement to go into Calibration Mode in case the Calibration Mode Switch is "true"
    calibrationModeFunction();
  } else {                                      // Else go into Main Function
    mainFunction();
  }
}

// MAIN FUNCTION----------------------------------------------------------------
void mainFunction() {
  pressOnOffButton();                           // Calls for the function that controls On/Off Button

  if (fightModeSwitch == true) {                // If fight Mode Switch is "true" go into fight mode function
    fightModeFunction();
  } else {                                      // Else go into Wait Function
    waitModeFunction();
  }
}

// MODES FUNCTIONS--------------------------------------------------------------
// Function for the Fight Mode, a controller with if statements to turn on and off other modes
void fightModeFunction(){
  // Connecting all sensor functions into local variables
  double frontSensorReadingInches = frontSensorDistanceInches();
  //bool leftFrontSensorOnWhite = leftFrontSensorDetectWhite();
  //bool rightFrontSensorOnWhite = rightFrontSensorDetectWhite();
  //bool leftBackSensorOnWhite = leftBackSensorDetectWhite();
  //bool rightBackSensorOnWhite = rightBackSensorDetectWhite();

  if (leftFrontSensorDetectWhite()){
    driveBackwardForSec(0.5);
    turnRightForDegree(90);
    driveForwardForSec(1);

    detectionModeSwitch = true;
    attackModeSwitch = false;
  }

  if (rightFrontSensorDetectWhite()){
    driveBackwardForSec(0.5);
    turnLeftForDegree(90);
    driveForwardForSec(1);

    detectionModeSwitch = true;
    attackModeSwitch = false;
  }

  if (leftBackSensorDetectWhite()){
    turnRightForDegree(60);
    driveForwardForSec(1);

    detectionModeSwitch = true;
    attackModeSwitch = false;
  }

  if (rightBackSensorDetectWhite()){
    turnLeftForDegree(60);
    driveForwardForSec(1);

    detectionModeSwitch = true;
    attackModeSwitch = false;
  }

  // If statements that are calling for Mode Functions in case Mode Switch is "true"
  if (detectionModeSwitch){
    detectionModeFunction(frontSensorReadingInches);
  }
  if (attackModeSwitch){
    attackModeFunction(frontSensorReadingInches);
  }
}

// Function that is on, when Fight Mode is "false", was created to give ability in future add something while waiting, instead of just "Stop Function"
void waitModeFunction(){
  stopMovement();

  if (leftFrontSensorDetectWhite() || rightFrontSensorDetectWhite() || leftBackSensorDetectWhite() || rightBackSensorDetectWhite()){
    fightModeSwitch = true;
  }

  if (frontSensorDistanceInches() <= 17){
    fightModeSwitch = true;
  }
}


void detectionModeFunction(double frontSensorReadingInches){
  //Serial.println("detectionModeFunction");
  if (frontSensorReadingInches > 17){
    turnLeftForSec(0);                             // Turns Left until front Sensor Detects an object within 17 inches
  } else {
    attackModeSwitch = true;                       // Turn Attack Mode On
    detectionModeSwitch = false;                   // Turn Detection Mode Off, because the opponent was detected
  }
}

void attackModeFunction(double frontSensorReadingInches){
  //Serial.println("attackModeFunction");
  if (frontSensorReadingInches <= 17){
    driveForwardForSec(0);                           // Continue Attack (moving straight into opponent) till the front sensor distance is less then 17 inches
  } else {                                           // When front sensor distance is more then 17 inches (the opponent was lost)
    detectionModeSwitch = true;
    attackModeSwitch = false;                        // Turn Attack Mode Off, as no opponent is seen
  }
}

/*void followModeFunction(double frontSensorReadingInches){
  //Serial.println("followModeFunction");
  turnRightForDegree(15);
  if (frontSensorReadingInches <= 17){
    attackModeSwitch = true;
    followModeSwitch = false;
    return;
  }
  turnLeftForDegree(30);
  if (frontSensorReadingInches <= 17){
    attackModeSwitch = true;
    followModeSwitch = false;
    return;
  }
  detectionModeSwitch = true;
  followModeSwitch = false;
}*/

// MOVEMENT FUNCTIONS-----------------------------------------------------------
// Main two movement function that control servos based on the input
void basicMovementForSec(int powerLeft, int powerRight, double moveSeconds) {
  int leftS, rightS;              // Declaring local variables for left and right servos

  // If input for first function variable is 1 - drive forward, if -1 - drive backward, else stop the servo
  if (powerLeft == 1) {
    leftS = 1000;
  } else if (powerLeft == -1) {
    leftS = 2000;
  } else {
    leftS = 1500;
  }

  // If input for first function variable is 1 - drive forward, if -1 - drive backward, else stop the servo
  if (powerRight == 1) {
    rightS = 2000;
  } else if (powerRight == -1) {
    rightS = 1000;
  } else {
    rightS = 1500;
  }

  // If input for time is 0 - drive forever, in no-zero, use the number as the number of seconds of the delay
  if (moveSeconds == 0) {
    leftServo.write(leftS);
    rightServo.write(rightS);
  } else {
    leftServo.write(leftS);
    rightServo.write(rightS);

    //Serial.println("moveSeconds = ");
    //Serial.println(moveSeconds * 1000);

    delay(moveSeconds * 1000);
  }
}
void basicMovementForDegree(int powerLeft, int powerRight, double turnAngle) {
  int leftS, rightS;               // Declaring local variables for left and right servos
  double moveSeconds;              // Declaring local variable for the number of seconds to move

  // If input for first function variable is 1 - drive forward, if -1 - drive backward, else stop the servo
  if (powerLeft == 1) {
    leftS = 1000;
  } else if (powerLeft == -1) {
    leftS = 2000;
  } else {
    leftS = 1500;
  }

  // If input for first function variable is 1 - drive forward, if -1 - drive backward, else stop the servo
  if (powerRight == 1) {
    rightS = 2000;
  } else if (powerRight == -1) {
    rightS = 1000;
  } else {
    rightS = 1500;
  }

  // Assigns moveSeconds a value from the function that transforms angle into the time needed for the turn
  moveSeconds = angleToSeconds(turnAngle);

  // If input for time is 0 - drive forever, in no-zero, use the number as the number of seconds of the delay
  if (moveSeconds == 0) {
    leftServo.write(leftS);
    rightServo.write(rightS);
  } else {
    leftServo.write(leftS);
    rightServo.write(rightS);

    //Serial.println("moveSeconds = ");
    //Serial.println(moveSeconds * 1000);

    delay(moveSeconds * 1000);
  }
}

// Other functions are created to improve the readability of the code and easier syntax
void stopMovement(){
  basicMovementForSec(0,0,0);
}

void turnRightForSec(double moveSeconds){
  basicMovementForSec(1, -1, moveSeconds);
}
void turnRightForDegree(double turnAngle){
  basicMovementForDegree(1, -1, turnAngle);
}

void turnLeftForSec(double moveSeconds){
  basicMovementForSec(-1, 1, moveSeconds);
}
void turnLeftForDegree(double turnAngle){
  basicMovementForDegree(-1, 1, turnAngle);
}

void driveForwardForSec(double moveSeconds){
  basicMovementForSec(1, 1, moveSeconds);
}
void driveBackwardForSec(double moveSeconds){
  basicMovementForSec(-1, -1, moveSeconds);
}


// SENSOR FUNCTIONS-------------------------------------------------------------
// Function for the left front ground sensor (outputs 1 for white and 0 for black)
bool leftFrontSensorDetectWhite() {
  int leftFrontSensorReading = digitalRead (downLeftFrontSensor);
  if (leftFrontSensorReading == 0){
    //Serial.println ("Left Front Detects White");
    return 1;
  } else {
    //Serial.println ("Left Front Detects Black");
    return 0;
  }
}

// Function for the right front ground sensor (outputs 1 for white and 0 for black)
bool rightFrontSensorDetectWhite() {
  int rightFrontSensorReading = digitalRead (downRightFrontSensor);
  if (rightFrontSensorReading == 0){
    //Serial.println ("Right Front Detects White");
    return 1;
  } else {
    //Serial.println ("Right Front Detects Black");
    return 0;
  }
}

// Function for the left back ground sensor (outputs 1 for white and 0 for black)
bool leftBackSensorDetectWhite() {
  int leftBackSensorReading = digitalRead (downLeftBackSensor);
  if (leftBackSensorReading == 0){
    //Serial.println ("Left Back Detects White");
    return 1;
  } else {
    //Serial.println ("Left Back Detects Black");
    return 0;
  }
}

// Function for the right back ground sensor (outputs 1 for white and 0 for black)
bool rightBackSensorDetectWhite() {
  int rightBackSensorReading = digitalRead (downRightBackSensor);
  if (rightBackSensorReading == 0){
    //Serial.println ("Right Back Detects White");
    return 1;
  } else {
    //Serial.println ("Right Back Detects Black");
    return 0;
  }
}

// Function for the front sensor (outputs distance to the object in front of the robot in inches)
float frontSensorDistanceInches () {
  float echoTime;
  float calculatedDistanceInches;

  digitalWrite (trigPinFrontSensor, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPinFrontSensor, LOW);

  echoTime = pulseIn(echoPinFrontSensor, HIGH);

  calculatedDistanceInches = echoTime / 148.0;

  return calculatedDistanceInches;
}


//ADDITIONAL FUNCTIONS----------------------------------------------------------
// Function check if On/Off Button pressed and changes Fight Mode status accordingly
void pressOnOffButton() {
  //read the state of the "On/Off" button value:
  int onOffButtonState = digitalRead(onOfButtonPin);

  //in case "On/Off" button is pressed change to and from the "fight mode"
  if (onOffButtonState == HIGH) {
    if (fightModeSwitch == false) {
      fightModeSwitch = true;
    } else {
      fightModeSwitch = false;
      detectionModeSwitch = true;
      attackModeSwitch = false;
    }
    delay(200); // adds delay for user to remove the finger
  }
}

// Function trans forms turning angle into time (s) based on the measured rotation speed
double angleToSeconds (double turnAngle) {
  double turnTime;
  turnTime = (turnAngle * secondsFor360Turn) / (360);
  //Serial.println ("turnTime --> ");
  //Serial.print (turnTime);
  return turnTime;
}

//END---------------------------------------------------------------------------
