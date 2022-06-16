/* Title: Prototype 5
 * Date: October 30, 2018
*/

// LIBRARIES--------------------------------------------------------------------
#include <Servo.h>
#include <time.h>


// CONNECTING SERVOS, SENSORS & BUTTON------------------------------------------
Servo leftServo;
Servo rightServo;

const int downLeftSensor = 12;
const int downRightSensor = 13;

const int trigPinFrontSensor = 8;
const int echoPinFrontSensor = 9;

const int onOfButtonPin = 2;

void setup() {
  leftServo.attach (4);
  rightServo.attach (7);

  pinMode (downLeftSensor, INPUT);
  pinMode (downRightSensor, INPUT);

  pinMode (trigPinFrontSensor, OUTPUT);
  pinMode (echoPinFrontSensor, INPUT);

  pinMode(onOfButtonPin, INPUT);

  Serial.begin (9600); //setting serial output to 9600 port
}


// MODES------------------------------------------------------------------------
bool fightMode = false;
bool calibrationMode = false;

bool borderMode;
bool detectionMode = true;
bool attackMode;
bool defenseMode;


// GLOBAL CONSTANTS-------------------------------------------------------------
const double robotVelocity = 7.5;         // Constant velosity of the robot (In/sec)
const double secondsFor360Turn = 2.7;     // Constant amount of time it takes robot to turn 360 degrees in seconds


// DECLARING FUNCTIONS----------------------------------------------------------
void mainFunction();

void fightFunction();
void waitFunction();

void borderFunction();
void detectionFunction();
void attackFunction();

void basicMovementSec(int powerLeft, int powerRight, double moveSeconds);
void basicMovementForDegree(int powerLeft, int powerRight, double turnAngle);
void stopMovement();
void turnRightForSec(double moveSeconds);
void turnRightForDegree(double turnAngle);
void turnLeftForSec(double moveSeconds);
void turnLeftForDegree(double turnAngle);
void driveForwardForSec(double moveSeconds);
void driveBackwardForSec(double moveSeconds);

bool leftSensorDetectWhite();
bool rightSensorDetectWhite();
float frontSensorDistanceInches ();

void pressOnOffButton();
double angleToSeconds (double turnAngle);
void calibrationFunction();


// STANDARD LOOP----------------------------------------------------------------
void loop(void) {
  if (calibrationMode) {
    calibrationFunction();
  } else {
    mainFunction();
  }
}

// MAIN FUNCTION----------------------------------------------------------------
void mainFunction() {
  pressOnOffButton(); // Calls for function that controls On/Off Button

  if (fightMode == true) {
    Serial.println("fightFunction");
    fightFunction();
  } else {
    Serial.println("waitFunction");
    waitFunction();
  }
}

// MODES FUNCTIONS---------------------------------------------------------------
void fightFunction(){

  double frontSensorReadingInches = frontSensorDistanceInches();
  bool leftSensorOnWhite = leftSensorDetectWhite();
  bool rightSensorOnWhite = rightSensorDetectWhite();

  if (leftSensorOnWhite || rightSensorOnWhite){
    borderMode = true;
  }

  if (leftSensorOnWhite || rightSensorOnWhite && frontSensorReadingInches){
    defenseMode = true;
  }

  if (borderMode){
    borderFunction(leftSensorOnWhite, rightSensorOnWhite);
  }
  if (detectionMode){
    detectionFunction(frontSensorReadingInches);
  }
  if (attackMode){
    attackFunction(frontSensorReadingInches);
  }
  if (defenseMode){
    defenseFunction();
  }
}

void waitFunction(){
  stopMovement(); //do nothing
   //Serial.println(digitalRead (downLeftSensor));
}

void borderFunction(bool leftSensorOnWhite, bool rightSensorOnWhite){
  if (leftSensorOnWhite){
    driveBackwardForSec(0.5);
    turnRightForDegree(90);
    driveForwardForSec(1);

    detectionMode = true;
    borderMode = false;
  } else if (rightSensorOnWhite){
    driveBackwardForSec(0.5);
    turnLeftForDegree(90);
    driveForwardForSec(1);

    detectionMode = true;
    borderMode = false;
  }
}
void detectionFunction(double frontSensorReadingInches){
  Serial.println("detectionFunction");
  if (random(1,0) == 1){
    if (frontSensorReadingInches >= 17){
      turnLeftForSec(0);
    } else {
      attackMode = true;
      detectionMode = false;
    }
  } else {
    if (frontSensorReadingInches >= 17){
      turnRightForSec(0);
    } else {
      attackMode = true;
      detectionMode = false;
    }
  }
}
void attackFunction(double frontSensorReadingInches){
  Serial.println("attackFunction");
  if (frontSensorReadingInches <= 17){
    driveForwardForSec(0);
  } else {
    detectionMode = true;
    attackMode = false;
  }
}
void defenseFunction(){

}

// MOVEMENT FUNCTIONS-----------------------------------------------------------
void basicMovementForSec(int powerLeft, int powerRight, double moveSeconds) {
  int left, right;

  if (powerLeft == 1) {
    left = 180;
  } else if (powerLeft == -1) {
    left = 0;
  } else {
    left = 90;
  }

  if (powerRight == 1) {
    right = 0;
  } else if (powerRight == -1) {
    right = 180;
  } else {
    right = 90;
  }

  if (moveSeconds == 0) {
    leftServo.write(left);
    rightServo.write(right);
  } else {
    leftServo.write(left);
    rightServo.write(right);

    Serial.println("moveSeconds = ");
    Serial.println(moveSeconds * 1000);

    delay(moveSeconds * 1000);
  }
}
void basicMovementForDegree(int powerLeft, int powerRight, double turnAngle) {
  int left, right;
  double moveSeconds;

  if (powerLeft == 1) {
    left = 180;
  } else if (powerLeft == -1) {
    left = 0;
  } else {
    left = 90;
  }

  if (powerRight == 1) {
    right = 0;
  } else if (powerRight == -1) {
    right = 180;
  } else {
    right = 90;
  }

  moveSeconds = angleToSeconds(turnAngle);

  if (moveSeconds == 0) {
    leftServo.write(left);
    rightServo.write(right);
  } else {
    leftServo.write(left);
    rightServo.write(right);

    Serial.println("moveSeconds = ");
    Serial.println(moveSeconds * 1000);

    delay(moveSeconds * 1000);
  }
}

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
// Function for the left ground sensor (outputs 1 for white and 0 for black)
bool leftSensorDetectWhite() {
  int leftSensorReading = digitalRead (downLeftSensor);
  if (leftSensorReading == 1){
    Serial.println ("Left Detects White");
    return 1;
  } else {
    Serial.println ("Left Detects Black");
    return 0;
  }
}

// Function for the left ground sensor (outputs 1 for white and 0 for black)
bool rightSensorDetectWhite() {
  int rightSensorReading = digitalRead (downRightSensor);
  if (rightSensorReading == 1){
    Serial.println ("Right Detects White");
    return 1;
  } else {
    Serial.println ("Right Deytects Black");
    return 0;
  }
}

// Function for the front sensor (outputs distance to the object in front of the robot)
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
// Function check if On/Off Button pressed and changes Fight Mode status acordenly
void pressOnOffButton() {
  //read the state of the "On/Off" button value:
  int onOffButtonState = digitalRead(onOfButtonPin);

  //in case "On/Off" button is pressed change to and from the "fight mode"
  if (onOffButtonState == HIGH) {
    if (fightMode == false) {
      fightMode = true;
      Serial.println("Fight Mode --> ");
      Serial.print(fightMode);
    } else {
      fightMode = false;
      Serial.println("Fight Mode --> ");
      Serial.print(fightMode);
    }
    delay(200); // adds delay for user to remove the finger
  }
}

// Function trans forms turning angle into time (s) based on the measured rotation speed
double angleToSeconds (double turnAngle) {
  double turnTime;
  turnTime = (turnAngle * secondsFor360Turn) / (360);
  Serial.print ("turnTime --> ");
  Serial.print (turnTime);
  return turnTime;
}

// Calibration funciton (for ground sensors, servos, and front sensor)
void calibrationFunction() {

  stopMovement();

  double frontSensorReading = frontSensorDistanceInches();
  Serial.println ("Front Distance (in) --> ");
  Serial.print (frontSensorReading);
  delay (100);

  int leftSensorReading = digitalRead (downLeftSensor);
  int rightSensorReading = digitalRead (downRightSensor);

  if (leftSensorReading || rightSensorReading){
    Serial.println ("");
    Serial.println ("-------------------------------------------------");
    Serial.println ("");

    if (leftSensorReading == 1){
      Serial.println ("Left Sensor is Not Calibrated");
    } else {
      Serial.println ("Left Sensor is Calibrated");
    }

    if (rightSensorReading == 1){
      Serial.println ("Right Sensor is Not Calibrated");
    } else {
      Serial.println ("Right Sensor is Calibrated");
    }

    delay (100);
  }
  else {
    Serial.println ("Both Sensors are Calibrated");
  }
}


//END---------------------------------------------------------------------------
