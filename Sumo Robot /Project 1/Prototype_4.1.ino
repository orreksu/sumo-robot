/* Title: Prototype 4
   Date: October 30, 2018
   Functions: super smart movement function, sensor functions, calibration function, calibrated the ground sensors and motors, front sensor
   Need to add: make front sensor work
*/

#include <Servo.h>
#include <time.h>





//DECLARING FUNCTIONS----------------------------------------------------------------------

void mainFunc();
void calibrationFunc();

void fightMode();
void fightMoveII();

void controlMove(int powerLeft, int powerRight, double moveSeconds);
double degreesToSeconds (double angleToTurn);
void randomMove();

void detectOpponent();
bool whiteDetected();

bool leftSensorWhite();
bool rightSensorWhite();
float frontSensorDistance();
//-----------------------------------------------------------------------------------------





//CONNECTING SERVOS, SENSORS & BUTTON-------------------------------------------------------

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
//-----------------------------------------------------------------------------------------




//GLOBAL VARIABLES-------------------------------------------------------------------------

bool fightModeStatus = false; //declaring bool varibale for the "Fight Mode" and setting it to "false"

bool turnFlag = true;
bool sawWhite;

const double robotVelocity = 7.5; //Constant velosity of the robot in inches per second
const double secondsFor360Turn = 2.7; //Constant amount of time it takes robot to turn 360 degrees in seconds
//-----------------------------------------------------------------------------------------




//STANDARD LOOP----------------------------------------------------------------------------

void loop(void) {
  //call the "mainFunction" (Main Function)
  mainFunc(); //comment for calibration

  //call the "calibFunction" (Calibration Function)
  //calibrationFunc();  //uncomment for calibration
}
//-----------------------------------------------------------------------------------------





//MAIN FUCTIONS--------------------------------------------------------------------------

void mainFunc() {
  //read the state of the "On/Off" button value:
  int onOffButtonState = digitalRead(onOfButtonPin);

  //in case "On/Off" button is pressed change to and from the "fight mode"
  if (onOffButtonState == HIGH) {
    if (fightModeStatus == false) {
      fightModeStatus = true;
      Serial.println("Fight mode --> ON");
    } else {
      fightModeStatus = false;
      Serial.println("Fight mode --> OFF");
    }
    delay(200);
  }

  //in case the main function is not in the "fight mode":
  if (fightModeStatus == false) {
    //the motors are turned off
    controlMove(0, 0, 0);
  }

  //in case the main function is in the "fight mode":
  else {
    //wait for 5 seconds for the start of the fight
    //delay (5000);

    /*Serial.println(turnFlag);
      if (turnFlag == true) {
      controlMove(1, 1, 1);
      turnFlag = false;
      }*/

    //randomMove();
    fightMoveII();

    //call the "fightMode" function (The Fight Mode)
    //fightMode();


    //controlMove(0, 0, 0); //stop
  }
}


void fightMode() {
  controlMove(1, 1, 0);
  //in case left or right down sensors detects white line
  if (leftSensorWhite()) {
    if (rightSensorWhite() == 1) {
      controlMove(1, -1, degreesToSeconds(150));
    }
  }
  else if (rightSensorWhite()) {
    if (leftSensorWhite()) {
      controlMove(-1, 1, degreesToSeconds(150));
    }
  }
  //in case no boundery detected
  else {
    detectOpponent();
  }
}

void fightMoveII() {
  controlMove(1, 1, 0);
  //in case left or right down sensors detects white line
  if (leftSensorWhite()) {
    sawWhite = true;
  }
  if (rightSensorWhite() && sawWhite) {
    controlMove(1, -1, degreesToSeconds(150));
    sawWhite = false;
  }



  else if (rightSensorWhite()) {
    if (leftSensorWhite()) {
      controlMove(-1, 1, degreesToSeconds(150));
    }
  }
  //in case no boundery detected
  else {
    detectOpponent();
  }
}

void detectOpponent() {
  controlMove(0, 0, 0);
}
//-----------------------------------------------------------------------------------------





//MOVEMENT FUCTIONS----------------------------------------------------------------------

//Basic controlled movement function for the robot (1 is forward, -1 is reverse, and 0 is stop; time is in seconds, while 0 for time is forever)
void controlMove(int powerLeft, int powerRight, double moveSeconds) {
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
  }

  else {
    leftServo.write(left);
    rightServo.write(right);

    Serial.println("moveSeconds = ");
    Serial.println(moveSeconds * 1000);

    delay(moveSeconds * 1000);
  }
}

double degreesToSeconds (double angleToTurn) {
  double timeToTurn;
  timeToTurn = (angleToTurn * secondsFor360Turn) / (360);
  return timeToTurn;
  Serial.print (timeToTurn);
}

void randomMove () {
  if (frontSensorDistance() <= 5) {

    if (turnFlag == true) {

      controlMove (1, -1, degreesToSeconds (45));
      controlMove (1, 1, 1);
      controlMove (-1, 1, degreesToSeconds (45));
      controlMove (1, 1, 1);

      turnFlag = false;
    }

  }
}
/*
  void Move_Mino_Mino (){
  if (FrontSensor<=5) {
   Move (-1,1,1);
   Move (1,-1,1);
  }
  }

  void TurnAround () {
  if (LeftSensor()==1) {
    Move (1,-1,8);
    Move (1,1,0);
  }

  if (RightSensor()==1) {
    Move (-1,1,8);
    Move (1,1,0);
  }
  }
*/
//-----------------------------------------------------------------------------------------





//SENSORS FUCTIONS-----------------------------------------------------------------------

//Function for the left ground sensor (outputs 1 for white and 0 for black)
bool leftSensorWhite() {
  int dleft = digitalRead (downLeftSensor);
  if (dleft == 1)
  {
    Serial.println ("Left Sees White");
    return 1;
  }
  else
  {
    Serial.println ("Left Sees Black");
    return 0;
  }
}

//Function for the left ground sensor (outputs 1 for white and 0 for black)
bool rightSensorWhite() {
  int dright = digitalRead (downRightSensor);
  if (dright == 1)
  {
    Serial.println ("Right Sees White");
    return 1;
  }
  else
  {
    Serial.println ("Right Sees Black");
    return 0;
  }
}

//Function for the front sensor (outputs distance to the object in front of the robot)
float frontSensorDistance () {
  float echoTime;
  float calculatedDistance;

  digitalWrite (trigPinFrontSensor, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPinFrontSensor, LOW);

  echoTime = pulseIn(echoPinFrontSensor, HIGH);

  calculatedDistance = echoTime / 148.0;

  return calculatedDistance;
}
//-----------------------------------------------------------------------------------------





//ADDITIONAL FUCTIONS--------------------------------------------------------------------

//Calibration funciton (for ground sensors, servos, and front sensor)
void calibrationFunc() {

  controlMove(0, 0, 0);

  float fronS = frontSensorDistance ();
  Serial.println ("Front Distance (in) --> ");
  Serial.print (fronS);

  int sleft = digitalRead (downLeftSensor);
  int sright = digitalRead (downRightSensor);
  if (sleft || sright)
  {
    Serial.println ("");
    Serial.println ("-------------------------------------------------");
    Serial.println ("");
    if (sleft == 1)
    {
      Serial.println ("Left Sensor is Not Calibrated");
      Serial.println (sleft);
    }
    if (sright == 1)
    {
      Serial.println ("Right Sensor is Not Calibrated");
      Serial.println (sright);
    }
    delay (100);
  }
  else {
    Serial.println ("Calibrated");
  }
}
//-----------------------------------------------------------------------------------------
