/* Title: Prototype 2
 * Date: October 30, 2018
 * Functions: super smart movement function, sensor functions, calibration function, calibrated the ground sensors and motors, front sensor
 * Need to add: make front sensor work
 */

#include <Servo.h>
#include <time.h>





//DECLARING FUNCTIONS----------------------------------------------------------------------

void mainFunc();
void calibrationFunc();

void controlMove(int powerLeft, int powerRight, double moveSeconds);
void controlMoveDelay(int powerLeft, int powerRight, double moveSeconds);
double degreesToSeconds (double angleToTurn);

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
const double robotVelocity = 8; //Constant velosity of the robot in inches per second
const double secondsFor360Turn = 2.7; //Constant amount of time it takes robot to turn 360 degrees in seconds
//-----------------------------------------------------------------------------------------
bool flag=true;




//STANDARD LOOP----------------------------------------------------------------------------

void loop(void) {
  //call the "mainFunction" (Main Function)
  mainFunc(); //comment for calibration
  
  //call the "calibFunction" (Calibration Function)
  //calibrationFunc();  //uncomment for calibration
}
//-----------------------------------------------------------------------------------------





//MAIN FUCTIONS--------------------------------------------------------------------------

void mainFunc(){
  //read the state of the "On/Off" button value:
  int onOffButtonState = digitalRead(onOfButtonPin);

  //in case "On/Off" button is pressed change to and from the "fight mode"
  if (onOffButtonState == HIGH){
    if (fightModeStatus == false){fightModeStatus = true;} else {fightModeStatus = false;}
    delay(200);
  }

  //in case the main function is not in the "fight mode":
  if (fightModeStatus == false){
    //the motors are turned off
    controlMove(0,0,0);
  }

  //in case the main function is in the "fight mode":
  else if(flag){
    //wait for 5 seconds for the start of the fight
    //delay (5000);
    
    //call the "fightMode" function (The Fight Mode)
    //fightMode();

    controlMove(1,1,1);
    flag=false;
  }
  else
    controlMove(0,0,0);
}


void fightMode(){
  if (downLeftSensor == 1 || downRightSensor == 1){
  //in case left or right down sensors detects white line 
  } 
  
  else {
    detectOpponent();
  }
  
}



void detectOpponent(){
  
}
//-----------------------------------------------------------------------------------------





//MOVEMENT FUCTIONS----------------------------------------------------------------------

//Basic controlled movement function for the robot (1 is forward, -1 is reverse, and 0 is stop; time is in seconds, while 0 for time is forever)
void controlMoveDelay(int powerLeft, int powerRight, double moveSeconds){ 
  int left, right;

  if (powerLeft == 1){left = 180;} else if (powerLeft == -1){left = 0;} else {left = 90;}
  if (powerRight == 1){right = 0;} else if (powerRight == -1){right = 180;} else {right = 90;}

  if (moveSeconds == 0){
      leftServo.write(left);
      rightServo.write(right);
  } else {
      leftServo.write(left);
      rightServo.write(right);
      delay(moveSeconds);
  }
}

void controlMove(int powerLeft, int powerRight, double moveSeconds){ 
  int left, right;

  if (powerLeft == 1){left = 180;} else if (powerLeft == -1){left = 0;} else {left = 90;}
  if (powerRight == 1){right = 0;} else if (powerRight == -1){right = 180;} else {right = 90;}

  if (moveSeconds == 0){
      leftServo.write(left);
      rightServo.write(right);
  } else {
    time_t seconds = moveSeconds;
    time_t endwait = time(NULL) + seconds;
    if (time(NULL) < endwait){
      leftServo.write(left);
      rightServo.write(right);
    }
  }
}

double degreesToSeconds (double angleToTurn) {
  double timeToTurn; 
  timeToTurn = (angleToTurn * secondsFor360Turn)/(360);
  return timeToTurn;
  Serial.print (timeToTurn);
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
bool leftSensorWhite(){
  int dleft = digitalRead (leftSensorWhite);
  if (dleft==1)
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
  int dright = digitalRead (rightSensorWhite);
  if (dright==1)
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
  
  controlMove(0,0,0);

  float fronS = frontSensorDistance ();
  Serial.println ("Front Distance (in) --> ");
  Serial.print (fronS);
  
  int sleft = digitalRead (leftSensorWhite);
  int sright = digitalRead (rightSensorWhite);
  if (sleft || sright)
  {
    Serial.println ("");
    Serial.println ("-------------------------------------------------");
    Serial.println ("");
    if (sleft==1)
    {
      Serial.println ("Left Sensor is Not Calibrated");
      Serial.println (sleft);
    }
    if (sright==1)
    {
      Serial.println ("Right Sensor is Not Calibrated");
      Serial.println (sright);
    }
    delay (100);
  }
  else{Serial.println ("Calibrated");}
}
//-----------------------------------------------------------------------------------------
