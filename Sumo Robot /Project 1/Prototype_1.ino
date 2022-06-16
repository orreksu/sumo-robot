/*Title: Prototype 1
 * Date: October 26, 2018
 * Functions: super smart movement function, sensor functions, calibration function, calibrated the ground sensors and motors
 * Need to add: start button, front sensor, check why battery pack fails everything, choose strategy
 */

#include <Servo.h>
#include <time.h>

//Functions
void Calibration();
void OldMainAction();

bool LeftSensor();
bool RightSensor();

void Move(int powerLeft, int powerRight, double moveSeconds);

void StayInCircle();


//Constants and connections
Servo servo_R;
Servo servo_L;
const int leftSensor = 12;
const int rightSensor = 13;

void setup() {
  pinMode (leftSensor, INPUT);
  pinMode (rightSensor, INPUT);
  servo_R.attach (7);
  servo_L.attach (4);
  Serial.begin (9600);
}

void loop(void) 
{
  StayInCircle();

  //Calibration();
  //OldMainAction();
}


void StayInCircle ()
{
  if (LeftSensor()==0 && RightSensor()==0){  //if both sees black
    Move(1,1,0);
  }
  else{
    Move (-1,1,1);
  }
}


//-------------Movement function for the robot-------------------------
void Move(int powerLeft, int powerRight, double moveSeconds){ //1 is forward, -1 is reverse, and 0 is stop && time is in seconds and 0 for time is forever
  int left, right;

  if (powerLeft == 1){left = 180;} else if (powerLeft == -1){left = 0;} else {left = 90;}
  if (powerRight == 1){right = 0;} else if (powerRight == -1){right = 180;} else {right = 90;}

  if (moveSeconds == 0){
    servo_L.write(left);
    servo_R.write(right);
  } else {
    time_t seconds = moveSeconds;
    time_t endwait = time(NULL) + seconds;
    if (time(NULL) < endwait){
    servo_L.write(left);
    servo_R.write(right);  
    }
  }
}
  


//----------Functions for ground sensors----------------------------------
bool LeftSensor(){  //Outputs 1 for white and 0 for black in the Left sensor
  int dleft = digitalRead (leftSensor);
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

bool RightSensor()  //Outputs 1 for white and 0 for black in the Right sensor
{
  int dright = digitalRead (rightSensor);
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




//--------Calibration funciton for ground sensors---------------------
void Calibration()  // 1 - white, 0 - black
{
  int sleft = digitalRead (leftSensor);
  int sright = digitalRead (rightSensor);
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




//------------Old main funciton from prof Freeman example--------------
void OldMainAction(){
  int rsensorState = digitalRead (rightSensor);
  int lsensorState = digitalRead (leftSensor);
  if (rsensorState==LOW){
    servo_R.write (180);
    servo_L.write (0);
    delay (500);
    servo_L.write(0);
    servo_R.write (0);
    delay (250);
    Serial.println("Right Sensor");
  } else if (lsensorState ==LOW){
    servo_R.write(180);
    servo_L.write(0);
    delay (500);
    servo_R.write (180);
    servo_L.write (180);
    delay (250);
    Serial.println ("Left Sensor");
  } else {
    servo_R.write (90);
    servo_L.write (90);
    delay (50);
  }
}
