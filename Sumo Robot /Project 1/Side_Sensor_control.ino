/*Title: Side Sensor Control
 * Purpose: Controlling our side down sensors 
 * Function: This program implements a very basic algorithim for detecting an edge and steering away
 * Date: October 26, 2018
 */

#include <Servo.h>

Servo servo_R;
Servo_L;
const int leftSensor =12;
const int rightSensor =13;

int lsensor=0;
intrsensorState = 0;

void setup() 
{
  pinMode (leftSensor, INPUT);
  pinMode (rightSensor, INPUT);
  servo_R.attach (7);
  servo_L.attach (4);
  Serial.begin (9600);
  

}

void loop(void) 
{
  rsensorState =digitalRead (rightSensor);
  lsensorState =digitalRead (leftSensor);

  if (rsensorState==LOW)
  {
    servo_R.write (180);
    servo_L.write (0);
    delay (500);
    servo_L.write(0);
    servo_R.write (0);
    delay (250);
    Serial.printIn("Right Sensor");
    
  }
  else if (lsensorState ==LOW)
  {
    servo_R.write(180);
    servo_L.write(0);
    delay (500);
    servo_R.write (180);
    servo_L.write (180);
    delay (250);
    Serial.printIn ("Left Sensor")
  }
else
servoR.write (0);
servo_L.write (180);
}
