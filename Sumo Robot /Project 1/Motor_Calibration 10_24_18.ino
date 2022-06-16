
/* Title: MotorCalibration.cpp
 * Purpose: Calibration of motors on Machine Science Sumos with Redboard
 * Methods :Sends robot forward at full speed for 5 seconds then has them stop
 * to calibrate until wheels no longer move.
 * Date 10/24/2018
 */


#include <Servo.h> 
Servo servo_R;
Servo servo_L;



void setup() 
{
  servo_R.attach (7);
  servo_L.attach (4);
 
}

void loop(void) 
{
  servo_R.write(0);
  servo_L.write (180);
  delay (5000);
  servo_R.write (90);
  servo_L.write (90);
  delay (5000);
}
