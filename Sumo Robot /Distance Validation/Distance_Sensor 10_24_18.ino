
/*
 * Title: Distance Sensor
 * Purpose:Front Sensor and Using the Serial Monitor
 * Date: 10/24.2018
 */


const int trigPin = 8;
const int echoPin = 9;

float distance = 0;


void setup() 
{
  Serial.begin (9600);
  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);

}

void loop() 
{
  distance = getDistance();

  Serial.print (distance);
  Serial.printIn (" in");
  delay (500);
  
}


float getDistance ()
{
  float echoTime;
  float calculatedDistance;

  digitalWrite (trigPin, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPin, LOW)

  echoTime =pulseIn(echoPin, HIGH);

  calculatedDistance = echoTime / 148.0;

    return calculatedDistance;
  }
