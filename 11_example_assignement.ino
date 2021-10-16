#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 400 // maximum distance to be measured (unit: mm)
#define  _DIST_ALPHA 0.3

#define _DUTY_MIN 550 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1475 // servo neutral position (90 degree)
#define _DUTY_MAX 2400 // servo full counterclockwise position (180 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema, alpha, dist_ema_backup; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  alpha = _DIST_ALPHA;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = alpha * dist_raw + (1 - alpha) * dist_ema;

// output the read value to the serial port
  Serial.print("Min:100,Low:180,raw:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());  
  Serial.println(",High:220,Max:400");

// adjust servo position according to the USS read value

  // add your code here!
  if(dist_ema<dist_min || dist_ema > dist_max){
    dist_ema = dist_ema_backup;
  }
  dist_ema_backup = dist_ema;
  if(dist_ema<=180) {
     myservo.writeMicroseconds(_DUTY_MIN);
     digitalWrite(PIN_LED, 1);
  }
  else if(dist_ema<=190){
    myservo.writeMicroseconds(_DUTY_MIN + 100);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=200){
    myservo.writeMicroseconds(_DUTY_MIN + 200);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema_backup<=210){
    myservo.writeMicroseconds(_DUTY_MIN + 310);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=220){
     myservo.writeMicroseconds(_DUTY_MIN + 410);
     digitalWrite(PIN_LED, 0);
  }
  else if (dist_ema<=230){
    myservo.writeMicroseconds(_DUTY_MIN + 510);
    digitalWrite(PIN_LED, 0);
  }
  else if (dist_ema<=240){
    myservo.writeMicroseconds(_DUTY_MIN + 620);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=250){
    myservo.writeMicroseconds(_DUTY_MIN + 720);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=260){
    myservo.writeMicroseconds(_DUTY_MIN + 820);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=270){
    myservo.writeMicroseconds(_DUTY_NEU);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=280){
    myservo.writeMicroseconds(_DUTY_NEU + 100);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=290){
    myservo.writeMicroseconds(_DUTY_NEU + 210);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=300){
    myservo.writeMicroseconds(_DUTY_NEU + 310);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=310){
    myservo.writeMicroseconds(_DUTY_NEU + 410);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=320){
    myservo.writeMicroseconds(_DUTY_NEU + 520);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=330){
    myservo.writeMicroseconds(_DUTY_NEU + 620);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=340){
    myservo.writeMicroseconds(_DUTY_NEU + 720);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=350){
    myservo.writeMicroseconds(_DUTY_NEU + 830);
    digitalWrite(PIN_LED, 0);
  }
  else if(dist_ema<=360){
    myservo.writeMicroseconds(_DUTY_MAX);
    digitalWrite(PIN_LED, 0);
  }
  else {
    myservo.writeMicroseconds(_DUTY_MAX);
    digitalWrite(PIN_LED, 1);
  }
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.

  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
