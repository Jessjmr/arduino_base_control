/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <Adafruit_MotorShield.h>

float t1,u;
long g_ant,grados;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  Serial.println("Basic Encoder Test:");
  t1=millis(); 
  g_ant=0.162162162*myEnc.read();
  if (!AFMS.begin(1600,&Wire1)) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");  
}

long oldPosition  = -999;

void loop() {
  float t2,d_finitas,rpm;
  long grados_actuales;
  
  //if (newPosition != oldPosition) {
  //  oldPosition = newPosition;
  //  Serial.println(newPosition);
  //}
  //Serial.println(t1);
  grados_actuales = 0.162162162*myEnc.read();
  t2=millis();
  d_finitas=(grados_actuales-g_ant)/((t2-t1)/1000);
  rpm=d_finitas/6;
  Serial.println(rpm);
  u=(0.5)*(60-rpm);
  if (u<0){
  myMotor->run(BACKWARD);}
  else  {
  myMotor->run(FORWARD);}
  
  myMotor->setSpeed(abs(u));
  t1=t2;
  g_ant=grados_actuales; 
}
