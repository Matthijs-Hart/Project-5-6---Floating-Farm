#ifndef _motorHandler_H    // Put these two lines at the top of your file.
#define _motorHandler_H    // (Use a suitable name, usually based on the file name.)
#include <Arduino.h>

bool motor1state = false;
#define motor1pin 12
#define motor1Forward 2
#define motor1Backward 10

bool motor2state = false;
#define motor2pin 13
#define motor2Forward 3
#define motor2Backward 11

bool motor3state = false;
#define motor3pin 14
#define motor3Forward 4
#define motor3Backward 12

bool motor4state = false;
#define motor4pin 27
#define motor4Forward 5
#define motor4Backward 13

void setupMotors(){
  pinMode(motor1pin, OUTPUT);
  pinMode(motor1Forward, OUTPUT);
  pinMode(motor2pin, OUTPUT);
  pinMode(motor2Forward, OUTPUT);
  pinMode(motor3pin, OUTPUT);
  pinMode(motor3Forward, OUTPUT);
  pinMode(motor4pin, OUTPUT);
  pinMode(motor4Forward, OUTPUT);

  digitalWrite(motor1Forward, HIGH);
  digitalWrite(motor2Forward, HIGH);
  digitalWrite(motor3Forward, HIGH);
  digitalWrite(motor4Forward, HIGH);

  digitalWrite(motor1Backward, LOW);
  digitalWrite(motor2Backward, LOW);
  digitalWrite(motor3Backward, LOW);
  digitalWrite(motor4Backward, LOW);
}

void motor1(){
  digitalWrite(motor1pin, !motor1state);
}

void motor2(){
  digitalWrite(motor2pin, !motor2state);
}

void motor3(){
  digitalWrite(motor3pin, !motor3state);
}   

void motor4(){
  digitalWrite(motor4pin, !motor4state);
}

#endif // _motorHandler_H    // Put this line at the end of your file