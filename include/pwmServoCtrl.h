// <<<<<<<<<<=========PWM Servo===========>>>>>>>>>>
#ifndef __PWMSERVOCTRL_H__
#define __PWMSERVOCTRL_H__

#include <ESP32_Servo.h>
Servo pwmServo;

#define PSERVO_PIN 4

int pwmServoChannel = 7;
int pwmServoInitPos = 90;


void pwmServoInit(){
  pwmServo.attach(PSERVO_PIN);
}


void GRAB_ServoCtrl(int posInput){
  pwmServo.write(posInput);
}

#endif
