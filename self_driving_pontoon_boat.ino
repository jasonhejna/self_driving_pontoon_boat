// Two states: on/off button controls autonomous mode on/off using interrupt.
// RETURN: signal output int values of throttle, and steering position

// autonomous mode: pwm decode into usable stepper motor values. 
// rising, falling functions do interrupt timing on incoming pwm signal.

// manual control mode: values are read from analog controlls.
// Potentiometer, or rotary encoder for steeting
// Slide pot for throttle

//stepper motor
#include <Stepper.h>
#define STEPS 200     # number of steps in steering stepper motor
Stepper steering(STEPS, 8, 9, 10, 11);
int stepper_midpoint = 0;
int min_bound = -300;
int max_bound = 300;
int current_pos = 0;

// pwm decode
byte PWM_PIN = 2;
int pwm_value;
int motor_speed = 0;

// servo
#include <Servo.h>
Servo throttle;
int servoPos;


void setup(){
  //pinMode(2,OUTPUT);
  pinMode(PWM_PIN, INPUT);
  
  Serial.begin(9600);

  steering.setSpeed(60);
  
  throttle.attach(13);
}



void loop(){

  pwm_value = pulseIn(PWM_PIN, HIGH); // raw input

  steeringPos = map(pwm_value, 1000, 1900, -255, 255);
  throttlePos = map(pwm_value, 1000, 1900, 0, 180);
  
  throttle.write(throttlePos);
  
  Serial.println(pwm_value);
  
  if (pwm_value > current_pos){
    steering.step(1);
    current_pos++;
  }
  else if (pwm_value < current_pos){
    steering.step(-1);
    current_pos--;
  }

}
