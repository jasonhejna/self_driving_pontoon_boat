// Two states: on/off button controls autonomous mode on/off using interrupt.
// RETURN: signal output int values of throttle, and steering position

// autonomous mode: pwm decode into usable stepper motor values. 
// rising, falling functions do interrupt timing on incoming pwm signal.

// manual control mode: values are read from analog controlls.
// Potentiometer, or rotary encoder for steeting
// Slide pot for throttle

//stepper motor
#include <Stepper.h>
#define STEPS 200                       // number of steps in steering stepper motor
Stepper steering(STEPS, 8, 9, 10, 11);
int steeringPos;
int stepper_midpoint = 0;
int current_pos = 0;

// pwm decode
unsigned int STEERING_PIN = 2;
unsigned int THROTTLE_PIN = 5;
int pwm_steering;
int pwm_throttle;

// servo
#include <Servo.h>
Servo throttle;
int throttlePos;


void setup(){

  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  
  Serial.begin(9600);

  steering.setSpeed(60);
  
  throttle.attach(13);
}



void loop(){

  pwm_steering = pulseIn(STEERING_PIN, HIGH); // PWM decoding raw input 
  pwm_throttle = pulseIn(THROTTLE_PIN, HIGH);

  steeringPos = map(pwm_steering, 1000, 1900, -255, 255);
  throttlePos = map(pwm_throttle, 1000, 1900, 0, 180);

  Serial.println("T");
  
  throttle.write(throttlePos);
  
  Serial.println(throttlePos);
  
  //if (pwm_value > current_pos){
  //  steering.step(1);
  //  current_pos++;
  //}
  //else if (pwm_value < current_pos){
  //  steering.step(-1);
  //  current_pos--;
  //}

}
