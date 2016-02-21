// Two states: on/off button controls autonomous mode on/off using interrupt.
// RETURN: signal output int values of throttle, and steering position

// autonomous mode: pwm decode into usable stepper motor values. 
// rising, falling functions do interrupt timing on incoming pwm signal.

// manual control mode: values are read from analog controlls.
// Potentiometer, or rotary encoder for steeting
// Slide pot for throttle

//stepper motor
#include <Stepper.h>
#define STEPS 200
Stepper stepper(STEPS, 8, 9, 10, 11);
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
throttle.attach(13);


void setup(){
  //pinMode(2,OUTPUT);
  pinMode(PWM_PIN, INPUT);
  
  Serial.begin(9600);

  stepper.setSpeed(60);
}



void loop(){

  //int granularity = 25;
  //int maxpt = 1850;
  //int minpt = 1050;
  //int midpt = (maxpt + minpt) / 2;
  //pwm_value = pulseIn(PWM_PIN, HIGH); // raw input
  //pwm_value = max(min(pwm_value, maxpt), minpt);
  //pwm_value = static_cast<double>(pwm_value - midpt)/(maxpt-midpt) * granularity; // reset zero and scale
  //pwm_value *= static_cast<double>(255) / granularity; // scale

  servoPos = map(val, 1000, 1900, 0, 180);

  Serial.println(pwm_value);
  
  if (pwm_value > current_pos){
    stepper.step(1);
    current_pos++;
  }
  else if (pwm_value < current_pos){
    stepper.step(-1);
    current_pos--;
  }

}
