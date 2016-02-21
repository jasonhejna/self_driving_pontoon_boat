// Two states: on/off button controls autonomous mode on/off using interrupt.
// RETURN: signal output int values of throttle, and steering position

// autonomous mode: pwm decode into usable stepper motor values. 
// rising, falling functions do interrupt timing on incoming pwm signal.

// manual control mode: values are read from analog controlls.
// Potentiometer, or rotary encoder for steeting
// Slide pot for throttle

//TODO: turn off motor (heat)

// toggle switches
int auto_mode;
unsigned int auto_mode_pin = 7;

//manual control potentiometer
unsigned int manual_steering_pin = 2;
int manual_steering_val = 0;

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

  auto_mode = digitalRead(auto_mode_pin);
}



void loop(){
  
  auto_mode = digitalRead(auto_mode_pin);
  
  if (auto_mode > 0){
    
    auto_throttle();

    auto_steering();
      
  }
  else{

    //manual_throttle();

    manual_steering();
    
  }



}

void manual_throttle(){
  
}

void manual_steering(){
  
  //Serial.println("m");
  
  manual_steering_val = analogRead(manual_steering_pin);

  //Serial.println(manual_steering_val);

  steeringPos = map(manual_steering_val, 0, 1024, -255, 255);
  
  if (steeringPos > current_pos){
    steering.step(1);
    current_pos++;
  }
  else if (steeringPos < current_pos){
    steering.step(-1);
    current_pos--;
  }
  
}

void auto_throttle(){
  
  // autopilot throttle
 
  pwm_throttle = pulseIn(THROTTLE_PIN, HIGH);

  throttlePos = map(pwm_throttle, 1000, 1900, 0, 180);
  
  throttle.write(throttlePos);

}

void auto_steering(){

  // autopilot steering

  pwm_steering = pulseIn(STEERING_PIN, HIGH); // PWM decoding raw input

  steeringPos = map(pwm_steering, 1000, 1900, -255, 255);
  
  if (steeringPos > current_pos){
    steering.step(1);
    current_pos++;
  }
  else if (steeringPos < current_pos){
    steering.step(-1);
    current_pos--;
  }

}
