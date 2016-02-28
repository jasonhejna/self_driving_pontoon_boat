// Two states: on/off button controls autonomous mode on/off using interrupt.
// RETURN: signal output int values of throttle, and steering position

// autonomous mode: pwm decode into usable stepper motor values. 
// rising, falling functions do interrupt timing on incoming pwm signal.

// manual control mode: values are read from analog controlls.
// Potentiometer, or rotary encoder for steeting
// Slide pot for throttle

//TODO: turn off motor (heat)


// LED
int auto_ledPin = 29;
int manu_ledPin = 31;

// toggle switches
int auto_mode;
int pause_mode;
unsigned int auto_mode_pin = 6;
unsigned int pause_mode_pin = 7;

bool first_loop = false;

// auto mode state
bool auto_mode_state = false;

//manual control potentiometers
unsigned int manual_steering_pin = 2;
int manual_steering_val;
unsigned int manual_throttle_pin = 2;
int manual_throttle_val;

//stepper motor
#include <Stepper.h>
#define STEPS 200                       // number of steps in steering stepper motor
Stepper steering(STEPS, 8, 9, 10, 11);
int steeringPos;
int stepper_midpoint = 0;
int current_steering_pos = 0;

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
  
  //Serial.begin(9600);

  steering.setSpeed(60);
  
  throttle.attach(13); // servo on in 13

  auto_mode = digitalRead(auto_mode_pin);

  pinMode(auto_ledPin, OUTPUT);
  pinMode(manu_ledPin, OUTPUT);
}



void loop(){

  pause_mode = digitalRead(pause_mode_pin);

  // Pause Mode switch is engaged 1:unpressed 0:depressed (engaged)
  if (pause_mode == 0){
    
    manual_throttle();

    //manual_steering(); //TODO: set pin to new potentiometer and uncomment

    if (auto_mode_state == true){
      auto_mode_state = false;
      digitalWrite(auto_ledPin, LOW);
      digitalWrite(manu_ledPin, HIGH);
    }
  }
  // Pause Mode switch is disengaged 1:unpressed 0:depressed (engaged)
  else{

    // check for start button press, it's a momentary button
    if (auto_mode_state == false){

      auto_mode = digitalRead(auto_mode_pin);

      if (auto_mode == 1){
        // boolean we set on button press to indicate auto mode on, until pause button press
        auto_mode_state = true;
        digitalWrite(auto_ledPin, HIGH);
        digitalWrite(manu_ledPin, LOW);
      }
      
    }

    // auto_mode button pressed, and so far no interrupt from pause button
    if (auto_mode_state == true){
      
      auto_throttle();

      auto_steering();
      
    }
    // pause button unpressed, but auto mode has not been engaged yet
    else{
      
      manual_throttle();

      //manual_steering(); //TODO set pin to new potentiometer and uncomment
      
      digitalWrite(auto_ledPin, LOW);
      digitalWrite(manu_ledPin, HIGH);
    }

  }

  

}

void manual_throttle(){
  // read PWM value from Pixhawk,
  // map it to motor value,
  // move motor to new position
  
  manual_throttle_val = analogRead(manual_throttle_pin);

  throttlePos = map(manual_throttle_val, 0, 1024, 0, 180);
  
  throttle.write(throttlePos);
  
}

void manual_steering(){
  
  manual_steering_val = analogRead(manual_steering_pin);

  steeringPos = map(manual_steering_val, 0, 1024, -255, 255);
  
  if (steeringPos > current_steering_pos){
    steering.step(1);
    current_steering_pos++;
  }
  else if (steeringPos < current_steering_pos){
    steering.step(-1);
    current_steering_pos--;
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
  
  if (steeringPos > current_steering_pos){
    steering.step(1);
    current_steering_pos++;
  }
  else if (steeringPos < current_steering_pos){
    steering.step(-1);
    current_steering_pos--;
  }

}
