#include <Arduino.h>
#include <Servo.h>

#define BAUDRATE 9600

#define FB_MOTOR_CMD  6
#define LR_MOTOR_CMD  9
#define LIFTER_CMD    5 

// pulse widths bounds of the receiver's PWM output
#define PWM_HIGH_BOUND  2000
#define PWM_LOW_BOUND   990

// Motor 1
#define IN1 2
#define IN2 3

// Motor 2
#define IN3 4
#define IN4 8

// motor speed control
#define ENA 7
#define ENB 10
uint8_t motor_speed = 0;
int y = 0;
int x = 0;

// function definitions
void moveForward();
void moveBackward();
void moveRight();
void moveLeft();
void halt();
void updateDirection();

void setup()
{
  Serial.begin(BAUDRATE);
  
  // setting motor(s) to pin(s)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // setting speed control
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // setup RC pins
  pinMode(FB_MOTOR_CMD, INPUT);
  pinMode(LR_MOTOR_CMD, INPUT);
  pinMode(LIFTER_CMD, INPUT);
  
}

void loop()
{
  y = pulseIn(FB_MOTOR_CMD, HIGH);
  x = pulseIn(LR_MOTOR_CMD, HIGH);

  // map y and x values
  y = map(y, PWM_LOW_BOUND, PWM_HIGH_BOUND, -255, 255);
  x = map(x, PWM_LOW_BOUND, PWM_HIGH_BOUND, -255, 255);

  int motor_speed = sqrt(pow(y, 2) + pow(x, 2));

  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);

  // forward
  if(y > x && y > 0 && y > -x)
    moveLeft();
  else if(y < x && y < 0 && y < -x)
    moveRight();
  else if(y < -x && x < 0 && y > x)
    moveForward();
  else if(y > -x && x > 0 && y < x)
    moveBackward();
}

void updateDirection()
{
  y = pulseIn(FB_MOTOR_CMD, HIGH);
  x = pulseIn(LR_MOTOR_CMD, HIGH);

  // map y and x values
  y = map(y, PWM_LOW_BOUND, PWM_LOW_BOUND, -255, 255);
  x = map(x, PWM_LOW_BOUND, PWM_LOW_BOUND, -255, 255);

  Serial.println(y);

  // forward
  if(y > x && y > 0 && y > -x)
    Serial.println("Forward");
  else if(y < x && y < 0 && y < -x)
    Serial.println("Backward");
  else if(y < -x && x < 0 && y > x)
    Serial.println("Left");
  else if(y > -x && y > 0 && y > -x)
    Serial.println("Right");
}

void moveForward(){
  digitalWrite(IN1, HIGH); // go forward
  digitalWrite(IN2, LOW);  // don't go backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward(){
   digitalWrite(IN1, 0);
   digitalWrite(IN2, 1);  
   digitalWrite(IN3, 0);
   digitalWrite(IN4, 1);
}

void moveRight(){
   digitalWrite(IN1, 0); 
   digitalWrite(IN2, 1);
   digitalWrite(IN3, 1);
   digitalWrite(IN4, 0);
}

void moveLeft(){
   digitalWrite(IN1, 1);
   digitalWrite(IN2, 0); 
   digitalWrite(IN3, 0);
   digitalWrite(IN4, 1);
}

void halt(){
  digitalWrite(IN1, 0); // go forward
  digitalWrite(IN2, 0);  // don't go backward
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}
