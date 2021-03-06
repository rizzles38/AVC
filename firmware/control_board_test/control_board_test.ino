
#include <Servo.h> 

#define RR_A 0
#define RR_B 1
#define RL_A 2
#define RL_B 3
#define FR_A 10
#define FR_B 11
#define FL_A 8
#define FL_B 9
#define ORG_LED 12
#define RED_LED 13
#define ESTOP_IN 7
#define AUTONOMOUS_OUT 4
#define STEER_OUT 5
#define THROT_OUT 6

Servo steering_servo;
Servo throttle_servo;
long elapsedTime = 0;
int throt = 1500;
int throtStart = 1500;
int throtMax = 1600;
int steer = 1500;
int steerStart = 1500;
int steerMax = 1825;
double steerProportion = 0;
double throtProportion = 0;
int steerStep = 1;
int throtStep = 1;

void setup() {
  //Set up inputs and outputs
  pinMode(RR_A, INPUT);
  pinMode(RR_B, INPUT);
  pinMode(RL_A, INPUT);
  pinMode(RL_B, INPUT);
  pinMode(FR_A, INPUT);
  pinMode(FR_B, INPUT);
  pinMode(FL_A, INPUT);
  pinMode(FL_B, INPUT);
  pinMode(ORG_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(ESTOP_IN, INPUT);
  pinMode(AUTONOMOUS_OUT, OUTPUT);
  pinMode(STEER_OUT, OUTPUT);
  pinMode(THROT_OUT, OUTPUT);

  //Attach and initialize servos
  steering_servo.attach(STEER_OUT);
  throttle_servo.attach(THROT_OUT);
  steering_servo.writeMicroseconds(1500);
  //throttle_servo.writeMicroseconds(1500);
}

void loop() {

// use digitalRead for reading autonomous value switch
  long elapsedTime = millis();

  if(elapsedTime > 3000 && elapsedTime < 5000) { // time between 3 seconds and 6 seconds - gradually ramp up throttle
     if (throt < throtMax) {
        throtProportion = (double)(elapsedTime-3000)/2000;
        throt = throtStart + (int) (throtProportion * (throtMax - throtStart));
     }
     digitalWrite(ORG_LED, HIGH);
     digitalWrite(RED_LED, LOW);
  }
  
  if(elapsedTime >= 5000&& elapsedTime < 6000) { // time between 6 seconds and 10 seconds - keep throttle constant, turn right
     if (steer < steerMax) {
      steerProportion = (double)(elapsedTime - 5000)/1000;
      steer = steerStart + (int) (steerProportion * (steerMax - steerStart));
     }
     digitalWrite(ORG_LED, LOW);
     digitalWrite(RED_LED, HIGH);
  }
  
  if(elapsedTime >= 6000) { // time more than 10 seconds - stop
     if (throt > throtStart) {
      throtProportion = (double)(elapsedTime - 6000)/1000;
      throt = throtMax - (int) (throtProportion * (throtMax - throtStart));
     }
     if (steer > steerStart) {
      steerProportion = (double)(elapsedTime - 6000)/1000;
      steer = steerMax - (int) (steerProportion * (steerMax - steerStart));
     }
     digitalWrite(ORG_LED, HIGH);
     digitalWrite(RED_LED, HIGH);
  }

  if (digitalRead(ESTOP_IN) == LOW) { // estop is low - button is on
    digitalWrite(AUTONOMOUS_OUT, HIGH);
    steering_servo.writeMicroseconds(steer);
    throttle_servo.writeMicroseconds(throt);
  }

}
