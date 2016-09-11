
#include <Servo.h>
#include <stdint.h>

#define RR_B 0
#define RR_A 1
#define RL_B 2
#define RL_A 3
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

// angle as a global
Servo steering_servo;
Servo throttle_servo;
int throtNeutral = 1500;
int throtMax = 1580;
int steerNeutral = 1500;
int steerMax = 2000;
int steerAngle;
bool autonomous = false;
bool methodCalled = false;
unsigned int steerStart = 2000;
unsigned int throttleStart = 4000;
unsigned int throttleSlow = 5000;
unsigned int throttleEnd = 7000;

void setup() {
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
  Serial.begin(9600);

  //Attach and initialize servos
  steering_servo.attach(STEER_OUT);
  throttle_servo.attach(THROT_OUT);
  steering_servo.writeMicroseconds(1500);

  steerAngle = 1000;
}


// Call calibrate angle once with a new steering angle each time estop is pushed in.  Start at a hard left, end at a hard right.
// Reset to hard left when cycle is done.
void loop() {

  if (autonomous == true) {
    calibrateAngle(steerAngle);
    if (steerAngle < steerMax) {
      steerAngle += 100;
    }
    else {
      steerAngle = 1000;  // reset to the beginning (hard left) if we went through the cycle
    }
    digitalWrite(AUTONOMOUS_OUT, LOW);
    autonomous = false;
    methodCalled = true;
  }

  if (digitalRead(ESTOP_IN) == HIGH) {
    methodCalled = false;
  }

  if (digitalRead(ESTOP_IN) == LOW && methodCalled == false) {
    autonomous = true;
  }
}

// Set steering angle, briefly ramp up the throttle, then stop applying throttle and wait for the vehicle to stop.
void calibrateAngle(int angle) {
      digitalWrite(AUTONOMOUS_OUT, HIGH);
      unsigned long startTime = millis();
      
      Serial.print("angle: ");
      Serial.print(angle);
      Serial.println();
      
      int steer = steerNeutral;
      int throt = throtNeutral;
      while (true) {
        unsigned long elapsedTime = millis() - startTime;
        
        // wait a couple seconds, then set steering angle
        if (elapsedTime > steerStart && elapsedTime < throttleStart) {
          steer = angle;
        }

        // set throttle to a constant value
        if (elapsedTime >= throttleStart && elapsedTime < throttleSlow) {
          throt = throtMax;
        }

        // throttle off, wait a couple seconds to stop
        if (elapsedTime >= throttleSlow && elapsedTime < throttleEnd) {
          throt = throtNeutral;
        }
        
        if (elapsedTime >= throttleEnd) {
          return;
        }
        
        steering_servo.writeMicroseconds(steer);
        throttle_servo.writeMicroseconds(throt);
      }
}

