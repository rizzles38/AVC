
#include <Servo.h>
#include <stdint.h>
#include <rover12_comm.h>

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

volatile int32_t countRearLeft = 0;
volatile int32_t countRearRight = 0;

void setup() {
  pinMode(RR_A, INPUT);
  pinMode(RR_B, INPUT);
  pinMode(RL_A, INPUT);
  pinMode(RL_B, INPUT);
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(RL_A),handleRLA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_B),handleRLB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_A),handleRRA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_B),handleRRB,CHANGE);

}

void loop() {
  // print out values
  Serial.print("Rear left: ");
  Serial.print(countRearLeft); 
  Serial.print(" Rear right: ");
  Serial.print(countRearRight);
  Serial.println();

  // Create a message
  rover12_comm::WheelEncMsg wheel_enc_msg;
  
  // Populate each wheel encoder count
  wheel_enc_msg.data.count_rear_left = countRearLeft;
  wheel_enc_msg.data.count_rear_right = countRearRight;
  
  // Encode and write to Serial
  wheel_enc_msg.encode();
  Serial.write(reinterpret_cast<uint8_t*>(&wheel_enc_msg), sizeof(wheel_enc_msg));
}

void handleRLA() {
  if (digitalRead(RL_A) == HIGH) {  // A changed from low to high
    if (digitalRead(RL_B) == LOW) { // B is low
      // CCW
      countRearLeft--;
    }
    else { // B is high
      // CW
      countRearLeft++;
    }
  }
  else { // A changed from high to low
    if(digitalRead(RL_B) == LOW) { // B is low
      // CW
      countRearLeft++;
    }
    else { // B is high
      // CCW
      countRearLeft--;
    }
  }
}

void handleRLB() {
  if (digitalRead(RL_B) == HIGH) {  // B changed from low to high
    if (digitalRead(RL_A) == LOW) { // A is low
      //CW
      countRearLeft++;
    }
    else { // A is high
      //CCW
      countRearLeft--;
    }
  }
  else { // B changed from high to low
    if(digitalRead(RL_A) == LOW) { // A is low
      //CCW
      countRearLeft--;
    }
    else { // A is high
      //CW
      countRearLeft++;
    }
  }
}

void handleRRA() {
  if (digitalRead(RR_A) == HIGH) {  // A changed from low to high
    if (digitalRead(RR_B) == LOW) { // B is low
      // CCW
      countRearRight--;
    }
    else { // B is high
      // CW
      countRearRight++;
    }
  }
  else { // A changed from high to low
    if(digitalRead(RR_B) == LOW) { // B is low
      // CW
      countRearRight++;
    }
    else { // B is high
      // CCW
      countRearRight--;
    }
  }
}
void handleRRB() {
  if (digitalRead(RR_B) == HIGH) {  // B changed from low to high
    if (digitalRead(RR_A) == LOW) { // A is low
      //CW
      countRearRight++;
    }
    else { // A is high
      //CCW
      countRearRight--;
    }
  }
  else { // B changed from high to low
    if(digitalRead(RR_A) == LOW) { // A is low
      //CCW
      countRearRight--;
    }
    else { // A is high
      //CW
      countRearRight++;
    }
  }
}


