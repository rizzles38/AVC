#include <stdint.h>


#include <FastGPIO.h>
#include <Servo.h>
#include <rover12_comm.h>

#define RR_B 0
#define RR_A 1
#define RL_B 2
#define RL_A 3
#define FR_A 10
#define FR_B 11
#define FL_A 8
#define FL_B 9
#define ORANGE_LED 12
#define RED_LED 13
#define ESTOP_IN 7
#define AUTONOMOUS_OUT 4
#define STEER_OUT 5
#define THROT_OUT 6

#define COMP_BAUD_RATE 115200

void orangeLED(int value) {
  FastGPIO::Pin<ORANGE_LED>::setOutput(value);
}

void redLED(int value) {
  FastGPIO::Pin<RED_LED>::setOutput(value);
}

class WheelEncoders {
public:
  explicit WheelEncoders(int interval) :
    count_rear_left(0),
    count_rear_right(0),
    next_time_(0),
    interval_(interval) {}

  void incrementRearLeft() { ++count_rear_left; }
  void decrementRearLeft() { --count_rear_left; }
  void incrementRearRight() { ++count_rear_right; }
  void decrementRearRight() { --count_rear_right; }

  void process() {
    unsigned long now = millis();

    if (next_time_ == 0) {
      next_time_ = now + interval_;
      return;
    } else if (now >= next_time_) {
      // Set the next sample time.
      next_time_ = now + interval_;

      // Create a message
      rover12_comm::WheelEncMsg wheel_enc_msg;

      // Populate each wheel encoder count
      wheel_enc_msg.data.count_rear_left = count_rear_left;
      wheel_enc_msg.data.count_rear_right = count_rear_right;

      // Encode and write to Serial
      wheel_enc_msg.encode();
      Serial.write(reinterpret_cast<uint8_t*>(&wheel_enc_msg), sizeof(wheel_enc_msg));
    }
  }

private:
  volatile int32_t count_rear_left;
  volatile int32_t count_rear_right;
  unsigned long next_time_;
  int interval_;
};

// Global variables.
WheelEncoders wheel_encoders(20); // 50 Hz report rate

void setup() {
  // Initialize LEDs.
  orangeLED(LOW);
  redLED(LOW);

  // Initialize wheel encoder pins.
  FastGPIO::Pin<RR_A>::setInput();
  FastGPIO::Pin<RR_B>::setInput();
  FastGPIO::Pin<RL_A>::setInput();
  FastGPIO::Pin<RL_B>::setInput();

  while (!Serial); // Wait for USB serial connection to open.
  Serial.begin(COMP_BAUD_RATE);

  attachInterrupt(digitalPinToInterrupt(RL_A), handleRLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_B), handleRLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_A), handleRRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_B), handleRRB, CHANGE);
}

void loop() {
  wheel_encoders.process();
}

void handleRLA() {
  if (FastGPIO::Pin<RL_A>::isInputHigh()) {    // A changed from low to high
    if (!FastGPIO::Pin<RL_B>::isInputHigh()) { // B is low
      wheel_encoders.decrementRearLeft();      //   CCW
    } else {                                   // B is high
      wheel_encoders.incrementRearLeft();      //   CW
    }
  } else {                                      // A changed from high to low
    if (!FastGPIO::Pin<RL_B>::isInputHigh()) { // B is low
      wheel_encoders.incrementRearLeft();      //   CW
    } else {                                   // B is high
      wheel_encoders.decrementRearLeft();      //   CCW
    }
  }
}

void handleRLB() {
  if (FastGPIO::Pin<RL_B>::isInputHigh()) {     // B changed from low to high
    if (!FastGPIO::Pin<RL_A>::isInputHigh()) {  // A is low
      wheel_encoders.incrementRearLeft();       //   CW
    } else {                                    // A is high
      wheel_encoders.decrementRearLeft();       //   CCW
    }
  } else {                                      // B changed from high to low
    if (!FastGPIO::Pin<RL_A>::isInputHigh()) {  // A is low
      wheel_encoders.decrementRearLeft();       //   CCW
    } else {                                    // A is high
      wheel_encoders.incrementRearLeft();       //   CW
    }
  }
}

void handleRRA() {
  if (FastGPIO::Pin<RR_A>::isInputHigh()) {    // A changed from low to high
    if (!FastGPIO::Pin<RR_B>::isInputHigh()) { // B is low
      wheel_encoders.decrementRearRight();     //   CCW
    } else {                                   // B is high
      wheel_encoders.incrementRearRight();     //   CW
    }
  } else {                                     // A changed from high to low
    if (!FastGPIO::Pin<RR_B>::isInputHigh()) { // B is low
      wheel_encoders.incrementRearRight();     // CW
    } else {                                   // B is high
      wheel_encoders.decrementRearRight();     //   CCW
    }
  }
}

void handleRRB() {
  if (FastGPIO::Pin<RR_B>::isInputHigh()) {    // B changed from low to high
    if (!FastGPIO::Pin<RR_A>::isInputHigh()) { // A is low
      wheel_encoders.incrementRearRight();     //   CW
    } else {                                   // A is high
      wheel_encoders.decrementRearRight();     //   CCW
    }
  } else {                                     // B changed from high to low
    if (!FastGPIO::Pin<RR_A>::isInputHigh()) { // A is low
      wheel_encoders.decrementRearRight();     //   CCW
    } else {                                   // A is high
      wheel_encoders.incrementRearRight();     //   CW
    }
  }
}
