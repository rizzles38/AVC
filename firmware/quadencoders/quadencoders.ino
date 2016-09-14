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
    count_rear_left_(0),
    count_rear_right_(0),
    prev_count_rear_left_(0),
    prev_count_rear_right_(0),
    next_time_(0),
    interval_(interval) {}

  void incrementRearLeft() { ++count_rear_left_; }
  void decrementRearLeft() { --count_rear_left_; }
  void incrementRearRight() { ++count_rear_right_; }
  void decrementRearRight() { --count_rear_right_; }

  float getVelocity() {
    return cur_velocity_;
  }

  // 50Hz
  void process() {
    unsigned long now = millis();

    // Compute instananeous velocity on each wheel based on previous tick
    // count.
    const int32_t rl_tick_delta = count_rear_left_ - prev_count_rear_left_;
    const int32_t rr_tick_delta = count_rear_right_ - prev_count_rear_right_;

    // TODO: Make these params (everything in meters).
    const float wheel_diameter = 0.102;
    const int num_stripes = 20.0;
    const float wheel_circumference = wheel_diameter * M_PI;
    const float meters_per_tick = wheel_circumference / (4.0 * num_stripes);

    // Translate tick counts to distances traveled.
    const float rl_distance = rl_tick_delta * meters_per_tick;
    const float rr_distance = rr_tick_delta * meters_per_tick;

    // Assume 50 Hz, this is bad. Maybe time it on the Arduino and include the
    // measurement in the packet? TODO
    const float time_delta = 0.020; // 50 Hz
    const float rl_speed = rl_distance / time_delta;
    const float rr_speed = rr_distance / time_delta;

    // Averaging wheel speeds gives a pretty good estimate of true speed.
    cur_velocity_ = (rl_speed + rr_speed) / 2.0;

    if (next_time_ == 0) {
      next_time_ = now + interval_;
      return;
    } else if (now >= next_time_) {
      // Set the next sample time.
      next_time_ = now + interval_;

      // Create a message
      rover12_comm::WheelEncMsg wheel_enc_msg;

      // Populate each wheel encoder count
      wheel_enc_msg.data.count_rear_left = count_rear_left_;
      wheel_enc_msg.data.count_rear_right = count_rear_right_;

      // Encode and write to Serial
      wheel_enc_msg.encode();
      Serial.write(reinterpret_cast<uint8_t*>(&wheel_enc_msg), sizeof(wheel_enc_msg));
    }
  }

private:
  float cur_velocity_;
  int32_t prev_count_rear_left_;
  int32_t prev_count_rear_right_;
  volatile int32_t count_rear_left_;
  volatile int32_t count_rear_right_;
  unsigned long next_time_;
  int interval_;
};

// Takes in the desired velocity and uses the current velocity estimate from wheel encoders to output a throttle commmand
class PIControl {
public:
  PIControl(float p_gain, float i_gain) :
    p_gain_(p_gain),
    i_gain_(i_gain),
    cum_error_(0) {}

  float process(float target, float measured) {
    // FINISH THIS METHOD
    // equation for PI control loop
    float error = target - measured;
    cum_error_ += error;
    float output = p_gain_ * error + i_gain_ * cum_error_;  // this will be normalized using gains
    return output;
  }
  

private:
  float cum_error_;
  float p_gain_;
  float i_gain_;
};

// Global variables.
WheelEncoders wheel_encoders(20); // 50 Hz report rate
PIControl throttle_control(0, 0);  // gains pulled out of ass

//Instantiate a PIDControl

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
  float output_velocity = throttle_control.process(target_velocity, wheel_encoders.getVelocity());

  if (output_velocity > 1.0) {
      output_velocity = 1.0;
  }
  if (output_velocity < -1.0) {
      output_velocity = -1.0;
  }
  output_throttle = output_velocity * 500 + 1500;

  // send throttle command
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
