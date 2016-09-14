#include <stdint.h>
#include <string.h>

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
  explicit PIControl(int interval)
    : target_(0.0f),
      output_(0.0f),
      cum_error_(0.0f),
      p_gain_(0.0f), // gains pulled out of ass
      i_gain_(0.0f),
      next_time_(0),
      interval_(interval),
      dt_(1.0 / interval) {}

  void setGains(float p_gain, float i_gain) {
    p_gain_ = p_gain;
    i_gain_ = i_gain;
  }

  void setTarget(float target) {
    target_ = target;
  }

  int getOutput() {
    return output_;
  }

  void process(float measured) {
    unsigned long now = millis();
    if (next_time_ == 0) {
      next_time_ = now + interval_;
    } else if (now >= next_time_) {
      next_time_ = now + interval_;
 
      // equation for PI control loop
      float error = target_ - measured;
      cum_error_ += error * dt_;
      float result = p_gain_ * error + i_gain_ * cum_error_;  // this will be normalized using gains

      // Remap to -1->1, then to 1000 -> 2000 (servo microseconds).
      if (result > 1.0) {
        result = 1.0;
      } else if (result < -1.0) {
        result = -1.0;
      }
      output_ = (int)500.0f * result + 1500.0f;
    }
  }

private:
  float target_;
  int output_;
  float cum_error_;
  float p_gain_;
  float i_gain_;
  unsigned long next_time_;
  int interval_;
  float dt_;
};

class Messenger {
public:
  explicit Messenger(PIControl& pi_control, float& steering_us, bool& estop)
    : buffer_idx_(-1),
      pi_control_(pi_control),
      steering_us_(steering_us),
      estop_(estop),
      last_autonomous_(false) {
    memset(buf_, 0, sizeof(buf_));  
  }

  void setup() {
    while (!Serial); // Wait for USB serial connection to open.
    Serial.begin(COMP_BAUD_RATE);
  }

  void process(bool autonomous_mode) {
    // Read bytes from the serial port into our buffer.
    while (Serial.available() > 0) {
      int b = Serial.read();
      if (buffer_idx_ < 0) {
        // Not synced yet.
        if (b == 0x0) {
          // Sync!
          buffer_idx_ = 0;
        }
      } else {
        buf_[buffer_idx_] = b;
        if (b == 0x0) {
          dispatchMessage();
          
        } else {
          ++buffer_idx_;
          if (buffer_idx_ >= sizeof(buf_)) {
            redLED(HIGH);
            buffer_idx_ = -1;
          }
        }
      }
    }

    if (autonomous_mode != last_autonomous_) {
      // Send update to the AI telling it the autonomous status changed.
      rover12_comm::EstopMsg msg;
      msg.data.autonomous = autonomous_mode;
      msg.encode();
      Serial.write(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
      last_autonomous_ = autonomous_mode;
    }
  }

private:
  void dispatchMessage() {
    int8_t type = static_cast<int8_t>(buf_[0]);
    switch (static_cast<rover12_comm::MsgType>(type)) {
      case rover12_comm::MsgType::ESTOP: {
        auto ptr = reinterpret_cast<rover12_comm::EstopMsg*>(buf_);
        ptr->decode();
        estopMessage(*ptr);
      } break;

      case rover12_comm::MsgType::CONTROL: {
        auto ptr = reinterpret_cast<rover12_comm::ControlMsg*>(buf_);
        ptr->decode();
        controlMessage(*ptr);
      } break;

      case rover12_comm::MsgType::PID_GAINS: {
        auto ptr = reinterpret_cast<rover12_comm::PidGainsMsg*>(buf_);
        ptr->decode();
        pidGainsMessage(*ptr);
      } break;

      default:
        redLED(HIGH);
        break;
    }
  }

  void estopMessage(const rover12_comm::EstopMsg& msg) {
    estop_ = msg.data.autonomous;
  }

  void controlMessage(const rover12_comm::ControlMsg& msg) {
    // Mapping determined by steering calibration regression.
    steering_us_ = -1118.39f * msg.data.steering_angle + 1556.265f;
    pi_control_.setTarget(msg.data.velocity);
  }

  void pidGainsMessage(const rover12_comm::PidGainsMsg& msg) {
    pi_control_.setGains(msg.data.kp, msg.data.ki);
  }

  uint8_t buf_[100];
  int buffer_idx_;
  PIControl& pi_control_;
  float& steering_us_;
  bool& estop_;
  bool last_autonomous_;
};

// Global variables.
bool estop; // True = Software requests autonomous mode.
float steering_us;
float throttle_us;
WheelEncoders wheel_encoders(20); // 50 Hz report rate
PIControl throttle_control(0.02); // PIControl expects we're operating at 50 Hz
Messenger messenger(throttle_control, steering_us, estop);
Servo steering_servo;
Servo throttle_servo;

void setup() {
  // Initialize LEDs.
  orangeLED(LOW);
  redLED(LOW);

  // Initialize wheel encoder pins.
  FastGPIO::Pin<RR_A>::setInput();
  FastGPIO::Pin<RR_B>::setInput();
  FastGPIO::Pin<RL_A>::setInput();
  FastGPIO::Pin<RL_B>::setInput();

  // Initialize estop I/Os.
  FastGPIO::Pin<ESTOP_IN>::setInput();
  FastGPIO::Pin<AUTONOMOUS_OUT>::setOutputLow();

  // Initialize serial messenger.
  messenger.setup();

  // Hook up encoder interrupts.
  attachInterrupt(digitalPinToInterrupt(RL_A), handleRLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_B), handleRLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_A), handleRRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_B), handleRRB, CHANGE);

  // Attach to steering/throttle servos.
  steering_servo.attach(STEER_OUT);
  throttle_servo.attach(THROT_OUT);

  // We're in autonomous mode if both the software and the physical switch
  // think we should be.
}

void loop() {
  // Are we actually in autonomous mode? We only are if both the software and the
  // physical switch agree that we are.
  bool autonomous_mode = FastGPIO::Pin<ESTOP_IN>::isInputHigh() && estop;
  FastGPIO::Pin<AUTONOMOUS_OUT>::setOutput(autonomous_mode);
  
  wheel_encoders.process();
  messenger.process(autonomous_mode);
  throttle_control.process(wheel_encoders.getVelocity());
  throttle_us = throttle_control.getOutput();

  // Set steering and throttle commands.
  steering_servo.writeMicroseconds(steering_us);
  throttle_servo.writeMicroseconds(throttle_us);
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
