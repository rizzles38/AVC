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

void debugMsg(const char* s) {
  rover12_comm::DebugMsg debug_msg;
  debug_msg.data.setName(s);
  debug_msg.encode();
  Serial.write(reinterpret_cast<uint8_t*>(&debug_msg), sizeof(debug_msg));
}

void debugInt(const char* name, int32_t i) {
  rover12_comm::DebugMsg debug_msg;
  debug_msg.data.setInt(name, i);
  debug_msg.encode();
  Serial.write(reinterpret_cast<uint8_t*>(&debug_msg), sizeof(debug_msg));
}

void debugFloat(const char* name, float f) {
  rover12_comm::DebugMsg debug_msg;
  debug_msg.data.setFloat(name, f);
  debug_msg.encode();
  Serial.write(reinterpret_cast<uint8_t*>(&debug_msg), sizeof(debug_msg));
}

class WheelEncoders {
public:
  explicit WheelEncoders(int interval) :
    count_rear_left_(0),
    count_rear_right_(0),
    next_time_(0),
    interval_(interval) {}

  void incrementRearLeft() { ++count_rear_left_; }
  void decrementRearLeft() { --count_rear_left_; }
  void incrementRearRight() { ++count_rear_right_; }
  void decrementRearRight() { --count_rear_right_; }

  // 50Hz
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
      wheel_enc_msg.data.count_rear_left = count_rear_left_;
      wheel_enc_msg.data.count_rear_right = count_rear_right_;

      // Encode and write to Serial
      wheel_enc_msg.encode();
      Serial.write(reinterpret_cast<uint8_t*>(&wheel_enc_msg), sizeof(wheel_enc_msg));
    }
  }

private:
  volatile int32_t count_rear_left_;
  volatile int32_t count_rear_right_;
  unsigned long next_time_;
  int interval_;
};

class Messenger {
public:
  explicit Messenger()
    : buffer_idx_(-1),
      steering_us_(1556),
      throttle_us_(1500),
      estop_(false),
      last_estop_time_(0),
      next_time_(0) {
    memset(buf_, 0, sizeof(buf_));
  }

  // Getters for data managed by the messenger.
  int steering_us() const { return steering_us_; }
  int throttle_us() const { return throttle_us_; }
  bool estop() const { return estop_; }
  unsigned long last_estop_time() const { return last_estop_time_; }

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
          buffer_idx_ = 0;
        } else {
          ++buffer_idx_;
          if (buffer_idx_ >= sizeof(buf_)) {
            redLED(HIGH);
            buffer_idx_ = -1;
          }
        }
      }
    }

    unsigned long now = millis();
    if (next_time_ == 0) {
      next_time_ = now + 100;
    } else if (now >= next_time_) {
      next_time_ = now + 100;

      // Send update to the driving computer @ 10 Hz with the autonomous status.
      rover12_comm::EstopMsg msg;
      msg.data.autonomous = autonomous_mode;
      msg.encode();
      Serial.write(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
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

      default:
        redLED(HIGH);
        break;
    }
  }

  void estopMessage(const rover12_comm::EstopMsg& msg) {
    estop_ = msg.data.autonomous;
    last_estop_time_ = millis();
  }

  void controlMessage(const rover12_comm::ControlMsg& msg) {
    steering_us_ = msg.data.steering_us;
    throttle_us_ = msg.data.throttle_us;
  }

  // Buffer for serial communication.
  uint8_t buf_[100];
  int buffer_idx_;

  // Steering and throttle servo durations (in microseconds).
  int steering_us_;
  int throttle_us_;

  // True = software is requesting autonomous mode.
  bool estop_;

  // Last time we saw an estop message (of any kind).
  unsigned long last_estop_time_;

  // Next time we should send an autonomous mode status update.
  unsigned long next_time_;
};

// Global variables.
WheelEncoders wheel_encoders(20); // 50 Hz report rate
Messenger messenger; // Communication with driving computer.
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
}

void loop() {
  // Are we actually in autonomous mode? We only are if both the software and the
  // physical switch agree that we are.
  bool autonomous_mode = !FastGPIO::Pin<ESTOP_IN>::isInputHigh() && // active low
                         messenger.estop() &&
                         messenger.last_estop_time() > (millis() - 500);
  FastGPIO::Pin<AUTONOMOUS_OUT>::setOutput(autonomous_mode);

  wheel_encoders.process();
  messenger.process(autonomous_mode);

  // Set steering and throttle commands.
  if (autonomous_mode) {
    steering_servo.writeMicroseconds(messenger.steering_us());
    throttle_servo.writeMicroseconds(messenger.throttle_us());
  } else {
    steering_servo.writeMicroseconds(1556);
    throttle_servo.writeMicroseconds(1500);
  }
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
