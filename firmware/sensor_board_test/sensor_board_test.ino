#include <stdint.h>

#include <FastGPIO.h>

#define BLUE_LED 5
#define ORANGE_LED 6
#define RED_LED 7
#define GPS_INIT_BAUD_RATE 9600
#define GPS_FAST_BAUD_RATE 115200
#define COMP_BAUD_RATE 115200

union BytesU16 {
    uint16_t value;
    uint8_t bytes[2];
};

uint16_t swapU16(uint16_t value) {
  BytesU16 b;
  b.value = value;
  return (b.bytes[0] << 8) | b.bytes[1];
}

void blueLED(int value) {
  FastGPIO::Pin<BLUE_LED>::setOutput(value);
}

void orangeLED(int value) {
  FastGPIO::Pin<ORANGE_LED>::setOutput(value);
}

void redLED(int value) {
  FastGPIO::Pin<RED_LED>::setOutput(value);
}

class Venus {
public:
  Venus()
    : nav_payload_({0}),
      payload_idx_(-1),
      parse_state_(ParseState::WAITING) {}

  // Configured the SkyTraq Venus 6 GPS receiver using its binary protocol from its
  // factory state.
  static bool config() {
    Serial1.begin(GPS_INIT_BAUD_RATE);
    delay(500);

    // Set baud rate to 115200.
    const uint8_t baud_rate_cmd[] = {0x05, 0, 5, 0};
    bool baud_rate_result = command(baud_rate_cmd, sizeof(baud_rate_cmd), true);
    if (!baud_rate_result) return false;
    Serial1.begin(GPS_FAST_BAUD_RATE);
    delay(100);

    // Set update rate to 10 Hz.
    const uint8_t update_rate_cmd[] = {0x0e, 10, 0};
    bool update_rate_result = command(update_rate_cmd, sizeof(update_rate_cmd), true);
    if (!update_rate_result) return false;
    delay(100);

    // Enable WAAS for better accuracy.
    const uint8_t waas_cmd[] = {0x37, 1, 0};
    bool waas_result = command(waas_cmd, sizeof(waas_cmd), true);
    if (!waas_result) return false;
    delay(100);

    // Set navigation mode to car.
    const uint8_t nav_mode_cmd[] = {0x3c, 0, 0};
    bool nav_mode_result = command(nav_mode_cmd, sizeof(nav_mode_cmd), true);
    if (!nav_mode_result) return false;
    delay(100);

    // Set message mode to binary.
    const uint8_t message_mode_cmd[] = {0x09, 2};
    bool message_mode_result = command(message_mode_cmd, sizeof(message_mode_cmd), true);
    if (!message_mode_result) return false;
    delay(100);
    
    return true;
  }

  void process() {
    while (Serial1.available() > 0) {
      uint8_t b = Serial1.read();
      if (parse_state_ == ParseState::WAITING) {
        if (navWaiting(b)) parse_state_ = ParseState::BUFFERING;
      } else if (parse_state_ == ParseState::BUFFERING) {
        if (navBuffering(b)) parse_state_ = ParseState::VALIDATING;
      } else if (parse_state_ == ParseState::VALIDATING) {
        navValidate(b);
        parse_state_ = ParseState::WAITING;
      }
    }
  }

private:
  enum class ParseState {
    WAITING,
    BUFFERING,
    VALIDATING
  };

  // Returns true if the first 5 bytes of a nav message have been observed.
  bool navWaiting(uint8_t b) {
    // Shuffle the first 5 bytes.
    for (int i = 0; i < 4; ++i) {
      nav_payload_[i] = nav_payload_[i + 1];
    }
    nav_payload_[4] = b;

    // Are we just now seeing a nav message?
    if (nav_payload_[0] == 0xa0 && nav_payload_[1] == 0xa1 &&
        nav_payload_[2] == 0x0 && nav_payload_[3] == 0x3b &&
        nav_payload_[4] == 0xa8) {
      nav_payload_[0] = 0xa8;
      payload_idx_ = 1;
      return true;
    }

    return false;
  }

  // Returns true if the full nav message has been buffered.
  bool navBuffering(uint8_t b) {
    nav_payload_[payload_idx_] = b;
    ++payload_idx_;
    if (payload_idx_ > 58) {
      payload_idx_ = -1;
      return true;
    }

    return false;
  }

  // Returns true if the checksum passes.
  void navValidate(uint8_t b) {
    if (checksum(nav_payload_, sizeof(nav_payload_)) == b) {
      // Checksum passes, forward the data packet out the USB serial link.
      Serial.print("WOO! ");
      Serial.print(b, HEX);
      Serial.print("\n");
    }
  }

  // Computes the Venus checksum over the given buffer.
  static uint8_t checksum(uint8_t* buf, int len) {
    uint8_t checksum = 0x00;
    for (int i = 0; i < len; ++i) {
      checksum = checksum ^ buf[i];
    }
    return checksum;
  }

  // Blocks on the Venus serial buffer until we see an ACK or NACK of message_id.
  static bool waitForResponse(uint8_t message_id) {
    const int buf_size = 9;
    uint8_t buf[buf_size] = {0};

    while (true) {
      if (Serial1.available() > 0) {
        // Shuffle the buffer and read a byte into the end.
        for (int i = 0; i < buf_size; ++i) {
          buf[i] = buf[i + 1];
        }
        buf[buf_size - 1] = Serial1.read();

        // Scan for ACK or NACK message.
        if (buf[0] == 0xa0 && buf[1] == 0xa1 &&
            buf[2] == 0x00 && buf[3] == 0x02 &&
            buf[7] == 0x0d && buf[8] == 0x0a) {
          // Validate the checksum.
          if (checksum(&buf[4], 2) == buf[6]) {
            // Is it the message we're waiting for?
            if (buf[5] == message_id) {
              if (buf[4] == 0x83) {
                // It's an ACK!
                return true;
              } else if (buf[4] == 0x84) {
                // It's a NACK!
                return false;
              }
            }
          }
        }
      }
    }
  }

  // Send a command with the given buffer and length with the appropriate framing
  // and checksum. If block_for_ack is true, then we'll swallow all serial data
  // until we see an ACK or NACK for the message we sent.
  static bool command(uint8_t* buf, uint16_t len, bool block_for_ack = false) {
    BytesU16 len_bytes;
    len_bytes.value = swapU16(len);
    
    Serial1.write(0xa0);
    Serial1.write(0xa1);
    Serial1.write(len_bytes.bytes[0]);
    Serial1.write(len_bytes.bytes[1]);
    Serial1.write(buf, len);
    Serial1.write(checksum(buf, len));
    Serial1.write(0x0d);
    Serial1.write(0x0a);

    if (block_for_ack) {
      orangeLED(HIGH);
      bool result = waitForResponse(buf[0]);
      orangeLED(LOW);
      return result;
    }

    return true;
  }

private:
  uint8_t nav_payload_[59];
  int payload_idx_;
  ParseState parse_state_;
};

void setup() {
  blueLED(LOW);
  orangeLED(LOW);
  redLED(LOW);
  
  if (!Venus::config()) {
    redLED(HIGH);
  } else {
    blueLED(HIGH);
  }

  while (!Serial); // Wait for USB serial connection to open.
  Serial.begin(COMP_BAUD_RATE);
}

Venus venus;
static int counter = 0;

void loop() {
  venus.process();
/*
  if (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }
*/
/*
  if (Serial1.available() > 0) {
    Serial.print(Serial1.read(), HEX);
    Serial.print(" ");
    ++counter;
    if (counter >= 20) {
      counter = 0;
      Serial.print("\n");
    }
  }
*/
}
