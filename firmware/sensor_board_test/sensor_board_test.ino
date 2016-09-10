#include <stdint.h>

#include <FastGPIO.h>

#define BLUE_LED 5
#define ORANGE_LED 6
#define RED_LED 7
#define GPS_INIT_BAUD_RATE 9600
#define GPS_FAST_BAUD_RATE 115200
#define COMP_BAUD_RATE 115200

void blueLED(int value) {
  FastGPIO::Pin<BLUE_LED>::setOutput(value);
}

void orangeLED(int value) {
  FastGPIO::Pin<ORANGE_LED>::setOutput(value);
}

void redLED(int value) {
  FastGPIO::Pin<RED_LED>::setOutput(value);
}

uint8_t venusChecksum(uint8_t* buf, int len) {
  uint8_t checksum = 0x00;
  for (int i = 0; i < len; ++i) {
    checksum = checksum ^ buf[i];
  }
  return checksum;
}

// Blocks on the Venus serial buffer until we see an ACK or NACK of message_id.
bool venusWaitForResponse(uint8_t message_id) {
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
        if (venusChecksum(&buf[4], 2) == buf[6]) {
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

bool venusCommand(uint8_t* buf, uint16_t len) {
  union {
    uint16_t value;
    uint8_t bytes[2];
  } big_endian_len;
  big_endian_len.value = len;
  
  Serial1.write(0xa0);
  Serial1.write(0xa1);
  Serial1.write(big_endian_len.bytes[1]);
  Serial1.write(big_endian_len.bytes[0]);
  Serial1.write(buf, len);
  Serial1.write(venusChecksum(buf, len));
  Serial1.write(0x0d);
  Serial1.write(0x0a);

  orangeLED(HIGH);
  bool result = venusWaitForResponse(buf[0]);
  orangeLED(LOW);
  return result;
}

// Configures the SkyTraq Venus 6 GPS Receiver using its binary protocol.
bool venusConfig() {
  delay(500);

  // Set baud rate to 115200.
  const uint8_t baud_rate_cmd[] = {
    0x05,
    0x00,
    0x05,
    0x00
  };
  bool baud_rate_result = venusCommand(baud_rate_cmd, sizeof(baud_rate_cmd));
  if (!baud_rate_result) return false;
  Serial1.begin(GPS_FAST_BAUD_RATE);

  delay(100);

  // Set update rate to 10 Hz.
  const uint8_t update_rate_cmd[] = {
    0x0e,
    10,
    0x00
  };
  bool update_rate_result = venusCommand(update_rate_cmd, sizeof(update_rate_cmd));
  if (!update_rate_result) return false;

  delay(100);

  // Enable WAAS for better accuracy.
  const uint8_t waas_cmd[] = {
    0x37,
    1,
    0x00
  };
  bool waas_result = venusCommand(waas_cmd, sizeof(waas_cmd));
  if (!waas_result) return false;

  return true;
}

void setup() {
  blueLED(LOW);
  orangeLED(LOW);
  redLED(LOW);
  
  Serial1.begin(GPS_INIT_BAUD_RATE);
  if (!venusConfig()) {
    redLED(HIGH);
  } else {
    blueLED(HIGH);
  }

  while (!Serial); // Wait for USB serial connection to open.
  Serial.begin(COMP_BAUD_RATE);
}

static int counter = 0;

void loop() {
  if (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }

  /*
  if (Serial1.available() > 0) {
    Serial.print(Serial1.read(), HEX);
    Serial.print(" ");
    ++counter;
    if (counter >= 20) {
      counter = 0;
      Serial.print("\n");
    }
  }*/
}
