// senseBox MCU (SAMD21 or S2) <- I2C -> Teensy 4.0 DNMS (-> MEMS mic)
// Reads LAeq (dBA) and LZeq (dBZ) after each Calculate command.

#include <Wire.h>
// #include <senseBoxIO.h> // for senseBox MCU (SAMD)

// senseBox MCU-S2 pins
#define I2C_SDA           PIN_QWIIC_SDA
#define I2C_SCL           PIN_QWIIC_SCL

#define WINDOW_MS         1000      // audio collection window in ms, change for longer/shorter avg
#define DNMS_I2C_ADDRESS  0x55      // as specified in Teensy's code

// DNMS Teensy command codes
#define DNMS_CMD_RESET            0x0001 // 1
#define DNMS_CMD_READ_VERSION     0x0002 // 2
#define DNMS_CMD_CALCULATE_LEQ    0x0003 // 3
#define DNMS_CMD_READ_DATA_READY  0x0004 // 4
#define DNMS_CMD_READ_LAEQ        0x0005 // 5
#define DNMS_CMD_READ_LZEQ        0x0011 // 17
#define DNMS_SET_ICS43434         0x001B // 27
#define DNMS_SET_IM72D128         0x001C // 28

#define CRC8_POLYNOMIAL           0x31
#define CRC8_INIT                 0xFF
#define CRC8_LEN                  1


static uint8_t crc8_word(const uint8_t *data) {
  uint16_t current_byte;
  uint8_t crc = CRC8_INIT;
  uint8_t crc_bit;
  /* calculates 8-Bit checksum with given polynomial */
  for (uint8_t i = 0; i < 2; ++i) {
    crc ^= data[i];
    for (crc_bit = 8; crc_bit > 0; --crc_bit) {
      if (crc & 0x80)
        crc = (crc << 1) ^ CRC8_POLYNOMIAL;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

// Write a 16-bit command (big-endian)
static bool writeCommand(uint16_t cmd) {
  Wire.beginTransmission(DNMS_I2C_ADDRESS);
  Wire.write((uint8_t)(cmd >> 8));
  Wire.write((uint8_t)(cmd & 0xFF));
  uint8_t err = Wire.endTransmission();
  return err == 0;
}

// Read exactly n bytes from wire.
static bool requestBytes(size_t n, uint8_t *data) {
  size_t got = Wire.requestFrom((int)DNMS_I2C_ADDRESS, (int)n);
  if (got < n) return false;
  for (size_t i = 0; i < n; ++i) {
    data[i] = Wire.read();
  }
  return true;
}

// Read "data_ready" (uint16_t + CRC).
static bool readDataReady(uint16_t &ready) {
  if (!writeCommand(DNMS_CMD_READ_DATA_READY)) return false;

  uint8_t buf[3];
  if (!requestBytes(3, buf)) return false;
  // Compare data with crc for corruption
  if (crc8_word(buf) != buf[2]) return false;
  ready = ((uint16_t)buf[0] << 8) | buf[1];
  return true;
}

// Read one float encoded as two 16-bit words with CRC per word: total 6 bytes.
static bool readFloat6(const uint8_t *p, float &out) {
  // Teensy replies are big-endian words with CRC-8 (poly 0x31, init 0xFF) per 16-bit word.
  // Floats are split into two words (MSW then LSW), each followed by its CRC byte: total 6 bytes per float.
  const uint8_t msw[2] = { p[0], p[1] }; const uint8_t crc1 = p[2];
  const uint8_t lsw[2] = { p[3], p[4] }; const uint8_t crc2 = p[5];

  // Verify crc per 16-bit word
  if (crc8_word(msw) != crc1) return false;
  if (crc8_word(lsw) != crc2) return false;

  // Reassemble to host (little-endian) order: b0,b1,b2,b3
  const uint8_t le_bytes[4] = { lsw[1], lsw[0], msw[1], msw[0] };
  memcpy(&out, le_bytes, 4); // Convert the 4 bytes to a float
  return true;
}

// Read three floats (Leq, Min, Max) after DNMS_CMD_READ_LAEQ or READ_LZEQ.
static bool readLeqTriplet(uint16_t cmd, float &leq, float &min, float &max) {
  if (!writeCommand(cmd)) return false;
  uint8_t raw[18];
  if (!requestBytes(sizeof raw, raw)) return false;
  return readFloat6(raw+0, leq) && readFloat6(raw+6, min) && readFloat6(raw+12, max);
}


void setup() {
  Serial.begin(9600);
  while (!Serial) {}  // TODO Add DEBUG to not block when not connected to USB

  Serial.println("\nDNMS Teensy data reader (senseBox MCU)");

  // Wire.begin(); // sensebox MCU (SAMD)
  // senseBoxIO.powerI2C(false);
  // senseBoxIO.powerI2C(true);

  Wire.begin(I2C_SDA, I2C_SCL); // sensebox MCU-S2
  
  Wire.setClock(100000);
  Wire.setTimeout(50);

  writeCommand(DNMS_SET_IM72D128);
  
  // Warm-up time to let Teensy accumulate some audio before first CALCULATE 
  delay(10000); // 10s Teensy start up time till first readings
}


void loop() {
  static uint32_t next_calc = 0;
  if (millis() < next_calc) return;

  // Ask Teensy to capture/compute a fresh block
  if (!writeCommand(DNMS_CMD_CALCULATE_LEQ)) {
    Serial.println("CALCULATE_LEQ failed");
    next_calc = millis() + WINDOW_MS;
    return;
  }

  // Poll data-ready
  uint16_t ready = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    if (readDataReady(ready) && ready == 1) break;
    delay(20);
  }

  if (ready != 1) {
    Serial.println("Timeout waiting for data_ready");    
    next_calc = millis() + WINDOW_MS;
    return;
  }

  // Read A-weighted triplet
  float laeq= NAN, la_min= NAN, la_max= NAN;
  if (!readLeqTriplet(DNMS_CMD_READ_LAEQ, laeq, la_min, la_max)) {
    Serial.println("READ_LAEQ failed (CRC/len)");    
    next_calc = millis() + WINDOW_MS;
    return;
  }

  // Read Z-weighted triplet
  // float lzeq= NAN, lz_min= NAN, lz_max= NAN;
  // if (!readLeqTriplet(DNMS_CMD_READ_LZEQ, lzeq, lz_min, lz_max)) {
  //   Serial.println("READ_LZEQ failed (CRC/len)");
  // }

  Serial.print("LAeq:"); Serial.print(laeq, 2); Serial.print('\t');
  Serial.print("LAmin:"); Serial.print(la_min, 2); Serial.print('\t');
  Serial.print("LAmax:"); Serial.println(la_max, 2);
  // Serial.print("LZeq:"); Serial.print(lzeq, 2); Serial.print('\t');
  // Serial.print("LZmin:"); Serial.print(lz_min, 2); Serial.print('\t');
  // Serial.print("LZmax:"); Serial.println(lz_max, 2);

  // Wait until window length to request dB data
  next_calc = millis() + WINDOW_MS;
}