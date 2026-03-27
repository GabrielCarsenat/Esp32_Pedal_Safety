#include <Arduino.h>
#include <Wire.h>

#define DEBUG true

#define POT1_PIN 34
#define POT2_PIN 35
#define LED_ERROR 2
#define CALBUTTON 4

#define AS5600_ADDR 0x36 // default I2C address

#define SAMPLE_MEANING_CNT 5

typedef struct sampling_data{
  unsigned int a:12;
  unsigned int b:12;
  unsigned char count;
} sampling_data_t;

enum DeviceState {
  RUNNING,
  CALIBRATION_RANGE,
  CALIBRATION_CARAC,
  ERROR
};

enum ErrCode {
  ERR_NONE,
  ERR_STD,
  ERR_SENSOR_MISSING,
  ERR_SENSOR_UNRESPONSIVE
};

unsigned long lastpress;
DeviceState state = ERROR;
DeviceState prevstate = RUNNING;

ErrCode errorcode = ERR_NONE;

uint16_t lowPot1 = 0;
uint16_t highPot1 = 4096;
uint16_t lowPot2 = 0;
uint16_t highPot2 = 4096;

//read status from AS5600
uint8_t readAS5600Status() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0B); // status register
  if (Wire.endTransmission() != 0) return 0xFF; // I2C error

  Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
  if (Wire.available() == 1) {
    return Wire.read();
  }
  return 0xFF; // error
}

// checksum computation
uint8_t computeChecksum(uint16_t a, uint16_t b, uint16_t c) {
  uint32_t sum = 0;
  sum += (a >> 8) & 0xFF;
  sum += a & 0xFF;
  sum += (b >> 8) & 0xFF;
  sum += b & 0xFF;
  sum += (c >> 8) & 0xFF;
  sum += c & 0xFF;
  return sum & 0xFF;
}

// read raw 12-bit angle from AS5600
// uint16_t readAS5600RawAngle() {
//   Wire.beginTransmission(AS5600_ADDR);
//   Wire.write(0x0C); // RAW ANGLE register high byte
//   if (Wire.endTransmission() != 0) {
//     // sensor did not ACK
//     return 0xFFFF;
//   }

//   Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
//   if (Wire.available() == 2) {
//     uint16_t highByte = Wire.read();
//     uint16_t lowByte = Wire.read();
//     return ((highByte << 8) | lowByte) & 0x0FFF; // mask 12-bit
//   }
//   return 0xFFFF; // error
// }

void IRAM_ATTR calmodeInterrupt() {
  unsigned long curr = millis();
  unsigned long dtime = lastpress - curr;
  if (dtime < 1000) {
    // debounce if needed
    return;
  }
  
  lastpress = curr;
  prevstate = state;
  state = CALIBRATION_RANGE;
}

void setup() {
  Serial.begin(115200);
  // Wire.begin(21, 22);
  analogReadResolution(12);

  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CALBUTTON, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(CALBUTTON), calmodeInterrupt, RISING);

  // check if sensor responds
  // Wire.beginTransmission(AS5600_ADDR);
  // if (Wire.endTransmission() == 0) {
  //   Serial.println("AS5600 detected!");
  //   state = RUNNING;
  // } else {
  //   Serial.println("AS5600 not detected!");
  //   state = ERROR;
  //   errcode = 1;
  // }
}

void loop() {
  if (state == RUNNING) {
    // uint16_t angle = readAS5600RawAngle();
    uint16_t angle=0;
    // if (angle == 0xFFFF) {
      // sensor lost
    //   state = ERROR;
    //   return;
    // }

    uint16_t pot1 = analogRead(POT1_PIN);
    uint16_t pot2 = analogRead(POT2_PIN);
    uint8_t checksum = computeChecksum(angle, pot1, pot2);

    // 7x 0xAA sync
    for (int i = 0; i < 7; i++) Serial.write(0xAA);
    Serial.write(0xAB);

    Serial.write((angle >> 8) & 0xFF);
    Serial.write(angle & 0xFF);

    Serial.write((pot1 >> 8) & 0xFF);
    Serial.write(pot1 & 0xFF);

    Serial.write((pot2 >> 8) & 0xFF);
    Serial.write(pot2 & 0xFF);

    Serial.write(checksum);
    Serial.write(0xBA);

    delay(20);

  } else if (state == CALIBRATION_RANGE) {
    uint16_t pot1 = analogRead(POT1_PIN);
    uint16_t pot2 = analogRead(POT2_PIN);

    if (pot1 > highPot1) highPot1 = pot1;
    if (pot1 < lowPot1) lowPot1 = pot1;
    if (pot2 > highPot2) highPot2 = pot2;
    if (pot2 < lowPot2) lowPot2 = pot2;

  } else if (state==CALIBRATION_CARAC){
    //n points = 100 50 based on pot 1 and 50 based on pot 2

    uint16_t pot1 = analogRead(POT1_PIN);
    uint16_t pot2 = analogRead(POT2_PIN);

    sampling_data_t samples[100];
    memset(samples, 0, 100);
    //then check in their respective areas where the samples could be added*
    unsigned int ix1 = 50*pot1/4096;
    unsigned int ix2 = 50+50*pot2/4096;
    if (samples[ix1].count<SAMPLE_MEANING_CNT){
      samples[ix1].a = pot1;
      samples[ix1].b = pot2;
      samples[ix1].count += 1;
    }
    if (!samples[ix2].count){
      samples[ix2].a = pot1;
      samples[ix2].b = pot2;
      samples[ix2].count += 1;
    }

  } else {
    // ERROR state: blink LED ominously
    while (1) {
      digitalWrite(LED_BUILTIN, 1); delay(40);
      digitalWrite(LED_BUILTIN, 0); delay(20);
      digitalWrite(LED_BUILTIN, 1); delay(40);
      digitalWrite(LED_BUILTIN, 0); delay(500);
    }
  }
}