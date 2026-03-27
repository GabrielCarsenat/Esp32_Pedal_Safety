#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#define AS5600_ADDR 0x36

#define POT1_PIN 34
#define POT2_PIN 35
#define LED_ERROR 2
#define LED_BUILTIN 2
#define CALBUTTON #xxxxx //THIS MUST BE DEFINED BEFORE UPLOAD...

enum DeviceState {
  RUNNING,
  CALIBRATION_RANGE,
  CALIBRATION_CARAC,
  ERROR
};

DeviceState state=RUNNING;
DeviceState prevstate=RUNNING;

//TODO: figure out what to do on these intitial values???
uint16_t lowPot1 = 0;
uint16_t highPot1 = 4096;

uint16_t lowPot2 = 0;
uint16_t highPot2 = 4096;

typedef struct data{
  unsigned int a:12;
  unsigned int b:12;
} data_t;

//24bits = 8*3 = 3bytes

uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);

  if (Wire.available() == 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  }
  return 0;
}

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

void calmodeInterrupt(){
  prevstate = state;
  state=CALIBRATION_RANGE;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  analogReadResolution(12);
  digitalWrite(LED_ERROR, 0);

  attachInterruptArg(digitalPinToInterrupt(CALBUTTON), calmodeInterrupt, RISING);
}

void loop() {
  if (state==RUNNING){
    uint16_t angle = readAS5600RawAngle();
    uint16_t pot1 = analogRead(POT1_PIN);
    uint16_t pot2 = analogRead(POT2_PIN);

    uint8_t checksum = computeChecksum(angle, pot1, pot2);

    // 7x 0xAA sync
    for (int i = 0; i < 7; i++) {
      Serial.write(0xAA);
    }

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
  } else if (state==CALIBRATION_RANGE){
    //we enter calibration mode:
    //first step: try the range of your 
    //store values continuously inside a buffer
    //the sample a normalised density over the whole range, taking at leas 1 point in low density parts.
    uint16_t pot1 = analogRead(POT1_PIN);
    uint16_t pot2 = analogRead(POT2_PIN);
    if (pot1>highPot1){
      highPot1=pot1;
    }
    if (pot1<lowPot1){
      lowPot1=pot1;
    }
    if(pot2>highPot2){
      highPot2=pot2;
    }
    if(pot2<lowPot2){
      lowPot2=pot2;
    }
  } else {
    digitalWrite(LED_ERROR, 1);
    while (1){
      //do nothing i hope this isn't optimized out
    }
  }
}