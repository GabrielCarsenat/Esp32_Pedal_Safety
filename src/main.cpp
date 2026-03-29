#include <Arduino.h>
#include <Wire.h>

#define DEBUG true

#define POT1_PIN 27
#define POT2_PIN 26
#define LED_ERROR 2
#define CALBUTTON 4

#define AS5600_ADDR 0x36 // default I2C address

//careful not to exceed max sample count: memory is 512 bytes and we have three bytes per sample
//so 170 is the upper cap + don't forget possible additional meta for number of samples and max/min ranges retained.
//also whether there's been a calibration at all. 
#define NSAMPLES 100
//sample_meaning_cnt is a unsiggned char on 1byte: should abs not exceed 127
#define SAMPLE_MEANING_CNT 5
//this does less sample coverage checks, careful not to overflow the uint_16_t.
#define SAMPLE_CHECK_FREQ 5

typedef struct sampling_data{
  uint_fast16_t a;
  unsigned int b;
  unsigned char count;
} sampling_data_t;

enum DeviceState {
  RUNNING,
  CALIBRATION_RANGE,
  CALIBRATION_CARAC,
  CALIBRATION_SAVE,
  ERROR
};

enum ErrCode {
  ERR_NONE,
  ERR_STD,
  ERR_SENSOR_MISSING,
  ERR_SENSOR_UNRESPONSIVE
};

unsigned long lastpress;
DeviceState state = CALIBRATION_CARAC;
DeviceState prevstate = CALIBRATION_RANGE;

ErrCode errorcode = ERR_NONE;

uint16_t lowPot1 = 0;
uint16_t highPot1 = 4096;
uint16_t lowPot2 = 0;
uint16_t highPot2 = 4096;

long pot1DynMean;
long pot2DynMean;

uint16_t sample_chk; //initialized at 1 down the line
uint16_t true_cnt;
sampling_data_t samples[NSAMPLES]; //set to 0's down the line

//read status from AS5600
// uint8_t readAS5600Status() {
//   Wire.beginTransmission(AS5600_ADDR);
//   Wire.write(0x0B); // status register
//   if (Wire.endTransmission() != 0) return 0xFF; // I2C error

//   Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
//   if (Wire.available() == 1) {
//     return Wire.read();
//   }
//   return 0xFF; // error
// }

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

void IRAM_ATTR calmodeInterrupt() { //IRAM_ATTR 
  unsigned long curr = millis();
  unsigned long dtime = lastpress - curr;
  if (dtime < 1000) {
    // debounce it x-x
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
  analogSetAttenuation(ADC_11db); //needed to read a full 3.3 v range
  
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
  #ifdef DEBUG
    Serial.print(">state:");
    Serial.printf("%d\n",state);
  #endif
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
    if (prevstate==RUNNING){
      prevstate=CALIBRATION_RANGE;
      highPot1=0;
      lowPot1=4096;
      highPot2=0;
      lowPot2=4096;
    }
    uint16_t pot1 = analogRead(POT1_PIN);
    uint16_t pot2 = analogRead(POT2_PIN);

    if (pot1 > highPot1) highPot1 = pot1;
    if (pot1 < lowPot1) lowPot1 = pot1;
    if (pot2 > highPot2) highPot2 = pot2;
    if (pot2 < lowPot2) lowPot2 = pot2;

    #ifdef DEBUG
      Serial.print(">pot1: ");
      Serial.printf("%d\n", pot1);
      Serial.print(">pot2: ");
      Serial.printf("%d\n", pot2);
      Serial.print(">Hpot1: ");
      Serial.printf("%d\n", highPot1);
      Serial.print(">Hpot2: ");
      Serial.printf("%d\n", highPot2);
      Serial.print(">Lpot1: ");
      Serial.printf("%d\n", lowPot1);
      Serial.print(">Lpot2: ");
      Serial.printf("%d\n", lowPot2);
    #endif
  
  } else if (state==CALIBRATION_CARAC){
    if (prevstate==CALIBRATION_RANGE){
      sample_chk = 1; //

      //this handles the true counts of our individual range measurements
      true_cnt = 0;
      // //these means are updated dynamically to check the centering of our samples.
      // pot1DynMean=0;
      // pot2DynMean=0;
      //then also check for interval between values?
      //the mean of the intervals btw values should be less than 2/3? value to tweak? /!\ account for edges as spaces /!\
      //the actual value should only depend and be a function of the true_cnt required count whoch would be 50?.
      //this cannot be computed on the fly and will require a loop if we pass mean and true count.
      //and should solve our distribution "width" too small problem.
      memset(samples, 0, sizeof(sampling_data_t)*NSAMPLES);
      prevstate=CALIBRATION_CARAC; //DO NOT FORGET ELSE WE REWRITE ON LOOP
    }
    //n points = 100 50 based on pot 1 and 50 based on pot 2
    
    uint16_t pot1 = analogRead(POT1_PIN);
    uint16_t pot2 = analogRead(POT2_PIN);

    #ifdef DEBUG
      Serial.print(">pot1: ");
      Serial.printf("%d\n", pot1);
      Serial.print(">pot2: ");
      Serial.printf("%d\n", pot2);
    #endif
    //on peut utilsier teleplot pour tester
    
    //then check in their respective areas where the samples could be added*
    unsigned int ix1 = (int) min(max(double(pot1-lowPot1)/(highPot1-lowPot1), 0.0) * NSAMPLES, (double) NSAMPLES-1);
    // unsigned int ix2 = min(max((pot2-lowPot2)/(highPot2-lowPot2), 0), NSAMPLES-1);
    unsigned char cnt1 = samples[ix1].count;

    #ifdef DEBUG
      Serial.print(">cnt1: ");
      Serial.printf("%d\n", cnt1);
      Serial.print(">ix1: ");
      Serial.printf("%d\n", ix1);
      Serial.print(">lowPot1: ");
      Serial.printf("%d\n", lowPot1);
      Serial.print(">highPot1: ");
      Serial.printf("%d\n", highPot1);
    #endif

    if (cnt1<SAMPLE_MEANING_CNT){
      if (cnt1==0){
        true_cnt++;
      }
      #ifdef DEBUG
        Serial.print(">XY: ");
        Serial.printf("%d:%d|xy\n", pot1,pot2);
      #endif

      //sampling means whithin each range. Assuming we are on a local slope the meaning should still keep us on the line?
      samples[ix1].a = (samples[ix1].a*cnt1 + pot1)/(cnt1+1);
      samples[ix1].b = (samples[ix1].a*cnt1 + pot2)/(cnt1+1);
      samples[ix1].count += 1;
      //this is perfect because we get a sorted array by default enabling quick sampling afterwards.
    }

    //then compute coverage to check if we have a good characteristic, let's say every 5 samples?
    #ifdef DEBUG
      Serial.print(">sample_chk: ");
      Serial.printf("%d\n", sample_chk);
      Serial.print(">true_cnt: ");
      Serial.printf("%d\n", true_cnt);
    #endif
    if (sample_chk==SAMPLE_CHECK_FREQ){
      sample_chk=1; //reset sample check counter
      //check we have at least 50% of samples full and our mean is close to the mean value
      
      if (true_cnt>NSAMPLES || DEBUG){
        float meandelta = 0;
        unsigned int prev=-1;
        unsigned int cnt=1;
        for (unsigned int i =0; i<NSAMPLES; i++){
          if (samples[i].count!=0){
            meandelta+=i-prev;
            prev=i;
            cnt++;
          } else{
            //no prob just continue?
          }
        }
        //and add the end one too
        if (samples[NSAMPLES-1].count==0){
          meandelta+=NSAMPLES-prev;
        }
        meandelta/=cnt;
        #ifdef DEBUG
        Serial.print(">meandelta: ");
        Serial.printf("%.4lf\n", meandelta);
        #endif

        if (meandelta<1.3){ //this was a reliably achievable value during testing.
          //save the array.
          state=CALIBRATION_SAVE;
        }

      } else{
        //basic condition not met: continue sampling
      }
    } else{
      sample_chk+=1;
    }

  } else if (state==CALIBRATION_SAVE){
  
  } else {
    // ERROR state: blink LED ominously
    // MAKE IT POSSIBLE TO GO TO CALIBRATION FROM AN ERROR STATE?
    while (1) {
      digitalWrite(LED_BUILTIN, 1); delay(40);
      digitalWrite(LED_BUILTIN, 0); delay(20);
      digitalWrite(LED_BUILTIN, 1); delay(40);
      digitalWrite(LED_BUILTIN, 0); delay(500);
    }
  }
}