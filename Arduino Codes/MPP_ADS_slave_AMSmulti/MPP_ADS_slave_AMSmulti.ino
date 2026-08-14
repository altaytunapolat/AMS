
#include <Arduino.h>
#include <Wire.h>
#include <ADS1115_WE.h>


const uint8_t MY_ADDRESS = 6;         
const uint8_t DE_PIN = 9;
const uint8_t RE_PIN = 10;
#define RS485_SERIAL Serial1


const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE   = 0x55;


const uint8_t CMD_BLINK        = 0x01;
const uint8_t CMD_SYNC_TRIGGER = 0x10;  
const uint8_t CMD_READ_LAST    = 0x11;  


#define I2C_ADDRESS 0x48
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
bool adc_ok = false;


const float VOLT_TO_uT = 35.0f;


static inline void enterTransmit() { digitalWrite(RE_PIN, HIGH); digitalWrite(DE_PIN, HIGH); delayMicroseconds(10); }
static inline void enterReceive()  { digitalWrite(DE_PIN, LOW);  digitalWrite(RE_PIN, LOW);  delayMicroseconds(10); }


volatile bool     sync_armed        = false;
volatile uint32_t sync_target_us    = 0;
volatile uint32_t sample_t_start_us = 0;
volatile int32_t  last_phase_err_us = 0;
volatile uint32_t last_seq          = 0;

volatile bool     have_sample       = false;
float             last_uT           = NAN;


bool readByteWithTimeout(uint8_t &b, uint16_t tmo_ms);
void handleSyncTrigger(const uint8_t* buf, uint8_t len);
void takeMeasurementAndStore();
void sendStoredMeasurement();
void blink();


void setup() {
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  enterReceive();

  
  Serial.begin(115200);
  RS485_SERIAL.begin(115200);      

  
  Wire.begin();
  Wire.setClock(400000);           

  adc_ok = adc.init();
  if (!adc_ok) {
    Serial.println("ADS1115 not connected!");
  } else {
    
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);
    adc.setCompareChannels(ADS1115_COMP_2_3);
    adc.setConvRate(ADS1115_860_SPS);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    delay(3); 
  }

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("Slave started, addr="); Serial.println(MY_ADDRESS);
}


void loop() {
  
  if (sync_armed && (int32_t)(micros() - sync_target_us) >= 0) {
    sync_armed = false;
    takeMeasurementAndStore();
    last_phase_err_us = (int32_t)(sample_t_start_us - sync_target_us);

    
    
    
    
    
    const uint32_t BAUD = 115200;      
    const uint8_t  FRAME_BYTES = 17;   
    const uint32_t FRAME_BITS  = (uint32_t)FRAME_BYTES * 10;
    const uint32_t FRAME_US    = (FRAME_BITS * 1000000UL) / BAUD;  
    const uint32_t SLOT_US     = FRAME_US + 1000;                   
    delayMicroseconds(SLOT_US * MY_ADDRESS);  

    sendStoredMeasurement();
  }

  
  while (RS485_SERIAL.available()) {
    int b = RS485_SERIAL.read();
    if (b != START_BYTE) continue;

    uint8_t addr=0, len=0;
    if (!readByteWithTimeout(addr, 20)) break;
    if (!readByteWithTimeout(len,  20)) break;
    if (len > 32) {
      
      for (uint8_t i=0;i<len+2;i++){ uint8_t tmp; if(!readByteWithTimeout(tmp,2)) break; }
      continue;
    }

    uint8_t buf[32];
    for (uint8_t i=0; i<len; i++) {
      if (!readByteWithTimeout(buf[i], 20)) goto end_frame;
    }

    uint8_t checksum=0, endb=0;
    if (!readByteWithTimeout(checksum, 20)) goto end_frame;
    if (!readByteWithTimeout(endb,     20)) goto end_frame;

    uint8_t calc = addr ^ len;
    for (uint8_t i=0;i<len;i++) calc ^= buf[i];
    if (checksum != calc || endb != END_BYTE) goto end_frame;

    
    if (addr == MY_ADDRESS || addr == 0) {
      const uint8_t cmd = buf[0];
      if      (cmd == CMD_BLINK)        blink();
      else if (cmd == CMD_SYNC_TRIGGER) handleSyncTrigger(buf, len);
      else if (cmd == CMD_READ_LAST)    sendStoredMeasurement(); 
    }

  end_frame:
    ;
  }
}


bool readByteWithTimeout(uint8_t &b, uint16_t tmo_ms) {
  unsigned long t0 = millis();
  while (!RS485_SERIAL.available()) {
    if (millis() - t0 > tmo_ms) return false;
  }
  b = (uint8_t)RS485_SERIAL.read();
  return true;
}

void blink() {
  digitalWrite(LED_BUILTIN, HIGH); delay(80);
  digitalWrite(LED_BUILTIN, LOW);
}


void handleSyncTrigger(const uint8_t* buf, uint8_t len) {
  if (len < 7) return;
  uint16_t delay_ms = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
  uint32_t seq = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) |
                 ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);

  uint32_t rx_us = micros();
  sync_target_us = rx_us + (uint32_t)delay_ms * 1000UL;
  last_seq = seq;
  sync_armed = true;
}


void takeMeasurementAndStore() {
  sample_t_start_us = micros();
  float v0 = adc_ok ? adc.getResult_V() : NAN;  
  last_uT = v0 * VOLT_TO_uT;
  have_sample = adc_ok && !isnan(v0);
  if (!have_sample) Serial.println("ADS1115 read failed (NaN).");
}


void sendStoredMeasurement() {
  float    out   = last_uT;
  int32_t  phase = have_sample ? last_phase_err_us : 0;
  uint32_t seq   = last_seq;
  const uint8_t len = 12;

  enterTransmit();
  RS485_SERIAL.write(START_BYTE);
  RS485_SERIAL.write(MY_ADDRESS);
  RS485_SERIAL.write(len);
  RS485_SERIAL.write((uint8_t*)&out,   4);
  RS485_SERIAL.write((uint8_t*)&phase, 4);
  RS485_SERIAL.write((uint8_t*)&seq,   4);

  uint8_t checksum = MY_ADDRESS ^ len;
  const uint8_t* p = (const uint8_t*)&out;   for (uint8_t i=0;i<4;i++) checksum ^= p[i];
  const uint8_t* q = (const uint8_t*)&phase; for (uint8_t i=0;i<4;i++) checksum ^= q[i];
  const uint8_t* r = (const uint8_t*)&seq;   for (uint8_t i=0;i<4;i++) checksum ^= r[i];

  RS485_SERIAL.write(checksum);
  RS485_SERIAL.write(END_BYTE);
  RS485_SERIAL.flush();
  enterReceive();
}
