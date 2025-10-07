// ===== RS-485 SLAVE: ADS1115 (A1-A3 only), synced sampling + auto-reply slots =====
#include <Arduino.h>
#include <Wire.h>
#include <ADS1115_WE.h>

// -------------------- CONFIG --------------------
const uint8_t MY_ADDRESS = 1;         // <-- set 1 or 2 per slave
const uint8_t DE_PIN = 9;
const uint8_t RE_PIN = 10;
#define RS485_SERIAL Serial1

// Framing
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE   = 0x55;

// Commands
const uint8_t CMD_BLINK        = 0x01;
const uint8_t CMD_SYNC_TRIGGER = 0x10;  // [0x10, delay_lo, delay_hi, seq0..seq3]
const uint8_t CMD_READ_LAST    = 0x11;  // (compat) reply 1 float + int32 + uint32 = 12B

// ADS1115
#define I2C_ADDRESS 0x48
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
bool adc_ok = false;

// Convert Volts -> µT  (adjust to your sensor)
const float VOLT_TO_uT = 35.0f;

// RS485 guard times for MAX485EPA+ (enable/disable to/from bus)
static inline void enterTransmit() { digitalWrite(RE_PIN, HIGH); digitalWrite(DE_PIN, HIGH); delayMicroseconds(10); }
static inline void enterReceive()  { digitalWrite(DE_PIN, LOW);  digitalWrite(RE_PIN, LOW);  delayMicroseconds(10); }

// -------------------- SYNC / STORAGE --------------------
volatile bool     sync_armed        = false;
volatile uint32_t sync_target_us    = 0;
volatile uint32_t sample_t_start_us = 0;
volatile int32_t  last_phase_err_us = 0;
volatile uint32_t last_seq          = 0;

volatile bool     have_sample       = false;
float             last_uT           = NAN;

// -------------------- PROTOTYPES --------------------
bool readByteWithTimeout(uint8_t &b, uint16_t tmo_ms);
void handleSyncTrigger(const uint8_t* buf, uint8_t len);
void takeMeasurementAndStore();
void sendStoredMeasurement();
void blink();

// -------------------- SETUP --------------------
void setup() {
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  enterReceive();

  // Serial ports
  Serial.begin(115200);
  RS485_SERIAL.begin(115200);      // you can raise to 250k / 500k if cabling is clean

  // I2C / ADC
  Wire.begin();
  Wire.setClock(400000);           // 400 kHz for faster reads

  adc_ok = adc.init();
  if (!adc_ok) {
    Serial.println("ADS1115 not connected!");
  } else {
    // Continuous conversion of A1-A3, fastest conversion rate
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);
    adc.setCompareChannels(ADS1115_COMP_1_3);
    adc.setConvRate(ADS1115_860_SPS);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    delay(3); // allow first conversion to complete
  }

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("Slave started, addr="); Serial.println(MY_ADDRESS);
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // 1) If a sync is armed and time reached: sample immediately
  if (sync_armed && (int32_t)(micros() - sync_target_us) >= 0) {
    sync_armed = false;
    takeMeasurementAndStore();
    last_phase_err_us = (int32_t)(sample_t_start_us - sync_target_us);

    // --- AUTO-REPLY IN A TIMESLOT (unsolicited) ---
    // OLD (too short):
    // delayMicroseconds(600 * MY_ADDRESS);
    
    // NEW (robust, computed from baud & frame size):
    const uint32_t BAUD = 115200;      // keep in sync with RS485_SERIAL.begin(...)
    const uint8_t  FRAME_BYTES = 17;   // AA, addr, len, 12B payload, cks, 55
    const uint32_t FRAME_BITS  = (uint32_t)FRAME_BYTES * 10;
    const uint32_t FRAME_US    = (FRAME_BITS * 1000000UL) / BAUD;  // ≈ 1480 µs @115200
    const uint32_t SLOT_US     = FRAME_US + 1000;                   // add 1.0 ms margin
    delayMicroseconds(SLOT_US * MY_ADDRESS);  // 1→~2.5 ms, 2→~5.0 ms at 115200

    sendStoredMeasurement();
  }

  // 2) Frame receiver (START, addr, len, payload..., checksum, END)
  while (RS485_SERIAL.available()) {
    int b = RS485_SERIAL.read();
    if (b != START_BYTE) continue;

    uint8_t addr=0, len=0;
    if (!readByteWithTimeout(addr, 20)) break;
    if (!readByteWithTimeout(len,  20)) break;
    if (len > 32) {
      // drop frame quickly
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

    // Address match or broadcast(0)
    if (addr == MY_ADDRESS || addr == 0) {
      const uint8_t cmd = buf[0];
      if      (cmd == CMD_BLINK)        blink();
      else if (cmd == CMD_SYNC_TRIGGER) handleSyncTrigger(buf, len);
      else if (cmd == CMD_READ_LAST)    sendStoredMeasurement(); // compatibility/manual read
    }

  end_frame:
    ;
  }
}

// -------------------- HELPERS --------------------
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

// CMD 0x10: [0x10, delay_lo, delay_hi, seq0..seq3]
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

// Continuous-mode read, timestamped at read start (closest to conversion-ready instant)
void takeMeasurementAndStore() {
  sample_t_start_us = micros();
  float v0 = adc_ok ? adc.getResult_V() : NAN;  // latest completed conversion
  last_uT = v0 * VOLT_TO_uT;
  have_sample = adc_ok && !isnan(v0);
  if (!have_sample) Serial.println("ADS1115 read failed (NaN).");
}

// Reply: float(µT) + int32(phase_err_us) + uint32(seq) = 12 bytes
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
