// ===== RS-485 MASTER: broadcast sync + collect 2 auto-replies (slots), A0-A3 only =====
#include <Arduino.h>

// Pins / Serial for RS485 transceiver
const uint8_t DE_PIN = 9;
const uint8_t RE_PIN = 10;
#define RS485_SERIAL Serial1

// Framing
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE   = 0x55;

// Commands
const uint8_t CMD_BLINK        = 0x01;
const uint8_t CMD_SYNC_TRIGGER = 0x10;  // [0x10, delay_lo, delay_hi, seq0..seq3]

// Guards for MAX485EPA+
static inline void enterTransmit() { digitalWrite(RE_PIN, HIGH); digitalWrite(DE_PIN, HIGH); delayMicroseconds(10); }
static inline void enterReceive()  { digitalWrite(DE_PIN, LOW);  digitalWrite(RE_PIN, LOW);  delayMicroseconds(10); }

// Sample struct (slave reply)
struct Sample {
  bool     ok;
  uint8_t  addr;
  float    uT;            // A0-A3 in µT
  int32_t  phase_err_us;
  uint32_t seq;
};

// Prototypes
void sendFrameN(uint8_t addr, const uint8_t* data, uint8_t len);
bool readFrame(uint8_t &addr, uint8_t &len, uint8_t *payload, uint8_t maxlen,
               uint16_t tmo_start_ms, uint16_t tmo_mid_ms);
bool readOneSample(Sample &s, uint32_t expect_seq, uint16_t tmo_ms);

void setup() {
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  enterReceive();

  Serial.begin(115200);
  RS485_SERIAL.begin(115200);       // can raise to 250k/500k if wiring is short/good
  delay(100);
  Serial.println("RS485 Master (sync slots) started");
}

void loop() {
  static uint32_t seq = 1;

  // === 1) Broadcast SYNC (short delay; ADS1115 @ 860SPS ≈1.16ms → 4ms is safe) ===
  const uint16_t delay_ms = 4;
  uint8_t payload[7] = {
    CMD_SYNC_TRIGGER,
    (uint8_t)(delay_ms & 0xFF),
    (uint8_t)(delay_ms >> 8),
    (uint8_t)(seq & 0xFF),
    (uint8_t)((seq >> 8) & 0xFF),
    (uint8_t)((seq >> 16) & 0xFF),
    (uint8_t)((seq >> 24) & 0xFF)
  };
  sendFrameN(0, payload, sizeof(payload));  // address 0 = broadcast
  // Serial.print("Sync in "); Serial.print(delay_ms); Serial.print(" ms, seq="); Serial.println(seq);

  // === 2) Wait just enough for sampling to complete ===
  delay(delay_ms + 4);  // 4ms margin

  // === 3) Collect two unsolicited replies (slots) ===
  Sample s1, s2;
  s1.ok = s2.ok = false;

  // We expect both frames within ~5 ms each; set a combined window
  const uint32_t t_collect_ms = 25;
  uint32_t t0 = millis();
  while (millis() - t0 < t_collect_ms && !(s1.ok && s2.ok)) {
    Sample s;
    if (readOneSample(s, seq, 5)) {
      if (s.addr == 1) s1 = s;
      else if (s.addr == 2) s2 = s;
    }
  }

  // === 4) Print compact line ===
  if (s1.ok)
    Serial.print("S1: "); else Serial.print("S1: -- ");
  if (s1.ok) {
    Serial.print(s1.uT, 3); Serial.print("uT ");
    Serial.print("ph="); Serial.print(s1.phase_err_us);
    Serial.print(" ");
  }

  if (s2.ok)
    Serial.print(" | S2: "); else Serial.print(" | S2: -- ");
  if (s2.ok) {
    Serial.print(s2.uT, 3); Serial.print("uT ");
    Serial.print("ph="); Serial.print(s2.phase_err_us);
  }
  Serial.println();

  seq++;
  // no big delay here → high update rate; add a tiny pause if needed
  // delay(2);
}

// ===== helpers =====
void sendFrameN(uint8_t addr, const uint8_t* data, uint8_t len) {
  enterTransmit();
  uint8_t checksum = addr ^ len;
  for (uint8_t i=0;i<len;i++) checksum ^= data[i];
  RS485_SERIAL.write(START_BYTE);
  RS485_SERIAL.write(addr);
  RS485_SERIAL.write(len);
  RS485_SERIAL.write(data, len);
  RS485_SERIAL.write(checksum);
  RS485_SERIAL.write(END_BYTE);
  RS485_SERIAL.flush();
  enterReceive();
}

bool readFrame(uint8_t &addr, uint8_t &len, uint8_t *payload, uint8_t maxlen,
               uint16_t tmo_start_ms, uint16_t tmo_mid_ms) {
  const unsigned long t_start = millis();

  // Sync to START
  while (true) {
    if (RS485_SERIAL.available()) {
      int b = RS485_SERIAL.read();
      if (b == START_BYTE) break;
    }
    if (millis() - t_start > tmo_start_ms) return false;
  }

  // Read addr + len
  unsigned long t_mid = millis();
  while (RS485_SERIAL.available() < 2) {
    if (millis() - t_mid > tmo_mid_ms) return false;
  }
  addr = RS485_SERIAL.read();
  len  = RS485_SERIAL.read();
  if (len > maxlen) return false;

  // Read payload
  for (uint8_t i=0; i<len; i++) {
    unsigned long t0 = millis();
    while (!RS485_SERIAL.available()) {
      if (millis() - t0 > tmo_mid_ms) return false;
    }
    payload[i] = RS485_SERIAL.read();
  }

  // Read checksum + END
  t_mid = millis();
  while (RS485_SERIAL.available() < 2) {
    if (millis() - t_mid > tmo_mid_ms) return false;
  }
  uint8_t checksum = RS485_SERIAL.read();
  uint8_t end      = RS485_SERIAL.read();

  uint8_t calc = addr ^ len;
  for (uint8_t i=0;i<len;i++) calc ^= payload[i];

  return (checksum == calc && end == END_BYTE);
}

// Expect 12B payload: float µT (4) + int32 phase (4) + uint32 seq (4)
bool readOneSample(Sample &s, uint32_t expect_seq, uint16_t tmo_ms) {
  uint8_t rx_addr=0, len=0;
  uint8_t buf[20];
  s.ok=false; s.addr=0; s.uT=NAN; s.phase_err_us=0; s.seq=0;

  if (!readFrame(rx_addr, len, buf, sizeof(buf), tmo_ms, tmo_ms)) return false;
  if (len != 12) return false;

  s.addr = rx_addr;
  memcpy(&s.uT,           buf,   4);
  memcpy(&s.phase_err_us, buf+4, 4);
  memcpy(&s.seq,          buf+8, 4);

  if (expect_seq && s.seq != expect_seq) return false;

  s.ok = true;
  return true;
}
