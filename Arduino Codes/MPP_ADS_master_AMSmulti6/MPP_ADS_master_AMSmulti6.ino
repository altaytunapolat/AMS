
#include <Arduino.h>


const uint8_t DE_PIN = 9;
const uint8_t RE_PIN = 10;
#define RS485_SERIAL Serial1


const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE   = 0x55;


const uint8_t CMD_SYNC_TRIGGER = 0x10;  


static inline void enterTX(){ digitalWrite(RE_PIN,HIGH); digitalWrite(DE_PIN,HIGH); delayMicroseconds(10); }
static inline void enterRX(){ digitalWrite(DE_PIN,LOW);  digitalWrite(RE_PIN,LOW);  delayMicroseconds(10); }


static const uint8_t  NUM_SLAVES   = 6;        
static const uint32_t BAUD         = 115200;   
static const uint8_t  FRAME_BYTES  = 17;       


static const uint32_t FRAME_BITS       = (uint32_t)FRAME_BYTES * 10;                 
static const uint32_t FRAME_US         = (FRAME_BITS * 1000000UL) / BAUD;            
static const uint32_t PER_SLOT_GUARD_US= 1000;                                        
static const uint32_t SLOT_US          = FRAME_US + PER_SLOT_GUARD_US;               


static const uint16_t COLLECT_MARGIN_MS = 22;   
static const uint16_t IDLE_GAP_MS       = 5;    
static const uint16_t MAX_IDLE_WAIT_MS  = 50;   


static inline void sendFrameN(uint8_t addr, const uint8_t* data, uint8_t len){
  enterTX();
  uint8_t cks = addr ^ len; for(uint8_t i=0;i<len;i++) cks ^= data[i];
  RS485_SERIAL.write(START_BYTE);
  RS485_SERIAL.write(addr);
  RS485_SERIAL.write(len);
  RS485_SERIAL.write(data, len);
  RS485_SERIAL.write(cks);
  RS485_SERIAL.write(END_BYTE);
  RS485_SERIAL.flush();
  enterRX();
}

static bool waitIdleGap(uint16_t gap_ms, uint16_t max_wait_ms){
  unsigned long t_start = millis();
  for(;;){
    unsigned long t_idle = millis();
    while(millis() - t_idle < gap_ms){
      if(RS485_SERIAL.available()){ (void)RS485_SERIAL.read(); t_idle = millis(); } 
      if(millis() - t_start > max_wait_ms) return false;
    }
    return true; 
  }
}


enum RxState : uint8_t { RX_WAIT_START, RX_ADDR, RX_LEN, RX_PAYLOAD, RX_CKS, RX_END };

struct Parser {
  RxState  st = RX_WAIT_START;
  uint8_t  addr = 0;
  uint8_t  len  = 0;
  uint8_t  buf[32];
  uint8_t  idx  = 0;
  uint8_t  cks  = 0;

  
  uint8_t feed(uint8_t b){
    switch(st){
      case RX_WAIT_START:
        if(b == START_BYTE){ st = RX_ADDR; }
        return 0;

      case RX_ADDR:
        addr = b;
        st   = RX_LEN;
        return 0;

      case RX_LEN:
        len = b;
        if(len > sizeof(buf)){ st = RX_WAIT_START; return 2; }
        idx = 0;
        st  = RX_PAYLOAD;
        return 0;

      case RX_PAYLOAD:
        buf[idx++] = b;
        if(idx >= len) st = RX_CKS;
        return 0;

      case RX_CKS:
        cks = b;
        st  = RX_END;
        return 0;

      case RX_END: {
        uint8_t endb = b;
        
        uint8_t calc = addr ^ len;
        for(uint8_t i=0;i<len;i++) calc ^= buf[i];
        bool ok = (calc == cks) && (endb == END_BYTE);
        st = RX_WAIT_START;
        return ok ? 1 : 2;
      }
    }
    st = RX_WAIT_START; return 2;
  }
};

struct Sample {
  bool     ok;
  float    uT;
  int32_t  phase_err_us;
  uint32_t seq;
};

void setup(){
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  enterRX();

  Serial.begin(115200);
  RS485_SERIAL.begin(115200);
  delay(50);
  Serial.println("RS485 Master 6S (non-blocking parser)");
}

void loop(){
  static uint32_t seq = 1;

  
  waitIdleGap(IDLE_GAP_MS, MAX_IDLE_WAIT_MS);

  
  const uint16_t delay_ms = 4;
  uint8_t pl[7] = {
    CMD_SYNC_TRIGGER,
    (uint8_t)(delay_ms & 0xFF), (uint8_t)(delay_ms >> 8),
    (uint8_t)(seq & 0xFF), (uint8_t)((seq>>8)&0xFF),
    (uint8_t)((seq>>16)&0xFF), (uint8_t)((seq>>24)&0xFF)
  };
  sendFrameN(0, pl, sizeof(pl));

  
  delay(delay_ms + 4);

  
  const uint16_t all_slots_ms = (uint16_t)((SLOT_US * (uint32_t)NUM_SLAVES + 999UL)/1000UL);
  const uint16_t t_collect_ms = all_slots_ms + COLLECT_MARGIN_MS;

  Sample s[NUM_SLAVES + 1]; 
  for(uint8_t i=1;i<=NUM_SLAVES;i++){ s[i].ok=false; }
  Parser P;

  const unsigned long t0 = millis();
  while(millis() - t0 < t_collect_ms){
    while(RS485_SERIAL.available()){
      uint8_t b = (uint8_t)RS485_SERIAL.read();
      uint8_t r = P.feed(b);
      if(r == 1){ 
        if(P.len == 12 && P.addr >= 1 && P.addr <= NUM_SLAVES){
          float u; int32_t ph; uint32_t sq;
          memcpy(&u,  P.buf,    4);
          memcpy(&ph, P.buf+4,  4);
          memcpy(&sq, P.buf+8,  4);
          if(sq == seq){
            s[P.addr].ok  = true;
            s[P.addr].uT  = u;
            s[P.addr].phase_err_us = ph;
            s[P.addr].seq = sq;
          }
        }
      }
      
    }
    
  }

  
  waitIdleGap(IDLE_GAP_MS, MAX_IDLE_WAIT_MS);

  
  for(uint8_t i=1;i<=NUM_SLAVES;i++){
    if(i==1) Serial.print("S1: "); else { Serial.print(" | S"); Serial.print(i); Serial.print(": "); }
    if(s[i].ok){ Serial.print(s[i].uT,3); Serial.print("uT ph="); Serial.print(s[i].phase_err_us); }
    else        Serial.print("--");
  }
  Serial.println();

  seq++;
}
