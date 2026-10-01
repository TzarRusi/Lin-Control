#include <Arduino.h>
#include <HardwareTimer.h>
#include <string.h>
#include <stdlib.h>

// STM32F103C8T6 Blue Pill, HSE=8 MHz. Native USB CDC for host commands.
// USART2 is owned directly here: PA2=TX, PA3=RX. PB0 controls TJA1021 SLP_N.
// Do not create a HardwareSerial instance for USART2.
constexpr uint32_t SLP_PIN = PB0;
constexpr uint32_t RX_PIN = PA3;
constexpr uint32_t TX_PIN = PA2;
constexpr uint32_t POWER_PIN = PB12; // PS_ON: open drain, released = OFF request.
HardwareTimer powerTimer(TIM3);
volatile bool powerRequested = false;
volatile uint32_t powerLeaseTicks = 0;
constexpr uint32_t ERROR_MASK = USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE;
volatile bool enabled = false;
uint32_t linBaud = 19200;
char command[64];
size_t commandLength = 0;
bool commandOverflow = false;
uint8_t capture[64];
size_t captureCount = 0;
bool captureOverflow = false;
uint32_t uartErrors = 0;
uint32_t transactionCounter = 0;
uint32_t lastHostMs = 0;

// One-shot queue. Payload CRC/counter are supplied explicitly by the host.
// No inferred table or motor command is built into the firmware.
uint8_t sequenceFrames[16][11];
volatile uint8_t sequenceCount = 0;
uint8_t sequenceRaw[16][64];
uint8_t sequenceRawCount[16];
uint32_t sequenceStartUs[16];
uint32_t sequenceErrors[16];
bool sequenceOverflow[16];

// Finite cycle scheduler, separate from the legacy V8 one-shot queue.
uint8_t cycleFrames[32][11];
volatile uint8_t cycleCount = 0;
bool cycleTiming = false;
uint8_t cycleCheckStreak = 0;
struct CycleRecord {
  uint32_t startUs;
  uint32_t errors;
  uint8_t kind, id, length;
  bool overflow;
  uint8_t raw[16];
};
CycleRecord cycleRecords[120];

// Read over SWD to verify execution before a native USB cable is connected.
extern "C" {
volatile uint32_t bridge_magic = 0;
volatile uint32_t bridge_heartbeat = 0;
volatile uint32_t bridge_cpu_hz = 0;
volatile uint32_t bridge_usart_brr = 0;
}

constexpr uint8_t protectedId(uint8_t id) {
  return id | (((id ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 1) << 6)
    | (((~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5))) & 1) << 7);
}
static_assert(protectedId(0x00) == 0x80, "PID 00");
static_assert(protectedId(0x01) == 0xC1, "PID 01");
static_assert(protectedId(0x3C) == 0x3C, "PID 3C");
static_assert(protectedId(0x3D) == 0x7D, "PID 3D");

constexpr uint8_t linChecksum(const uint8_t* data, size_t count, uint8_t seed) {
  uint16_t sum = seed;
  for (size_t i = 0; i < count; ++i) {
    sum += data[i];
    if (sum > 255) sum -= 255;
  }
  return (uint8_t)(sum ^ 255);
}
constexpr uint8_t productIdRequest[] = {0x7F,0x06,0xB2,0x00,0xFF,0x7F,0xFF,0xFF};
static_assert(linChecksum(productIdRequest, 8, 0) == 0x48, "Read identifier checksum");
constexpr uint8_t knownStatus[] = {0xD9,0x00,0x00,0xFE,0xFF,0xFF,0xFF,0xFF};
static_assert(linChecksum(knownStatus, 8, 0x5E) == 0xC8, "Captured enhanced checksum");

int hexNibble(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

int parseHexByte(const char* input) {
  int high = hexNibble(input[0]);
  int low = hexNibble(input[1]);
  return high < 0 || low < 0 ? -1 : (high << 4) | low;
}

void disableLin() {
  digitalWrite(SLP_PIN, LOW);
  enabled = false;
  sequenceCount = 0;
  cycleCount = 0;
}

// Called by the 1 kHz interrupt as well as normal code. No USB or delays here.
void powerOff() {
  GPIOB->BSRR = GPIO_PIN_12; // Release the open-drain output first.
  GPIOB->BRR = GPIO_PIN_0;
  powerRequested = false;
  powerLeaseTicks = 0;
  enabled = false;
  sequenceCount = 0;
  cycleCount = 0;
}

void powerTick() {
  if (powerRequested && (powerLeaseTicks == 0 || --powerLeaseTicks == 0)) powerOff();
}

// A validated cycle can produce enough USB report text to outlive the normal
// three-second host lease. Renew only while emitting that report; if USB stops
// draining a line for three seconds, the timer interrupt still cuts the PSU.
void renewPowerLeaseForCycleReport() {
  if (!Serial.dtr()) return;
  noInterrupts();
  bool renew = powerRequested;
  if (renew) powerLeaseTicks = 3000;
  interrupts();
  if (renew) lastHostMs = millis();
}

void printPower() {
  Serial.print(F("POWER REQUEST=")); Serial.print(powerRequested ? 1 : 0);
  Serial.print(F(" LEASE_MS=")); Serial.print(powerLeaseTicks);
  Serial.println(F(" PIN=PB12 MODE=OPEN_DRAIN VOLTAGE_VERIFIED=0"));
}

void hexByte(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

void collect(bool keep) {
  // Reading SR then DR clears RXNE and PE/FE/NE/ORE on STM32F1.
  uint32_t sr = USART2->SR;
  if (sr & (USART_SR_RXNE | ERROR_MASK)) {
    uint8_t b = (uint8_t)USART2->DR;
    if (keep) {
      uartErrors |= sr & ERROR_MASK;
      if (sr & USART_SR_RXNE) {
        if (captureCount < sizeof(capture)) capture[captureCount++] = b;
        else captureOverflow = true;
      }
    }
  }
}

void clearCapture() {
  collect(false);
  captureCount = 0;
  captureOverflow = false;
  uartErrors = 0;
}

void configureUart() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  USART2->CR1 = 0;
  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);
  USART2->CR2 = USART_CR2_LINEN; // 8N1, 13-bit hardware Break.
  USART2->CR3 = 0;             // No DMA/IRQ/flow control; service DR by polling.
  USART2->BRR = (HAL_RCC_GetPCLK1Freq() + linBaud / 2) / linBaud;
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  bridge_usart_brr = USART2->BRR;
  clearCapture();
}

bool waitFlag(uint32_t flag, bool keep) {
  uint32_t start = micros();
  if (cycleTiming && (!powerRequested || !Serial.dtr() || Serial.available())) return false;
  while (!(USART2->SR & flag)) {
    collect(keep);
    if (cycleTiming && (!powerRequested || !Serial.dtr() || Serial.available())) return false;
    if ((uint32_t)(micros() - start) > 10000) {
      powerOff();
      if (!cycleTiming) Serial.println(F("ERR UART_TIMEOUT DISABLED"));
      return false;
    }
  }
  collect(keep);
  return true;
}

bool sendByte(uint8_t b) {
  if (!waitFlag(USART_SR_TXE, true)) return false;
  USART2->DR = b;
  return waitFlag(USART_SR_TC, true);
}

bool sendBreak() {
  if (!waitFlag(USART_SR_TC, false)) return false;
  USART2->SR = ~USART_SR_TC;
  USART2->CR1 |= USART_CR1_SBK;
  if (!waitFlag(USART_SR_TC, false)) return false;
  delayMicroseconds((1000000UL + linBaud - 1) / linBaud);
  clearCapture(); // A Break FE/zero is expected; discard only before SYNC.
  return true;
}

void printInfo() {
  Serial.print(F("INFO VERSION=STM32_LIN_10 USB=CDC LIN=")); Serial.print(linBaud);
  Serial.print(F(" ENABLE=")); Serial.print(enabled ? 1 : 0);
  Serial.print(F(" RX=")); Serial.print(digitalRead(RX_PIN));
  Serial.print(F(" CPU=")); Serial.print(SystemCoreClock);
  Serial.print(F(" PCLK1=")); Serial.print(HAL_RCC_GetPCLK1Freq());
  Serial.print(F(" BRR=")); Serial.print(USART2->BRR);
  Serial.print(F(" TX_PIN=PA2 RX_PIN=PA3 SLP_PIN=PB0 POWER_REQUEST=")); Serial.println(powerRequested ? 1 : 0);
}

bool readyToTransmit() {
  if (!enabled) { Serial.println(F("ERR DISABLED")); return false; }
  if (!powerRequested) { Serial.println(F("ERR POWER_OFF")); return false; }
  if (digitalRead(RX_PIN) == LOW) { powerOff(); Serial.println(F("ERR RX_LOW")); return false; }
  clearCapture();
  ++transactionCounter;
  return true;
}

void reportCapture(const uint8_t* tx, size_t txSize) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < 30000) collect(true);
  bool echoMatches = captureCount >= txSize;
  for (size_t i = 0; i < txSize && echoMatches; ++i) {
    if (capture[i] != tx[i]) echoMatches = false;
  }
  // No USB logging during USART timing/capture.
  if (!echoMatches || uartErrors || captureOverflow) powerOff();
  Serial.print(F("RESULT TX="));
  for (size_t i = 0; i < txSize; ++i) hexByte(tx[i]);
  Serial.print(F(" RAW="));
  for (size_t i = 0; i < captureCount; ++i) hexByte(capture[i]);
  Serial.print(F(" COUNT=")); Serial.print(captureCount);
  Serial.print(F(" ECHO=")); Serial.print(echoMatches ? F("OK") : F("MISMATCH"));
  Serial.print(F(" AFTER_ECHO="));
  if (echoMatches) {
    for (size_t i = txSize; i < captureCount; ++i) hexByte(capture[i]);
  } else Serial.print(F("UNKNOWN"));
  Serial.print(F(" CAPTURE_OVERFLOW=")); Serial.print(captureOverflow ? 1 : 0);
  Serial.print(F(" UART_ERRORS=")); Serial.print(uartErrors, HEX);
  Serial.print(F(" SEQ=")); Serial.println(transactionCounter);
}

// During a burst any received USB byte requests cancellation. There is no
// blocking USB output until LIN is disabled. Maximum burst: 16 * 100 ms.
// Use dtr(): STM32 core 3.0.0 operator bool() includes delay(10).
void runSequence(uint8_t periodMs) {
  if (!enabled) { Serial.println(F("ERR DISABLED")); return; }
  if (linBaud != 19200) { Serial.println(F("ERR SEQ_BAUD")); return; }
  if (!sequenceCount) { Serial.println(F("ERR SEQ_EMPTY")); return; }
  if (!powerRequested) { Serial.println(F("ERR POWER_OFF")); return; }
  uint8_t planned = sequenceCount;
  sequenceCount = 0; // Consumed even if interrupted; never auto-replay.
  uint8_t sent = 0;
  const char* outcome = "OK";
  uint32_t startUs = micros();
  const uint32_t periodUs = (uint32_t)periodMs * 1000;
  for (uint8_t step = 0; step < planned; ++step) {
    uint32_t due = (uint32_t)step * periodUs;
    while ((uint32_t)(micros() - startUs) < due) {
      if (!powerRequested || !Serial.dtr() || Serial.available()) { outcome = "CANCELLED"; break; }
    }
    if (strcmp(outcome, "OK")) break;
    if (!powerRequested || !Serial.dtr() || Serial.available()) { outcome = "CANCELLED"; break; }
    if ((uint32_t)(micros() - startUs) > due + 2000) { outcome = "LATE"; break; }
    if (digitalRead(RX_PIN) == LOW) { outcome = "RX_LOW"; break; }
    sequenceStartUs[step] = micros() - startUs;
    clearCapture();
    ++transactionCounter;
    bool transmitted = sendBreak();
    for (uint8_t i = 0; i < 11 && transmitted; ++i) transmitted = sendByte(sequenceFrames[step][i]);
    uint32_t captureStart = micros();
    while (transmitted && (uint32_t)(micros() - captureStart) < 2000) {
      collect(true);
      if (!powerRequested || !Serial.dtr() || Serial.available()) { outcome = "CANCELLED"; break; }
    }
    sequenceRawCount[step] = (uint8_t)captureCount;
    memcpy(sequenceRaw[step], capture, captureCount);
    sequenceErrors[step] = uartErrors;
    sequenceOverflow[step] = captureOverflow;
    ++sent;
    if (!transmitted) { outcome = "UART_TIMEOUT"; break; }
    if (strcmp(outcome, "OK")) break;
    // Subscriber frames must produce exactly our echo, no competing publisher.
    if (uartErrors || captureOverflow || captureCount != 11 || memcmp(capture, sequenceFrames[step], 11)) {
      outcome = "CAPTURE_ERROR"; break;
    }
  }
  disableLin();
  if (strcmp(outcome, "OK")) powerOff();
  while (Serial.available()) Serial.read();
  commandLength = 0;
  commandOverflow = false;
  if (!Serial) return;
  for (uint8_t step = 0; step < sent; ++step) {
    Serial.print(F("SEQ_FRAME INDEX=")); Serial.print(step);
    Serial.print(F(" START_US=")); Serial.print(sequenceStartUs[step]);
    Serial.print(F(" TX="));
    for (uint8_t b : sequenceFrames[step]) hexByte(b);
    Serial.print(F(" RAW="));
    for (uint8_t i = 0; i < sequenceRawCount[step]; ++i) hexByte(sequenceRaw[step][i]);
    Serial.print(F(" UART_ERRORS=")); Serial.print(sequenceErrors[step], HEX);
    Serial.print(F(" OVERFLOW=")); Serial.println(sequenceOverflow[step] ? 1 : 0);
  }
  Serial.print(F("SEQ_DONE SENT=")); Serial.print(sent);
  Serial.print(F(" PLANNED=")); Serial.print(planned);
  Serial.print(F(" RESULT=")); Serial.print(outcome);
  Serial.println(F(" ENABLE=0"));
}

constexpr uint8_t cycleCrc(const uint8_t* data, size_t length) {
  uint8_t crc = 0xFF;
  for (size_t i=0; i<length; ++i) {
    crc ^= data[i];
    for (uint8_t bit=0; bit<8; ++bit) crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x2F : 0));
  }
  return crc ^ 0xFF;
}
constexpr uint8_t cycleCheckVector[] = {'1','2','3','4','5','6','7','8','9'};
static_assert(cycleCrc(cycleCheckVector,9) == 0xDF, "Cycle CRC reference vector");
bool cycleStatusValid(uint8_t id, const uint8_t* data, size_t length, uint32_t errors, bool overflow) {
  if (length != 9 || errors || overflow) return false;
  static const uint8_t table14[] = {46,212,127,42,208,123,38,204,119,34,200,115,30,196,111,26};
  static const uint8_t table1e[] = {15,222,178,134,90,46,253,209,165,121,77,33,240,196,152,108};
  static const uint8_t table32[] = {3,232,210,188,166,144,122,100,78,56,34,12,241,219,197,175};
  const uint8_t* table = nullptr; uint8_t pdu=0;
  if (id==0x14) { table=table14; pdu=6; }
  else if (id==0x1E) { table=table1e; pdu=4; }
  else if (id==0x32) { table=table32; pdu=3; }
  else return false;
  if (linChecksum(data,8,protectedId(id)) != data[8]) return false;
  uint8_t input[6];
  for (uint8_t i=1;i<pdu;++i) input[i-1]=data[i];
  input[pdu-1]=table[data[1]&15];
  return cycleCrc(input,pdu)==data[0];
}
bool cycleObserveLeft(bool left14, bool left32, uint8_t& streak) {
  if (left14 && left32) { if (streak<3) ++streak; } else streak=0;
  return streak>=3;
}
const char* cycleCancelReason() {
  if (!powerRequested) return "POWER_LOST";
  if (!Serial.dtr() || Serial.available()) return "CANCELLED";
  return nullptr;
}
void runCycle(uint8_t periodMs, uint8_t cycles, bool monitor) {
  uint32_t duration=(uint32_t)periodMs*cycles;
  if ((periodMs!=100 && periodMs!=200) || !cycles || duration>2400 || cycles>24) { Serial.println(F("ERR CYCLE_PERIOD")); return; }
  if (!enabled) { Serial.println(F("ERR DISABLED")); return; }
  if (!powerRequested) { Serial.println(F("ERR POWER_OFF")); return; }
  if (linBaud!=19200) { Serial.println(F("ERR CYCLE_BAUD")); return; }
  if (!monitor && cycleCount!=32) { Serial.println(F("ERR CYCLE_TABLE")); return; }
  if (powerLeaseTicks < duration+500) { powerOff(); Serial.println(F("ERR CYCLE_LEASE")); return; }
  const uint8_t expected=cycles*5;
  const uint32_t slotUs=(uint32_t)periodMs*1000/5;
  static const uint8_t slotIds[]={0x0A,0x28,0x14,0x32,0x1E};
  uint8_t sent=0, streak=0;
  bool left14=false;
  const char* outcome="OK";
  cycleCount=0; // Consumed before transmission; never automatically replayed.
  cycleTiming=true;
  const uint32_t start=micros();
  for (uint8_t index=0;index<expected;++index) {
    uint32_t due=(uint32_t)index*slotUs;
    const char* cancelled=nullptr;
    while ((uint32_t)(micros()-start)<due) {
      cancelled=cycleCancelReason(); if(cancelled) break;
    }
    if (!cancelled) cancelled=cycleCancelReason();
    if (cancelled) { outcome=cancelled; break; }
    if ((uint32_t)(micros()-start)>due+2000) { outcome="LATE"; break; }
    if (digitalRead(RX_PIN)==LOW) { outcome="RX_LOW"; break; }
    uint8_t slot=index%5;
    if (!slot) left14=false;
    CycleRecord& record=cycleRecords[index];
    memset(&record,0,sizeof(record));
    record.startUs=micros()-start; record.id=slotIds[slot];
    record.kind=(slot<2) ? (monitor?0:1) : 2;
    clearCapture();
    bool transmitted=true;
    if (record.kind) {
      ++transactionCounter;
      transmitted=sendBreak();
      if (record.kind==1) {
        const uint8_t* frame=cycleFrames[((index/5)%16)*2+slot];
        for(uint8_t b=0;b<11 && transmitted;++b) transmitted=sendByte(frame[b]);
      } else {
        if(transmitted) transmitted=sendByte(0x55);
        if(transmitted) transmitted=sendByte(protectedId(record.id));
      }
    }
    while (transmitted && (uint32_t)(micros()-start)<due+slotUs-2000) {
      collect(true);
      cancelled=cycleCancelReason(); if(cancelled) break;
    }
    collect(true);
    record.errors=uartErrors;
    record.overflow=captureOverflow || captureCount>sizeof(record.raw);
    record.length=(uint8_t)(captureCount<sizeof(record.raw)?captureCount:sizeof(record.raw));
    memcpy(record.raw,capture,record.length);
    ++sent;
    if(cancelled) { outcome=cancelled; break; }
    if(!transmitted) { outcome="UART_TIMEOUT"; break; }
    if ((uint32_t)(micros()-start)>due+slotUs) { outcome="SLOT_OVERRUN"; break; }
    if(record.errors || record.overflow) { outcome="CAPTURE_ERROR"; break; }
    if (!record.kind) {
      if(captureCount) { outcome="IDLE_ACTIVITY"; break; }
    } else if (record.kind==1) {
      if(captureCount!=11 || memcmp(capture,cycleFrames[((index/5)%16)*2+slot],11)) { outcome="CAPTURE_ERROR"; break; }
    } else {
      if(captureCount!=11 || capture[0]!=0x55 || capture[1]!=protectedId(record.id)) { outcome="STATUS_CAPTURE"; break; }
      if(!cycleStatusValid(record.id,capture+2,9,uartErrors,captureOverflow)) { outcome="STATUS_CRC"; break; }
      if(slot==2) left14=capture[7]==0xE2;
      if(slot==3 && cycleObserveLeft(left14,(capture[3]&0xF0)==0xB0,streak)) { outcome="LEFT_SIGNATURE"; powerOff(); break; }
    }
  }
  disableLin();
  if(strcmp(outcome,"OK")) powerOff();
  cycleTiming=false;
  while(Serial.available()) Serial.read();
  commandLength=0; commandOverflow=false;
  if(!Serial.dtr()) return;
  renewPowerLeaseForCycleReport();
  for(uint8_t i=0;i<sent;++i) {
    const CycleRecord& r=cycleRecords[i];
    Serial.print(F("CYCLE_SLOT INDEX="));Serial.print(i);
    Serial.print(F(" START_US="));Serial.print(r.startUs);
    Serial.print(F(" TYPE="));Serial.print(r.kind==0?"IDLE":(r.kind==1?"COMMAND":"STATUS"));
    Serial.print(F(" ID="));hexByte(r.id);Serial.print(F(" TX="));
    if(r.kind==1) for(uint8_t b:cycleFrames[((i/5)%16)*2+i%5]) hexByte(b);
    else if(r.kind==2) { hexByte(0x55);hexByte(protectedId(r.id)); }
    Serial.print(F(" RAW="));for(uint8_t b=0;b<r.length;++b) hexByte(r.raw[b]);
    Serial.print(F(" UART_ERRORS="));Serial.print(r.errors,HEX);
    Serial.print(F(" OVERFLOW="));Serial.println(r.overflow?1:0);
    renewPowerLeaseForCycleReport();
  }
  renewPowerLeaseForCycleReport();
  Serial.print(F("CYCLE_DONE SLOTS="));Serial.print(sent);
  Serial.print(F(" EXPECTED="));Serial.print(expected);
  Serial.print(F(" RESULT="));Serial.print(outcome);
  Serial.print(F(" ENABLE=0 LEFT_STREAK="));Serial.print(streak);
  Serial.print(F(" POWER_REQUEST="));Serial.println(powerRequested?1:0);
  renewPowerLeaseForCycleReport();
}

void executeCommand() {
  lastHostMs = millis();
  if (!strncmp(command,"CYCLECHECK ",11)) {
    if(powerRequested || enabled) { Serial.println(F("ERR CHECK_POWER"));return; }
    if(strlen(command)!=35 || command[13]!=' ' || command[32]!=' ') { Serial.println(F("ERR CHECK_FORMAT"));return; }
    int id=parseHexByte(command+11), errors=parseHexByte(command+33);uint8_t data[9];
    if(id<0 || errors<0) { Serial.println(F("ERR CHECK_FORMAT"));return; }
    for(uint8_t i=0;i<9;++i) { int b=parseHexByte(command+14+2*i);if(b<0) {Serial.println(F("ERR CHECK_FORMAT"));return;}data[i]=(uint8_t)b; }
    Serial.print(F("CYCLECHECK VALID="));Serial.println(cycleStatusValid((uint8_t)id,data,9,(uint32_t)errors,false)?1:0);return;
  }
  if (!strncmp(command,"CYCLEPAIR ",10)) {
    if(powerRequested || enabled) {Serial.println(F("ERR CHECK_POWER"));return;}
    if(!strcmp(command,"CYCLEPAIR RESET")) {cycleCheckStreak=0;Serial.println(F("OK CYCLEPAIR_RESET"));return;}
    if(strlen(command)!=47 || command[28]!=' ') {Serial.println(F("ERR CHECK_FORMAT"));return;}
    uint8_t a[9],b[9];
    for(uint8_t i=0;i<9;++i) {int x=parseHexByte(command+10+2*i), y=parseHexByte(command+29+2*i);if(x<0||y<0) {Serial.println(F("ERR CHECK_FORMAT"));return;}a[i]=x;b[i]=y;}
    if(!cycleStatusValid(0x14,a,9,0,false)||!cycleStatusValid(0x32,b,9,0,false)) {cycleCheckStreak=0;Serial.println(F("ERR CHECK_CRC"));return;}
    bool stop=cycleObserveLeft(a[5]==0xE2,(b[1]&0xF0)==0xB0,cycleCheckStreak);
    Serial.print(F("CYCLEPAIR STREAK="));Serial.print(cycleCheckStreak);Serial.print(F(" STOP="));Serial.println(stop?1:0);return;
  }
  if (!strcmp(command,"CYCLECLR")) {cycleCount=0;Serial.println(F("OK CYCLE_CLEARED"));return;}
  if (!strncmp(command,"CYCLEADD ",9)) {
    if(!enabled) {Serial.println(F("ERR DISABLED"));return;}
    if(strlen(command)!=28 || command[11]!=' ') {Serial.println(F("ERR CYCLE_FORMAT"));return;}
    if(cycleCount>=32) {Serial.println(F("ERR CYCLE_FULL"));return;}
    int id=parseHexByte(command+9);
    if(id!=(cycleCount%2?0x28:0x0A)) {Serial.println(F("ERR CYCLE_ID_ORDER"));return;}
    uint8_t frame[11]={0x55,protectedId((uint8_t)id)};
    for(uint8_t i=0;i<8;++i) {int b=parseHexByte(command+12+2*i);if(b<0) {Serial.println(F("ERR CYCLE_FORMAT"));return;}frame[i+2]=b;}
    frame[10]=linChecksum(frame+2,8,frame[1]);memcpy(cycleFrames[cycleCount++],frame,11);
    Serial.print(F("OK CYCLE_COUNT="));Serial.println(cycleCount);return;
  }
  if (!strncmp(command,"CYCLERUN ",9) || !strncmp(command,"MONITOR ",8)) {
    bool monitor=command[0]=='M';uint8_t offset=monitor?8:9;
    if(strlen(command)!=offset+5 || command[offset+2]!=' ') {Serial.println(F("ERR CYCLE_PERIOD"));return;}
    int period=parseHexByte(command+offset), cycles=parseHexByte(command+offset+3);
    if(period<0 || cycles<0) {Serial.println(F("ERR CYCLE_PERIOD"));return;}
    runCycle((uint8_t)period,(uint8_t)cycles,monitor);return;
  }
  if (!strcmp(command, "POWER 0") || !strcmp(command, "STOP")) {
    powerOff(); printPower(); return;
  }
  if (!strcmp(command, "POWER?")) { printPower(); return; }
  if (!strcmp(command, "POWER 1") || !strcmp(command, "POWER KEEP")) {
    bool turnOn = !strcmp(command, "POWER 1");
    bool accepted;
    noInterrupts();
    accepted = turnOn ? !powerRequested : powerRequested;
    if (accepted) {
      powerLeaseTicks = 3000;
      powerRequested = true;
      GPIOB->BRR = GPIO_PIN_12;
    }
    interrupts();
    if (!accepted) Serial.println(turnOn ? F("ERR POWER_ALREADY_ON") : F("ERR POWER_OFF"));
    else printPower();
    return;
  }
  if (!strcmp(command, "SEQCLR")) {
    sequenceCount = 0; Serial.println(F("OK SEQ_CLEARED")); return;
  }
  if (!strncmp(command, "SEQADD ", 7)) {
    if (strlen(command) != 26 || command[9] != ' ') { Serial.println(F("ERR SEQ_FORMAT")); return; }
    int id = parseHexByte(command + 7);
    if (id != 0x0A && id != 0x28) { Serial.println(F("ERR SEQ_ID")); return; }
    uint8_t frame[11] = {0x55, protectedId((uint8_t)id)};
    for (uint8_t i = 0; i < 8; ++i) {
      int b = parseHexByte(command + 10 + 2*i);
      if (b < 0) { Serial.println(F("ERR SEQ_FORMAT")); return; }
      frame[2+i] = (uint8_t)b;
    }
    if (!enabled) { Serial.println(F("ERR DISABLED")); return; }
    if (sequenceCount >= 16) { Serial.println(F("ERR SEQ_FULL")); return; }
    frame[10] = linChecksum(frame+2, 8, frame[1]);
    memcpy(sequenceFrames[sequenceCount++], frame, 11);
    Serial.print(F("OK SEQ_COUNT=")); Serial.println(sequenceCount); return;
  }
  if (!strncmp(command, "SEQRUN ", 7)) {
    if (strlen(command) != 9) { Serial.println(F("ERR SEQ_PERIOD")); return; }
    int periodMs = parseHexByte(command + 7);
    if (periodMs < 20 || periodMs > 100) { Serial.println(F("ERR SEQ_PERIOD")); return; }
    runSequence((uint8_t)periodMs); return;
  }
  if (!strcmp(command, "INFO")) { printInfo(); return; }
  if (!strcmp(command, "PING")) { Serial.println(F("PONG STM32_LIN_10")); return; }
  if (!strcmp(command, "HELP")) {
    Serial.println(F("POWER_CMDS POWER? POWER 0|1|KEEP STOP; POWER KEEP renews 3000ms lease; no voltage feedback"));
    Serial.println(F("CMDS INFO PING ENABLE 0|1 BAUD 9600|10400|19200 ECHO WAKE POLL hh IDENT nad id READ nad did DTC nad sub DIAG nad hex FRAME id hex SEQCLR SEQADD id 8bytes SEQRUN mm")); return;
  }
  if (!strcmp(command, "ENABLE 0") || !strcmp(command, "ENABLE 1")) {
    enabled = command[7] == '1';
    if (!enabled) disableLin(); // Explicit disable also invalidates queued frames.
    digitalWrite(SLP_PIN, enabled ? HIGH : LOW);
    delay(5); clearCapture(); printInfo(); return;
  }
  if (!strncmp(command, "BAUD ", 5)) {
    uint32_t requested;
    if (!strcmp(command + 5, "9600")) requested = 9600;
    else if (!strcmp(command + 5, "10400")) requested = 10400;
    else if (!strcmp(command + 5, "19200")) requested = 19200;
    else { Serial.println(F("ERR BAUD")); return; }
    if (enabled) { Serial.println(F("ERR DISABLE_FIRST")); return; }
    linBaud = requested; configureUart(); printInfo(); return;
  }
  if (!strcmp(command, "ECHO")) {
    // Detached actuator only; arbitrary test bytes are not LIN commands.
    if (!readyToTransmit()) return;
    const uint8_t bytes[] = {0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34};
    for (uint8_t b : bytes) if (!sendByte(b)) return;
    reportCapture(bytes, sizeof(bytes)); return;
  }
  if (!strcmp(command, "WAKE")) {
    if (!readyToTransmit()) return;
    USART2->CR1 &= ~USART_CR1_UE;
    digitalWrite(TX_PIN, HIGH); pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW); delayMicroseconds(300);
    digitalWrite(TX_PIN, HIGH); delayMicroseconds(300);
    configureUart(); delay(100); clearCapture();
    Serial.println(F("OK WAKE_SENT_NOT_VERIFIED")); return;
  }
  if (!strncmp(command, "IDENT ", 6)) {
    if (strlen(command) != 11 || command[8] != ' ') { Serial.println(F("ERR IDENT")); return; }
    int nad = parseHexByte(command + 6);
    int identifier = parseHexByte(command + 9);
    // LIN 2.2A Read by Identifier: product identity or serial number only.
    // Broadcast NAD is useful for the single detached actuator; 7E is not a config NAD.
    if (nad < 1 || nad > 0x7F || nad == 0x7E || identifier < 0 || identifier > 63 || (identifier > 1 && identifier < 32)) {
      Serial.println(F("ERR IDENT")); return;
    }
    uint8_t data[] = {(uint8_t)nad,0x06,0xB2,(uint8_t)identifier,0xFF,0x7F,0xFF,0xFF};
    sendDiagnosticRead(data); return;
  }
  if (!strncmp(command, "DIAG ", 5)) {
    size_t size = strlen(command);
    if (size < 10 || size > 20 || command[7] != ' ' || ((size-8) & 1)) { Serial.println(F("ERR DIAG")); return; }
    int nad = parseHexByte(command + 5);
    if (nad < 1 || nad > 0x7D) { Serial.println(F("ERR DIAG")); return; }
    uint8_t p[6] = {};
    size_t count = (size-8)/2;
    for (size_t i=0; i<count; ++i) {
      int b = parseHexByte(command+8+i*2);
      if (b < 0) { Serial.println(F("ERR DIAG")); return; }
      p[i] = (uint8_t)b;
    }
    // Narrow exploration only: reads, routine RESULTS, return IO control,
    // default/extended session and tester present. No programming or writes.
    bool allowed = (p[0] == 0x22 && count == 3)
      || (p[0] == 0x31 && count == 4 && p[1] == 3)
      || (p[0] == 0x2F && count == 4 && p[3] == 0)
      || (p[0] == 0x10 && count == 2 && (p[1] == 1 || p[1] == 3))
      || (p[0] == 0x3E && count == 2 && p[1] == 0)
      || ((p[0] == 0x21 || p[0] == 0x1A) && count == 2)
      // Incomplete requests cannot encode an IO operation, routine, or memory range.
      || ((p[0] == 0x2F || p[0] == 0x31 || p[0] == 0x23) && count == 1);
    if (!allowed) { Serial.println(F("ERR DIAG_SERVICE")); return; }
    uint8_t data[8] = {(uint8_t)nad,(uint8_t)count,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    memcpy(data+2,p,count);
    sendDiagnosticRead(data); return;
  }
  if (!strncmp(command, "DTC ", 4)) {
    if (strlen(command) != 9 || command[6] != ' ') { Serial.println(F("ERR DTC")); return; }
    int nad = parseHexByte(command + 4);
    int sub = parseHexByte(command + 7);
    // ReadDTCInformation only: count, current list, supported list. Never clear DTCs.
    if (nad < 1 || nad > 0x7D || (sub != 1 && sub != 2 && sub != 0x0A)) {
      Serial.println(F("ERR DTC")); return;
    }
    uint8_t data[] = {(uint8_t)nad,(uint8_t)(sub == 0x0A ? 2 : 3),0x19,(uint8_t)sub,0xFF,0xFF,0xFF,0xFF};
    sendDiagnosticRead(data); return;
  }
  if (!strncmp(command, "READ ", 5)) {
    if (strlen(command) != 12 || command[7] != ' ') { Serial.println(F("ERR READ")); return; }
    int nad = parseHexByte(command + 5);
    int didHigh = parseHexByte(command + 8);
    int didLow = parseHexByte(command + 10);
    // UDS ReadDataByIdentifier, physical NAD; no session/write/reset services.
    if (nad < 1 || nad > 0x7D || didHigh < 0 || didLow < 0) {
      Serial.println(F("ERR READ")); return;
    }
    uint8_t data[] = {(uint8_t)nad,0x03,0x22,(uint8_t)didHigh,(uint8_t)didLow,0xFF,0xFF,0xFF};
    sendDiagnosticRead(data); return;
  }
  if (!strncmp(command, "FRAME ", 6)) {
    size_t size = strlen(command);
    if (size < 11 || size > 25 || command[8] != ' ' || ((size-9)&1)) { Serial.println(F("ERR FRAME")); return; }
    int id = parseHexByte(command+6);
    // Do not publish reserved diagnostic IDs or the actuator's observed responses.
    if (id < 0 || id > 0x3B || id == 0x14 || id == 0x1E || id == 0x32) { Serial.println(F("ERR FRAME_ID")); return; }
    size_t count = (size-9)/2;
    uint8_t frame[11] = {0x55,protectedId((uint8_t)id)};
    for (size_t i=0; i<count; ++i) {
      int value = parseHexByte(command+9+i*2);
      if (value < 0) { Serial.println(F("ERR FRAME")); return; }
      frame[2+i] = (uint8_t)value;
    }
    frame[count+2] = linChecksum(frame+2,count,frame[1]);
    if (!readyToTransmit() || !sendBreak()) return;
    for (size_t i=0; i<count+3; ++i) if (!sendByte(frame[i])) return;
    reportCapture(frame,count+3); return;
  }
  if (!strncmp(command, "POLL ", 5)) {
    if (strlen(command + 5) != 2) { Serial.println(F("ERR ID")); return; }
    for (size_t i = 5; i < 7; ++i) {
      char c = command[i];
      if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) {
        Serial.println(F("ERR ID")); return;
      }
    }
    uint8_t id = (uint8_t)strtoul(command + 5, NULL, 16);
    if (id > 0x3D || id == 0x3C) { Serial.println(F("ERR ID")); return; }
    if (!readyToTransmit() || !sendBreak()) return;
    const uint8_t header[] = {0x55, protectedId(id)};
    for (uint8_t b : header) if (!sendByte(b)) return;
    reportCapture(header, sizeof(header)); return;
  }
  Serial.println(F("ERR COMMAND"));
}

void sendDiagnosticRead(const uint8_t* data) {
  if (!readyToTransmit() || !sendBreak()) return;
  // LIN diagnostic frames 3C/3D always use classic checksum, excluding PID.
  uint8_t frame[11] = {0x55,0x3C};
  memcpy(frame + 2, data, 8);
  frame[10] = linChecksum(data, 8, 0);
  for (uint8_t value : frame) if (!sendByte(value)) return;
  reportCapture(frame, sizeof(frame));
  // Response is read by a later POLL 3D; host handles P2 timing and segmentation.
}

void setup() {
  // Preload HIGH before enabling open drain to avoid an active-low startup pulse.
  digitalWrite(POWER_PIN, HIGH); pinMode(POWER_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SLP_PIN, LOW); pinMode(SLP_PIN, OUTPUT);
  powerOff();
  powerTimer.setOverflow(1000, HERTZ_FORMAT);
  powerTimer.attachInterrupt(powerTick);
  powerTimer.resume();
  digitalWrite(PC13, HIGH); pinMode(PC13, OUTPUT);
  configureUart();
  Serial.begin(115200); // Native CDC: host baud setting does not clock USART2.
  bridge_cpu_hz = SystemCoreClock;
  bridge_magic = 0x4C494E31; // LIN1
  lastHostMs = millis();
}

void loop() {
  static bool wasConnected = false;
  static uint32_t blinkMs = 0;
  bool connected = Serial.dtr();
  if (connected && !wasConnected) {
    powerOff(); commandLength = 0; commandOverflow = false;
    Serial.println(F("STM32_LIN_10 READY DISABLED"));
  }
  if (!connected || (uint32_t)(millis() - lastHostMs) > 3000) powerOff();
  wasConnected = connected;
  if ((uint32_t)(millis() - blinkMs) >= 500) {
    blinkMs = millis(); ++bridge_heartbeat;
    digitalWrite(PC13, (bridge_heartbeat & 1) ? LOW : HIGH);
  }
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      command[commandLength] = '\0';
      if (commandOverflow) Serial.println(F("ERR LINE_TOO_LONG"));
      else if (commandLength) executeCommand();
      commandLength = 0; commandOverflow = false;
    } else if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
    else commandOverflow = true;
  }
}





