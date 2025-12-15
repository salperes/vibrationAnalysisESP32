#include <Arduino.h>
#include <Wire.h>

static const uint8_t LIS_ADDR = 0x18;
static const uint8_t REG_WHOAMI = 0x0F;
static const uint8_t REG_OUT_X_L = 0x28;

bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int got = Wire.requestFrom((int)addr, (int)len);
  if (got != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t whoami() {
  uint8_t v = 0;
  i2cReadReg(LIS_ADDR, REG_WHOAMI, &v, 1);
  return v;
}

void throughputTest(uint32_t i2cClockHz, uint32_t loops) {
  Wire.setClock(i2cClockHz);
  delay(50);

  uint8_t b[6];
  // warm-up
  for (int i=0;i<50;i++) i2cReadReg(LIS_ADDR, REG_OUT_X_L, b, 6);

  uint32_t t0 = micros();
  uint32_t ok = 0;

  for (uint32_t i=0;i<loops;i++) {
    if (i2cReadReg(LIS_ADDR, REG_OUT_X_L, b, 6)) ok++;
  }

  uint32_t dt = micros() - t0;
  double sec = dt / 1e6;
  double reads_per_s = ok / sec;
  double bytes_per_s = reads_per_s * 6.0;

  Serial.printf("\n[I2C %.0f kHz] loops=%u ok=%u time=%.3fs\n",
                i2cClockHz/1000.0, (unsigned)loops, (unsigned)ok, sec);
  Serial.printf("reads/s = %.1f  => bytes/s = %.1f\n", reads_per_s, bytes_per_s);
  Serial.printf("max sample rate (6B per sample) ~= %.1f Hz\n", reads_per_s);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);

  // LIS2DW12'i basitçe enable et (CTRL1/CTRL6 minimum):
  // CTRL1 = ODR=200Hz(0b0110), MODE=HP(01), LP_MODE=01 => 0b0110 01 01 = 0x65
  // CTRL6 = BW=ODR/2(00), FS=2g(00), LOW_NOISE=1 -> 0b00 00 0 1 00 = 0x04
  i2cWriteReg(LIS_ADDR, 0x20, 0x65);
  i2cWriteReg(LIS_ADDR, 0x25, 0x04);
  // CTRL2: BDU=1 (bit3), IF_ADD_INC=1 (bit2) => 0x0C
  i2cWriteReg(LIS_ADDR, 0x21, 0x0C);

  Serial.printf("WHOAMI=0x%02X (expect 0x44)\n", whoami());

  // Testler:
  throughputTest(100000, 5000);
  throughputTest(400000, 10000);
  throughputTest(1000000, 20000); // bazı hatlarda 1MHz stabil olmayabilir
}

void loop() {}



/*

#include <Arduino.h>
#include <Wire.h>

static const uint8_t LIS_ADDR = 0x18;
static const uint8_t REG_OUT_X_L = 0x28;

static volatile uint32_t dueCount = 0;
static hw_timer_t* timer0 = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&mux);
  dueCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

bool startTimerHz(uint16_t hz) {
  if (!hz) return false;
  uint32_t period_us = 1000000UL / hz;
  timer0 = timerBegin(0, 80, true); // 1MHz tick
  timerAttachInterrupt(timer0, &onTimer, true);
  timerAlarmWrite(timer0, period_us, true);
  timerAlarmEnable(timer0);
  return true;
}
void stopTimer() {
  if (!timer0) return;
  timerAlarmDisable(timer0);
  timerDetachInterrupt(timer0);
  timerEnd(timer0);
  timer0 = nullptr;
}

bool i2cRead6(uint8_t* b) {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(REG_OUT_X_L);
  if (Wire.endTransmission(false) != 0) return false;
  int got = Wire.requestFrom((int)LIS_ADDR, 6);
  if (got != 6) return false;
  for (int i=0;i<6;i++) b[i] = Wire.read();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);
  Wire.setClock(1000000); // 400k ile de dene

  // Sensör minimal enable (aynı ayarlar)
  auto wreg = [&](uint8_t r, uint8_t v){
    Wire.beginTransmission(LIS_ADDR);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
  };
  wreg(0x20, 0x65); // CTRL1
  wreg(0x25, 0x04); // CTRL6
  wreg(0x21, 0x0C); // CTRL2

  const uint16_t targetHz = 1600;   // 100..1600 dene
  const uint32_t testSec  = 10;     // 10 sn test
  const uint32_t targetN  = (uint32_t)targetHz * testSec;

  Serial.printf("RT test: %u Hz for %u s => %u ticks\n", targetHz, testSec, (unsigned)targetN);

  dueCount = 0;
  startTimerHz(targetHz);

  uint8_t b[6];
  uint32_t got = 0;
  uint32_t fail = 0;
  uint32_t maxBacklog = 0;

  uint64_t sumReadUs = 0;
  uint32_t tStart = millis();

  while (got < targetN && (millis() - tStart) < (testSec * 1000UL + 2000UL)) {
    uint32_t localDue;
    portENTER_CRITICAL(&mux);
    localDue = dueCount;
    dueCount = 0;
    portEXIT_CRITICAL(&mux);

    if (localDue > maxBacklog) maxBacklog = localDue;

    while (localDue && got < targetN) {
      uint32_t t0 = micros();
      bool ok = i2cRead6(b);
      uint32_t dt = micros() - t0;

      if (ok) {
        got++;
        sumReadUs += dt;
      } else {
        fail++;
      }
      localDue--;
    }
    delay(0);
  }

  stopTimer();

  double avgReadUs = got ? (double)sumReadUs / got : 0.0;
  Serial.printf("DONE got=%u fail=%u avgRead=%.1fus maxBacklog=%u\n",
                (unsigned)got, (unsigned)fail, avgReadUs, (unsigned)maxBacklog);

  // Basit karar:
  // maxBacklog sürekli büyüyorsa yetişmiyor; küçük dalgalanma normal.
}

void loop() {}

*/