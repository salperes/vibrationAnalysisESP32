#include "LIS2DW12_ESP32.h"
#include <math.h>
#include <Preferences.h>

static constexpr uint32_t CAL_VERSION = 1;

struct CalBlob {
  uint32_t version;
  uint8_t  enabled;
  float    offset_g[3];
  float    scale[3];
  uint32_t crc;
};

static uint32_t crc32_simple(const uint8_t* data, size_t len) {
  uint32_t c = 0xA5A5A5A5u;
  for (size_t i=0; i<len; i++) {
    c ^= data[i];
    c = (c << 5) | (c >> 27);
    c += 0x9E3779B9u;
  }
  return c;
}

LIS2DW12::LIS2DW12(TwoWire& wire, uint8_t i2cAddr)
: _wire(wire), _addr(i2cAddr) {}

bool LIS2DW12::begin(int sda, int scl, uint32_t clockHz) {
  if (sda >= 0 && scl >= 0) {
    _wire.begin(sda, scl);
  }
  _wire.setClock(clockHz);

  if (!probe()) return false;

  setBDU(true);
  setAutoIncrement(true);

  return true;
}

bool LIS2DW12::probe() {
  return whoAmI() == WHOAMI_VALUE;
}

uint8_t LIS2DW12::whoAmI() {
  uint8_t v = 0;
  if (!readReg(REG_WHOAMI, v)) return 0;
  return v;
}

bool LIS2DW12::applyConfig(const Config& cfg) {
  if (!setBDU(cfg.bdu)) return false;
  if (!setAutoIncrement(cfg.autoInc)) return false;
  if (!setPowerMode(cfg.odr, cfg.mode, cfg.lpMode)) return false;
  if (!setScaleAndFilters(cfg.fs, cfg.lowNoise, cfg.bw)) return false;
  return true;
}

bool LIS2DW12::setPowerMode(Odr odr, Mode mode, LowPowerMode lpMode) {
  uint8_t v = (uint8_t(odr) << 4) | (uint8_t(mode) << 2) | uint8_t(lpMode);
  if (!writeReg(REG_CTRL1, v)) return false;

  _odr = odr;
  _mode = mode;
  _lpMode = lpMode;
  return true;
}

bool LIS2DW12::setScaleAndFilters(FullScale fs, bool lowNoise, Bandwidth bw, bool highPassPathFDS) {
  uint8_t v = (uint8_t(bw) << 6) | (uint8_t(fs) << 4);
  if (highPassPathFDS) v |= (1 << 3);
  if (lowNoise)        v |= (1 << 2);

  if (!writeReg(REG_CTRL6, v)) return false;
  _fs = fs;
  return true;
}

bool LIS2DW12::setRateHz(uint16_t hz) {
  Odr odr = odrFromHz(hz);
  return setPowerMode(odr, _mode, _lpMode);
}

LIS2DW12::Odr LIS2DW12::odrFromHz(uint16_t hz) {
  if (hz == 0) return Odr::PowerDown;
  if (hz <= 2)  return Odr::Hz12_5_or_1_6;
  if (hz <= 13) return Odr::Hz12_5;
  if (hz <= 25) return Odr::Hz25;
  if (hz <= 50) return Odr::Hz50;
  if (hz <= 100) return Odr::Hz100;
  if (hz <= 200) return Odr::Hz200;
  if (hz <= 400) return Odr::Hz400_or_200;
  if (hz <= 800) return Odr::Hz800_or_200;
  return Odr::Hz1600_or_200;
}

bool LIS2DW12::setBDU(bool enable) {
  return readModifyWrite(REG_CTRL2, (1 << 3), enable ? (1 << 3) : 0);
}

bool LIS2DW12::setAutoIncrement(bool enable) {
  return readModifyWrite(REG_CTRL2, (1 << 2), enable ? (1 << 2) : 0);
}

bool LIS2DW12::readRaw(int16_t& x, int16_t& y, int16_t& z) {
  uint8_t b[6];
  if (!readBytes(REG_OUT_X_L_ADDR, b, 6)) return false;
  x = (int16_t)(uint16_t(b[1]) << 8 | b[0]);
  y = (int16_t)(uint16_t(b[3]) << 8 | b[2]);
  z = (int16_t)(uint16_t(b[5]) << 8 | b[4]);
  return true;
}

bool LIS2DW12::readRaw(int16_t out[3]) {
  return readRaw(out[0], out[1], out[2]);
}

uint8_t LIS2DW12::activeResolutionBits() const {
  if (_mode == Mode::LowPower && _lpMode == LowPowerMode::LP1_12bit) return 12;
  return 14;
}

bool LIS2DW12::readRawAligned(int16_t& x, int16_t& y, int16_t& z) {
  if (!readRaw(x, y, z)) return false;

  uint8_t bits = activeResolutionBits();
  uint8_t shift = (bits == 12) ? 4 : 2;

  x = (int16_t)(x >> shift);
  y = (int16_t)(y >> shift);
  z = (int16_t)(z >> shift);
  return true;
}

bool LIS2DW12::readRawAligned(int16_t out[3]) {
  return readRawAligned(out[0], out[1], out[2]);
}

float LIS2DW12::sensitivity_mg_per_lsb(uint8_t resBits, FullScale fs) const {
  const bool is12 = (resBits == 12);
  switch (fs) {
    case FullScale::G2:  return is12 ? 0.976f : 0.244f;
    case FullScale::G4:  return is12 ? 1.952f : 0.488f;
    case FullScale::G8:  return is12 ? 3.904f : 0.976f;
    case FullScale::G16: return is12 ? 7.808f : 1.952f;
  }
  return is12 ? 0.976f : 0.244f;
}

float LIS2DW12::alignedRawToG(int16_t alignedRaw, uint8_t resBits, FullScale fs) const {
  float mg_per_lsb = sensitivity_mg_per_lsb(resBits, fs);
  return (float(alignedRaw) * mg_per_lsb) / 1000.0f;
}

int16_t LIS2DW12::quantizeAlignedRaw(int16_t alignedRaw, uint8_t fromBits, uint8_t toBits) const {
  if (toBits == 0 || toBits >= fromBits) return alignedRaw;
  uint8_t drop = fromBits - toBits;

  int16_t q = (int16_t)(alignedRaw >> drop);
  q = (int16_t)(q << drop);
  return q;
}

bool LIS2DW12::readG_uncal(float& gx, float& gy, float& gz) {
  int16_t ax, ay, az;
  if (!readRawAligned(ax, ay, az)) return false;

  uint8_t res = activeResolutionBits();

  if (_qBits == 10 || _qBits == 12 || _qBits == 14) {
    ax = quantizeAlignedRaw(ax, res, _qBits);
    ay = quantizeAlignedRaw(ay, res, _qBits);
    az = quantizeAlignedRaw(az, res, _qBits);
    res = _qBits;
  }

  gx = alignedRawToG(ax, res, _fs);
  gy = alignedRawToG(ay, res, _fs);
  gz = alignedRawToG(az, res, _fs);
  return true;
}

void LIS2DW12::applyCalibration(float& gx, float& gy, float& gz) const {
  if (!_cal.enabled) return;
  gx = (gx - _cal.offset_g[0]) * _cal.scale[0];
  gy = (gy - _cal.offset_g[1]) * _cal.scale[1];
  gz = (gz - _cal.offset_g[2]) * _cal.scale[2];
}

bool LIS2DW12::readG(float& gx, float& gy, float& gz) {
  if (!readG_uncal(gx, gy, gz)) return false;
  applyCalibration(gx, gy, gz);
  return true;
}

bool LIS2DW12::readG(float out_g[3]) {
  return readG(out_g[0], out_g[1], out_g[2]);
}

// ---------- Calibration ----------
bool LIS2DW12::calibrateStatic(uint16_t samples, uint16_t sampleDelayMs, float expectedZ_g) {
  if (samples < 10) samples = 10;

  for (int i = 0; i < 10; i++) {
    float gx, gy, gz;
    readG_uncal(gx, gy, gz);
    delay(sampleDelayMs);
  }

  double sx=0, sy=0, sz=0;
  for (uint16_t i=0; i<samples; i++) {
    float gx, gy, gz;
    if (!readG_uncal(gx, gy, gz)) return false;
    sx += gx; sy += gy; sz += gz;
    delay(sampleDelayMs);
  }

  float mx = float(sx / samples);
  float my = float(sy / samples);
  float mz = float(sz / samples);

  _cal.offset_g[0] = mx - 0.0f;
  _cal.offset_g[1] = my - 0.0f;
  _cal.offset_g[2] = mz - expectedZ_g;

  _cal.scale[0] = 1.0f;
  _cal.scale[1] = 1.0f;
  _cal.scale[2] = 1.0f;
  _cal.enabled = true;
  return true;
}

void LIS2DW12::setAxisScaleFromMeasured(uint8_t axis, float measured_g, float expected_g) {
  if (axis > 2) return;
  if (fabsf(measured_g) < 1e-6f) return;
  _cal.scale[axis] = expected_g / measured_g;
  _cal.enabled = true;
}

bool LIS2DW12::collectPoseAverage(Pose /*pose*/, float outAvg_g[3], uint16_t samples, uint16_t sampleDelayMs) {
  if (samples < 10) samples = 10;

  for (int i=0; i<10; i++) {
    float gx, gy, gz;
    readG_uncal(gx, gy, gz);
    delay(sampleDelayMs);
  }

  double sx=0, sy=0, sz=0;
  for (uint16_t i=0; i<samples; i++) {
    float gx, gy, gz;
    if (!readG_uncal(gx, gy, gz)) return false;
    sx += gx; sy += gy; sz += gz;
    delay(sampleDelayMs);
  }
  outAvg_g[0] = float(sx / samples);
  outAvg_g[1] = float(sy / samples);
  outAvg_g[2] = float(sz / samples);
  return true;
}

bool LIS2DW12::calibrate6PositionFromAverages(const float meas[6][3]) {
  const float mxp = meas[(int)Pose::Xp][0];
  const float mxn = meas[(int)Pose::Xn][0];
  const float myp = meas[(int)Pose::Yp][1];
  const float myn = meas[(int)Pose::Yn][1];
  const float mzp = meas[(int)Pose::Zp][2];
  const float mzn = meas[(int)Pose::Zn][2];

  const float dx = mxp - mxn;
  const float dy = myp - myn;
  const float dz = mzp - mzn;

  if (fabsf(dx) < 0.5f || fabsf(dy) < 0.5f || fabsf(dz) < 0.5f) return false;

  _cal.offset_g[0] = 0.5f * (mxp + mxn);
  _cal.offset_g[1] = 0.5f * (myp + myn);
  _cal.offset_g[2] = 0.5f * (mzp + mzn);

  _cal.scale[0] = 2.0f / dx;
  _cal.scale[1] = 2.0f / dy;
  _cal.scale[2] = 2.0f / dz;

  _cal.enabled = true;
  return true;
}

bool LIS2DW12::calibrate6PositionInteractive(Stream& s, uint16_t samples, uint16_t sampleDelayMs) {
  float meas[6][3] = {0};

  auto waitEnter = [&](const char* msg) {
    s.println(msg);
    s.println("Devam icin ENTER gonder...");
    while (!s.available()) delay(10);
    while (s.available()) s.read();
    delay(200);
  };

  waitEnter("Poz 1/6: +X (X ekseni yukari)");
  if (!collectPoseAverage(Pose::Xp, meas[(int)Pose::Xp], samples, sampleDelayMs)) return false;

  waitEnter("Poz 2/6: -X (X ekseni asagi)");
  if (!collectPoseAverage(Pose::Xn, meas[(int)Pose::Xn], samples, sampleDelayMs)) return false;

  waitEnter("Poz 3/6: +Y (Y ekseni yukari)");
  if (!collectPoseAverage(Pose::Yp, meas[(int)Pose::Yp], samples, sampleDelayMs)) return false;

  waitEnter("Poz 4/6: -Y (Y ekseni asagi)");
  if (!collectPoseAverage(Pose::Yn, meas[(int)Pose::Yn], samples, sampleDelayMs)) return false;

  waitEnter("Poz 5/6: +Z (Z ekseni yukari)");
  if (!collectPoseAverage(Pose::Zp, meas[(int)Pose::Zp], samples, sampleDelayMs)) return false;

  waitEnter("Poz 6/6: -Z (Z ekseni asagi)");
  if (!collectPoseAverage(Pose::Zn, meas[(int)Pose::Zn], samples, sampleDelayMs)) return false;

  bool ok = calibrate6PositionFromAverages(meas);
  if (ok) {
    s.println("6-pozisyon kalibrasyon OK.");
    s.printf("Offset(g): X=%.6f Y=%.6f Z=%.6f\n", _cal.offset_g[0], _cal.offset_g[1], _cal.offset_g[2]);
    s.printf("Scale    : X=%.6f Y=%.6f Z=%.6f\n", _cal.scale[0], _cal.scale[1], _cal.scale[2]);
  } else {
    s.println("6-pozisyon kalibrasyon FAILED.");
  }
  return ok;
}

// ---------- NVS ----------
bool LIS2DW12::saveCalibrationNVS(const char* nameSpace, const char* key) {
  Preferences pref;
  if (!pref.begin(nameSpace, false)) return false;

  CalBlob b{};
  b.version = CAL_VERSION;
  b.enabled = _cal.enabled ? 1 : 0;
  for (int i=0; i<3; i++) { b.offset_g[i] = _cal.offset_g[i]; b.scale[i] = _cal.scale[i]; }
  b.crc = 0;
  b.crc = crc32_simple(reinterpret_cast<const uint8_t*>(&b), sizeof(CalBlob) - sizeof(uint32_t));

  size_t written = pref.putBytes(key, &b, sizeof(b));
  pref.end();
  return written == sizeof(b);
}

bool LIS2DW12::loadCalibrationNVS(const char* nameSpace, const char* key) {
  Preferences pref;
  if (!pref.begin(nameSpace, true)) return false;

  CalBlob b{};
  size_t got = pref.getBytes(key, &b, sizeof(b));
  pref.end();
  if (got != sizeof(b)) return false;
  if (b.version != CAL_VERSION) return false;

  uint32_t crc = crc32_simple(reinterpret_cast<const uint8_t*>(&b), sizeof(CalBlob) - sizeof(uint32_t));
  if (crc != b.crc) return false;

  _cal.enabled = (b.enabled != 0);
  for (int i=0; i<3; i++) { _cal.offset_g[i] = b.offset_g[i]; _cal.scale[i] = b.scale[i]; }
  return true;
}

bool LIS2DW12::clearCalibrationNVS(const char* nameSpace, const char* key) {
  Preferences pref;
  if (!pref.begin(nameSpace, false)) return false;
  bool ok = pref.remove(key);
  pref.end();
  return ok;
}

// ---------- Low-level ----------
bool LIS2DW12::writeReg(uint8_t reg, uint8_t val) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  _wire.write(val);
  return _wire.endTransmission() == 0;
}

bool LIS2DW12::readReg(uint8_t reg, uint8_t& val) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  if (_wire.endTransmission(false) != 0) return false;
  if (_wire.requestFrom((int)_addr, 1) != 1) return false;
  val = _wire.read();
  return true;
}

bool LIS2DW12::readBytes(uint8_t startReg, uint8_t* buf, size_t len) {
  _wire.beginTransmission(_addr);
  _wire.write(startReg);
  if (_wire.endTransmission(false) != 0) return false;
  int got = _wire.requestFrom((int)_addr, (int)len);
  if (got != (int)len) return false;
  for (size_t i=0; i<len; i++) buf[i] = _wire.read();
  return true;
}

bool LIS2DW12::readModifyWrite(uint8_t reg, uint8_t clearMask, uint8_t setMask) {
  uint8_t v = 0;
  if (!readReg(reg, v)) return false;
  v &= ~clearMask;
  v |= setMask;
  return writeReg(reg, v);
}
