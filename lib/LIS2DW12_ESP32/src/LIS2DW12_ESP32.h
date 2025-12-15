#pragma once
#include <Arduino.h>
#include <Wire.h>

class LIS2DW12 {
public:
  // Registers
  static constexpr uint8_t REG_WHOAMI  = 0x0F; // WHO_AM_I => 0x44
  static constexpr uint8_t REG_CTRL1   = 0x20; // ODR + MODE + LP_MODE
  static constexpr uint8_t REG_CTRL2   = 0x21; // BDU + IF_ADD_INC + ...
  static constexpr uint8_t REG_CTRL6   = 0x25; // BW_FILT + FS + LOW_NOISE
  static constexpr uint8_t REG_OUT_X_L_ADDR = 0x28; // 6 bytes burst

  static constexpr uint8_t WHOAMI_VALUE = 0x44;

  // CTRL1 fields
  enum class Odr : uint8_t {
    PowerDown      = 0b0000,
    Hz12_5_or_1_6  = 0b0001, // HP: 12.5Hz / LP: 1.6Hz
    Hz12_5         = 0b0010,
    Hz25           = 0b0011,
    Hz50           = 0b0100,
    Hz100          = 0b0101,
    Hz200          = 0b0110,
    Hz400_or_200   = 0b0111, // HP: 400Hz / LP: 200Hz
    Hz800_or_200   = 0b1000, // HP: 800Hz / LP: 200Hz
    Hz1600_or_200  = 0b1001  // HP: 1.6kHz / LP: 200Hz
  };

  enum class Mode : uint8_t {
    LowPower = 0b00,
    HighPerf = 0b01,
    OnDemand = 0b10
  };

  enum class LowPowerMode : uint8_t {
    LP1_12bit = 0b00,
    LP2_14bit = 0b01,
    LP3_14bit = 0b10,
    LP4_14bit = 0b11
  };

  // CTRL6 fields
  enum class FullScale : uint8_t {
    G2  = 0b00,
    G4  = 0b01,
    G8  = 0b10,
    G16 = 0b11
  };

  enum class Bandwidth : uint8_t {
    ODR_div2  = 0b00,
    ODR_div4  = 0b01,
    ODR_div10 = 0b10,
    ODR_div20 = 0b11
  };

  struct Config {
    Odr          odr      = Odr::Hz100;
    Mode         mode     = Mode::HighPerf;
    LowPowerMode lpMode   = LowPowerMode::LP2_14bit;
    FullScale    fs       = FullScale::G2;
    Bandwidth    bw       = Bandwidth::ODR_div2;
    bool         lowNoise = true;
    bool         bdu      = true;  // Block Data Update
    bool         autoInc  = true;  // IF_ADD_INC
  };

  struct Calibration {
    bool  enabled = false;
    float offset_g[3] = {0, 0, 0}; // X,Y,Z offset (g)
    float scale[3]    = {1, 1, 1}; // X,Y,Z scale (gain)
  };

  // 6-pozisyon sırasi
  enum class Pose : uint8_t {
    Xp = 0, Xn = 1, Yp = 2, Yn = 3, Zp = 4, Zn = 5
  };

public:
  LIS2DW12(TwoWire& wire = Wire, uint8_t i2cAddr = 0x18);

  bool begin(int sda = -1, int scl = -1, uint32_t clockHz = 400000);

  bool probe();
  uint8_t whoAmI();

  bool applyConfig(const Config& cfg);
  bool setPowerMode(Odr odr, Mode mode, LowPowerMode lpMode);
  bool setScaleAndFilters(FullScale fs, bool lowNoise,
                          Bandwidth bw = Bandwidth::ODR_div2,
                          bool highPassPathFDS = false);

  bool setRateHz(uint16_t hz);

  bool setBDU(bool enable);
  bool setAutoIncrement(bool enable);

  bool readRaw(int16_t& x, int16_t& y, int16_t& z);
  bool readRaw(int16_t out[3]);

  bool readRawAligned(int16_t& x, int16_t& y, int16_t& z);
  bool readRawAligned(int16_t out[3]);

  bool readG(float& gx, float& gy, float& gz);
  bool readG(float out_g[3]);

  uint8_t activeResolutionBits() const;

  float sensitivity_mg_per_lsb(uint8_t resBits, FullScale fs) const;
  float alignedRawToG(int16_t alignedRaw, uint8_t resBits, FullScale fs) const;

  void setOutputQuantization(uint8_t bits) { _qBits = bits; }
  uint8_t getOutputQuantization() const { return _qBits; }

  bool calibrateStatic(uint16_t samples = 500, uint16_t sampleDelayMs = 5, float expectedZ_g = 1.0f);

  bool collectPoseAverage(Pose pose, float outAvg_g[3], uint16_t samples = 500, uint16_t sampleDelayMs = 5);
  bool calibrate6PositionFromAverages(const float meas[6][3]);
  bool calibrate6PositionInteractive(Stream& s, uint16_t samples = 500, uint16_t sampleDelayMs = 5);

  void setAxisScaleFromMeasured(uint8_t axis /*0:X 1:Y 2:Z*/, float measured_g, float expected_g);

  void setCalibration(const Calibration& c) { _cal = c; }
  Calibration getCalibration() const { return _cal; }
  void clearCalibration() { _cal = Calibration(); }

  bool saveCalibrationNVS(const char* nameSpace = "lis2dw12", const char* key = "cal");
  bool loadCalibrationNVS(const char* nameSpace = "lis2dw12", const char* key = "cal");
  bool clearCalibrationNVS(const char* nameSpace = "lis2dw12", const char* key = "cal");

  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t& val);
  bool readBytes(uint8_t startReg, uint8_t* buf, size_t len);

  uint8_t address() const { return _addr; }
  TwoWire& wire() const { return _wire; }
  FullScale getFullScale() const { return _fs; }
  Mode getMode() const { return _mode; }
  LowPowerMode getLowPowerMode() const { return _lpMode; }
  Odr getOdr() const { return _odr; }

private:
  TwoWire& _wire;
  uint8_t  _addr;

  FullScale _fs = FullScale::G2;
  Mode _mode = Mode::HighPerf;
  LowPowerMode _lpMode = LowPowerMode::LP2_14bit;
  Odr _odr = Odr::Hz100;

  Calibration _cal;
  uint8_t _qBits = 0;

  bool readModifyWrite(uint8_t reg, uint8_t clearMask, uint8_t setMask);
  void applyCalibration(float& gx, float& gy, float& gz) const;

  bool readG_uncal(float& gx, float& gy, float& gz);
  int16_t quantizeAlignedRaw(int16_t alignedRaw, uint8_t fromBits, uint8_t toBits) const;

  static Odr odrFromHz(uint16_t hz);
};
