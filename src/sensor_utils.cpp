#include <Preferences.h>

#include "sensor_utils.h"

// ---- Cached live-preview sensor ----
LIS2DW12 g_liveSensor(Wire, 0x18);
bool g_liveSensorReady = false;

// ---- File validation ----
// Accepts these forms only:
//   "/accelYYMMDDHHMMSS.dat"        (manual recording)
//   "/accelYYMMDDHHMMSS_NN.dat"
//   "/grabYYMMDDHHMMSS.dat"         (triggered/grab recording)
//   "/grabYYMMDDHHMMSS_NN.dat"
// where YYMMDDHHMMSS is 12 digits and NN is 2 digits.
bool isSafeAccelFile(String p)
{
  if (!p.startsWith("/"))
    p = "/" + p;
  if (!p.endsWith(".dat"))
    return false;

  // Detect prefix and digit-start offset.
  size_t digitStart;
  if (p.startsWith("/accel"))
    digitStart = 6; // "/accel" is 6 chars
  else if (p.startsWith("/grab"))
    digitStart = 5; // "/grab" is 5 chars
  else
    return false;

  const size_t L = p.length();
  // Without "_NN": digitStart + 12 + ".dat"(4) = digitStart + 16
  // With    "_NN": digitStart + 12 + 3 + 4   = digitStart + 19
  if (L != digitStart + 16 && L != digitStart + 19)
    return false;

  for (size_t i = digitStart; i < digitStart + 12; i++)
    if (p[i] < '0' || p[i] > '9')
      return false;

  if (L == digitStart + 19)
  {
    if (p[digitStart + 12] != '_')
      return false;
    if (p[digitStart + 13] < '0' || p[digitStart + 13] > '9')
      return false;
    if (p[digitStart + 14] < '0' || p[digitStart + 14] > '9')
      return false;
  }

  return true;
}

bool fileExists(const String &path) { return LittleFS.exists(path); }

// ---- String helpers ----
String jsonEscape(const String &s)
{
  String out;
  out.reserve(s.length() + 4);
  for (unsigned i = 0; i < s.length(); i++)
  {
    char c = s[i];
    if (c == '"')
      out += "\\\"";
    else if (c == '\\')
      out += "\\\\";
    else
      out += c;
  }
  return out;
}

bool isValidYYMMDDHHMMSS(const String &ts)
{
  if (ts.length() != 12)
    return false;
  for (size_t i = 0; i < 12; i++)
    if (ts[i] < '0' || ts[i] > '9')
      return false;
  return true;
}

void copyToFixed(const String &v, char *dst, size_t cap)
{
  if (cap == 0) return;
  memset(dst, 0, cap);
  size_t n = v.length();
  if (n >= cap) n = cap - 1;
  memcpy(dst, v.c_str(), n);
}

String makeNewFileNameFromUI(const String &ts12)
{
  String base = "/accel" + ts12;
  String path = base + ".dat";
  if (!LittleFS.exists(path))
    return path;

  for (int i = 1; i <= 99; i++)
  {
    char suf[8];
    snprintf(suf, sizeof(suf), "_%02d", i);
    String p2 = base + String(suf) + ".dat";
    if (!LittleFS.exists(p2))
      return p2;
  }
  // 100 collisions in the same second is unreachable in practice; signal failure.
  return String();
}

// ---- Header read ----
bool readParsedHeader(File &f, ParsedHeader &out)
{
  memset(&out, 0, sizeof(out));

  // Peek at magic + version (10 bytes) without consuming the rest yet.
  if (f.size() < 10)
    return false;

  char magic[8];
  uint16_t version = 0;
  f.seek(0, SeekSet);
  if (f.read((uint8_t *)magic, 8) != 8)
    return false;
  if (f.read((uint8_t *)&version, 2) != 2)
    return false;
  if (memcmp(magic, "LIS2DW12", 8) != 0)
    return false;

  f.seek(0, SeekSet);

  if (version == 3)
  {
    if (f.size() < (int)sizeof(FileHeaderV3))
      return false;
    FileHeaderV3 h;
    if (f.read((uint8_t *)&h, sizeof(h)) != sizeof(h))
      return false;
    out.version      = 3;
    out.header_bytes = sizeof(FileHeaderV3);
    out.rate_hz      = h.rate_hz;
    out.record_s     = h.record_s;
    out.samples      = h.samples;
    out.fs_g         = h.fs_g;
    out.res_bits     = h.res_bits;
    out.q_bits       = h.q_bits;
    for (int i = 0; i < 3; i++)
    {
      out.cal_offset_g[i] = h.cal_offset_g[i];
      out.cal_scale[i]    = h.cal_scale[i];
    }
    out.trig_mode = 0xFF; // N/A for V3
    return true;
  }

  if (version == 4)
  {
    if (f.size() < (int)sizeof(FileHeaderV4))
      return false;
    FileHeaderV4 h;
    if (f.read((uint8_t *)&h, sizeof(h)) != sizeof(h))
      return false;
    out.version        = 4;
    out.header_bytes   = sizeof(FileHeaderV4);
    out.rate_hz        = h.rate_hz;
    out.record_s       = h.record_s;
    out.samples        = h.samples;
    out.fs_g           = h.fs_g;
    out.res_bits       = h.res_bits;
    out.q_bits         = h.q_bits;
    out.flags          = h.flags;
    for (int i = 0; i < 3; i++)
    {
      out.cal_offset_g[i] = h.cal_offset_g[i];
      out.cal_scale[i]    = h.cal_scale[i];
    }
    out.pre_samples    = h.pre_samples;
    out.threshold_used = h.threshold_used;
    out.trig_mode      = h.trig_mode;
    out.trig_mult      = h.trig_mult;
    memcpy(out.serial_no,     h.serial_no,     sizeof(out.serial_no));
    memcpy(out.device_name,   h.device_name,   sizeof(out.device_name));
    memcpy(out.meas_point,    h.meas_point,    sizeof(out.meas_point));
    memcpy(out.scan_dir,      h.scan_dir,      sizeof(out.scan_dir));
    memcpy(out.operator_name, h.operator_name, sizeof(out.operator_name));
    memcpy(out.notes,         h.notes,         sizeof(out.notes));
    return true;
  }

  return false; // unsupported version
}

// ---- Header rewrite (samples count, after recording finalizes) ----
// Both V3 and V4 keep `samples` as a uint32 at the same byte offset (10..13).
bool rewriteHeaderSamples(const String &path, uint32_t samplesWritten)
{
  if (!fileExists(path))
    return false;
  File f = LittleFS.open(path, "r+");
  if (!f)
    return false;

  // Read version to know which struct layout to use.
  char magic[8];
  uint16_t version = 0;
  if (f.read((uint8_t *)magic, 8) != 8 ||
      f.read((uint8_t *)&version, 2) != 2)
  {
    f.close();
    return false;
  }
  if (memcmp(magic, "LIS2DW12", 8) != 0)
  {
    f.close();
    return false;
  }

  f.seek(0, SeekSet);
  bool ok = false;
  if (version == 3)
  {
    FileHeaderV3 h;
    if (f.read((uint8_t *)&h, sizeof(h)) == sizeof(h))
    {
      h.samples = samplesWritten;
      f.seek(0, SeekSet);
      ok = (f.write((uint8_t *)&h, sizeof(h)) == sizeof(h));
    }
  }
  else if (version == 4)
  {
    FileHeaderV4 h;
    if (f.read((uint8_t *)&h, sizeof(h)) == sizeof(h))
    {
      h.samples = samplesWritten;
      f.seek(0, SeekSet);
      ok = (f.write((uint8_t *)&h, sizeof(h)) == sizeof(h));
    }
  }
  f.close();
  return ok;
}

// ---- FullScale conversion ----
LIS2DW12::FullScale fsFromG(uint8_t fs_g)
{
  switch (fs_g)
  {
  case 2:
    return LIS2DW12::FullScale::G2;
  case 4:
    return LIS2DW12::FullScale::G4;
  case 8:
    return LIS2DW12::FullScale::G8;
  case 16:
    return LIS2DW12::FullScale::G16;
  default:
    return LIS2DW12::FullScale::G2;
  }
}

uint8_t fsToByte(LIS2DW12::FullScale fs)
{
  switch (fs)
  {
  case LIS2DW12::FullScale::G2:
    return 2;
  case LIS2DW12::FullScale::G4:
    return 4;
  case LIS2DW12::FullScale::G8:
    return 8;
  case LIS2DW12::FullScale::G16:
    return 16;
  }
  return 2;
}

// ---- Pose name ----
const char *poseName(int step)
{
  switch (step)
  {
  case 0:
    return "X+";
  case 1:
    return "X-";
  case 2:
    return "Y+";
  case 3:
    return "Y-";
  case 4:
    return "Z+";
  case 5:
    return "Z-";
  default:
    return "-";
  }
}

// ---- Analysis math ----
float mgPerLsb(uint8_t res_bits, uint8_t fs_g)
{
  // AN5038 Table 15
  const bool is12 = (res_bits == 12);
  switch (fs_g)
  {
  case 2:
    return is12 ? 0.976f : 0.244f;
  case 4:
    return is12 ? 1.952f : 0.488f;
  case 8:
    return is12 ? 3.904f : 0.976f;
  case 16:
    return is12 ? 7.808f : 1.952f;
  default:
    return is12 ? 0.976f : 0.244f;
  }
}

float rawAlignedToG(int16_t rawAligned, uint8_t res_bits, uint8_t fs_g)
{
  float mg = mgPerLsb(res_bits, fs_g);
  return (float)rawAligned * (mg / 1000.0f);
}

float applyCal1(float g, float offset, float scale)
{
  return (g - offset) * scale;
}

float rmsFromSumSq(double sumSq, uint32_t n)
{
  return (n > 0) ? sqrtf((float)(sumSq / (double)n)) : 0.0f;
}

// ---- Device identity ----
void loadDeviceIdentity()
{
  // Serial: hex of MAC's last 6 bytes (12 hex chars + null = 13 of 16)
  uint64_t mac = ESP.getEfuseMac();
  uint32_t hi = (uint32_t)((mac >> 24) & 0xFFFFFF);
  uint32_t lo = (uint32_t)(mac & 0xFFFFFF);
  snprintf(g_device_serial, sizeof(g_device_serial), "%06lX%06lX",
           (unsigned long)hi, (unsigned long)lo);

  Preferences pref;
  bool loaded = false;
  if (pref.begin("device", true))
  {
    String n = pref.getString("name", "");
    pref.end();
    if (n.length() > 0 && n.length() < sizeof(g_device_name))
    {
      memset(g_device_name, 0, sizeof(g_device_name));
      memcpy(g_device_name, n.c_str(), n.length());
      loaded = true;
    }
  }
  if (!loaded)
  {
    // Default: accMeter-XXXXXX (last 6 hex chars of serial -> short tag)
    snprintf(g_device_name, sizeof(g_device_name), "accMeter-%06lX",
             (unsigned long)lo);
  }

  Serial.printf("[ID] serial=%s  name=%s\n", g_device_serial, g_device_name);
}

bool saveDeviceName(const char *name)
{
  if (!name) return false;
  size_t L = strlen(name);
  if (L == 0 || L >= sizeof(g_device_name)) return false;

  Preferences pref;
  if (!pref.begin("device", false)) return false;
  size_t wrote = pref.putString("name", name);
  pref.end();
  if (wrote != L) return false;

  memset(g_device_name, 0, sizeof(g_device_name));
  memcpy(g_device_name, name, L);
  return true;
}

// ---- Calibration boot-load ----
void loadCalibrationAtBoot()
{
  if (g_liveSensor.loadCalibrationNVS("lis2dw12", "cal"))
  {
    auto cal = g_liveSensor.getCalibration();
    g_calibPresent = cal.enabled;
    if (cal.enabled)
    {
      Serial.printf("[CAL] Loaded from NVS: off=[%.4f,%.4f,%.4f] sc=[%.4f,%.4f,%.4f]\n",
                    cal.offset_g[0], cal.offset_g[1], cal.offset_g[2],
                    cal.scale[0], cal.scale[1], cal.scale[2]);
    }
    else
    {
      Serial.println("[CAL] NVS blob found but enabled=false");
    }
  }
  else
  {
    g_calibPresent = false;
    Serial.println("[CAL] No calibration in NVS (run /api/calibrate_static or /api/calibrate6)");
  }
}

// ---- Live preview state ----
void resetLivePreviewState()
{
  g_liveLastMs = 0;
  for (int i = 0; i < 3; i++)
  {
    g_live_g[i] = 0;
    g_live_acc_mps2[i] = 0;
    g_live_vel_mmps[i] = 0;
    g_live_disp_mm[i] = 0;
  }
  g_live_mag_acc = 0;
  g_live_mag_vel_mmps = 0;
  g_live_mag_disp_mm = 0;
  g_live_dt_us = 0;
}
