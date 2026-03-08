#include "sensor_utils.h"

// ---- Cached live-preview sensor ----
LIS2DW12 g_liveSensor(Wire, 0x18);
bool g_liveSensorReady = false;

// ---- File validation ----
bool isSafeAccelFile(String p)
{
  if (!p.startsWith("/"))
    p = "/" + p;
  if (!p.startsWith("/accel"))
    return false;
  if (!p.endsWith(".dat"))
    return false;
  if (p.indexOf("..") >= 0)
    return false;
  if (p.indexOf("//") >= 0)
    return false;
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
  return base + "_" + String(millis()) + ".dat";
}

// ---- Header rewrite ----
bool rewriteHeaderSamples(const String &path, uint32_t samplesWritten)
{
  if (!fileExists(path))
    return false;
  File f = LittleFS.open(path, "r+");
  if (!f)
    return false;

  FileHeaderV3 h{};
  size_t got = f.read((uint8_t *)&h, sizeof(h));
  if (got != sizeof(h))
  {
    f.close();
    return false;
  }
  h.samples = samplesWritten;
  f.seek(0, SeekSet);
  size_t wrote = f.write((uint8_t *)&h, sizeof(h));
  f.close();
  return wrote == sizeof(h);
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
}
