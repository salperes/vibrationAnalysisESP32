#include <Arduino.h>
#include <LittleFS.h>

#include "file_handlers.h"
#include "sensor_utils.h"

// ======================= File list cache =======================
static String buildFilesJsonNow()
{
  String out = "[";
  bool first = true;

  File root = LittleFS.open("/");
  if (!root)
    return "[]";
  if (!root.isDirectory())
  {
    root.close();
    return "[]";
  }

  File f = root.openNextFile();
  while (f)
  {
    String name = f.name();
    if (!name.startsWith("/"))
      name = "/" + name;

    if (isSafeAccelFile(name))
    {
      if (!first)
        out += ",";
      first = false;
      out += "{";
      out += "\"name\":\"" + jsonEscape(name) + "\",";
      out += "\"size\":" + String((uint32_t)f.size());
      out += "}";
    }

    f.close();
    f = root.openNextFile();
    delay(0);
  }
  root.close();

  out += "]";
  return out;
}

static String listFilesJsonCached()
{
  uint32_t now = millis();
  if (now - g_listCacheMs > LIST_CACHE_TTL_MS)
  {
    g_listCache = buildFilesJsonNow();
    g_listCacheMs = now;
  }
  return g_listCache;
}

void rebuildListCache()
{
  g_listCache = buildFilesJsonNow();
  g_listCacheMs = millis();
}

void setRawMaskBits(uint8_t bits)
{
  if (bits == 2 || bits == 3 || bits == 4)
    g_rawMaskBits = bits;
  else
    g_rawMaskBits = 0;
  g_liveLastMs = 0;
}

// ======================= FS info =======================
static String fsInfoJson()
{
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  size_t freeB = (total >= used) ? (total - used) : 0;

  String s = "{";
  s += "\"total\":" + String((uint32_t)total) + ",";
  s += "\"used\":" + String((uint32_t)used) + ",";
  s += "\"free\":" + String((uint32_t)freeB);
  s += "}";
  return s;
}

// ======================= HTTP Handlers =======================
void handleApiList() { server.send(200, "application/json", listFilesJsonCached()); }
void handleApiFsInfo() { server.send(200, "application/json", fsInfoJson()); }

void handleApiRawMask()
{
  if (!server.hasArg("bits"))
  {
    server.send(400, "text/plain", "Missing bits");
    return;
  }
  uint8_t bits = (uint8_t)server.arg("bits").toInt();
  setRawMaskBits(bits);
  server.send(200, "text/plain", "OK");
}

void handleDownload()
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file");
    return;
  }
  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path))
  {
    server.send(400, "text/plain", "Bad file");
    return;
  }
  if (!fileExists(path))
  {
    server.send(404, "text/plain", "Not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f)
  {
    server.send(500, "text/plain", "Open failed");
    return;
  }

  String basename = path;
  if (basename.startsWith("/"))
    basename.remove(0, 1);
  server.sendHeader("Content-Type", "application/octet-stream");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + basename + "\"");
  server.sendHeader("Connection", "close");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

void handleDownloadCSV()
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file");
    return;
  }
  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path))
  {
    server.send(400, "text/plain", "Bad file");
    return;
  }
  if (!fileExists(path))
  {
    server.send(404, "text/plain", "Not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f)
  {
    server.send(500, "text/plain", "Open failed");
    return;
  }
  ParsedHeader h;
  if (!readParsedHeader(f, h))
  {
    f.close();
    server.send(400, "text/plain", "Bad header");
    return;
  }

  String basename = path;
  if (basename.startsWith("/"))
    basename.remove(0, 1);
  String csvName = basename;
  if (csvName.endsWith(".dat"))
    csvName = csvName.substring(0, csvName.length() - 4) + ".csv";

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + csvName + "\"");
  server.sendHeader("Connection", "close");
  server.send(200);

  // CSV preamble: keep V3 layout for compatibility; emit V4 metadata only
  // when present (analysis tools can detect by line presence).
  String hdr;
  hdr.reserve(1024);
  hdr += "# " + basename + "\n";
  hdr += "# header_version=" + String(h.version) + "\n";
  hdr += "# rate_hz=" + String(h.rate_hz) + "\n";
  hdr += "# record_s=" + String(h.record_s) + "\n";
  hdr += "# samples=" + String(h.samples) + "\n";
  hdr += "# fs_g=" + String(h.fs_g) + "\n";
  hdr += "# res_bits=" + String(h.res_bits) + "\n";
  hdr += "# q_bits=" + String(h.q_bits) + "\n";
  hdr += "# cal_offset_g=" + String(h.cal_offset_g[0], 6) + "," + String(h.cal_offset_g[1], 6) + "," + String(h.cal_offset_g[2], 6) + "\n";
  hdr += "# cal_scale=" + String(h.cal_scale[0], 6) + "," + String(h.cal_scale[1], 6) + "," + String(h.cal_scale[2], 6) + "\n";
  if (h.version >= 4)
  {
    hdr += "# flags=" + String(h.flags) + "\n";
    hdr += "# triggered=" + String((h.flags & 0x01) ? 1 : 0) + "\n";
    hdr += "# truncated=" + String((h.flags & 0x02) ? 1 : 0) + "\n";
    hdr += "# pre_samples=" + String(h.pre_samples) + "\n";
    if (h.trig_mode != 0xFF)
    {
      hdr += "# trig_mode=" + String(h.trig_mode == 0 ? "auto" : "manual") + "\n";
      hdr += "# trig_mult=" + String(h.trig_mult) + "\n";
      hdr += "# threshold_used_mps2=" + String(h.threshold_used, 6) + "\n";
    }
    hdr += "# serial_no=" + String(h.serial_no) + "\n";
    hdr += "# device_name=" + String(h.device_name) + "\n";
    hdr += "# meas_point=" + String(h.meas_point) + "\n";
    hdr += "# scan_dir=" + String(h.scan_dir) + "\n";
    hdr += "# operator=" + String(h.operator_name) + "\n";
    hdr += "# notes=" + String(h.notes) + "\n";
  }
  hdr += "t_ms,ax_raw,ay_raw,az_raw\n";

  server.sendContent(hdr);

  const uint32_t dt_us = (h.rate_hz > 0) ? (1000000UL / h.rate_hz) : 1000;
  uint64_t t_us = 0;

  char line[96];
  Sample6 s;
  String chunk;
  chunk.reserve(2048);

  while (f.read((uint8_t *)&s, sizeof(s)) == sizeof(s))
  {
    uint32_t ms_int = (uint32_t)(t_us / 1000ULL);
    uint32_t ms_frac = (uint32_t)(t_us % 1000ULL);
    int n = snprintf(line, sizeof(line), "%lu.%03lu,%d,%d,%d\n",
                     (unsigned long)ms_int, (unsigned long)ms_frac,
                     (int)s.ax, (int)s.ay, (int)s.az);
    if (n > 0)
      chunk += String(line);

    t_us += dt_us;

    if (chunk.length() > 1800)
    {
      server.sendContent(chunk);
      chunk = "";
      delay(0);
    }
  }

  if (chunk.length())
    server.sendContent(chunk);
  server.sendContent("");
  f.close();
}

void handleApiDelete()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file");
    return;
  }

  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path))
  {
    server.send(400, "text/plain", "Bad file");
    return;
  }
  if (!fileExists(path))
  {
    server.send(404, "text/plain", "Not found");
    return;
  }

  LittleFS.remove(path);
  rebuildListCache();

  server.send(200, "text/plain", "Deleted");
}
