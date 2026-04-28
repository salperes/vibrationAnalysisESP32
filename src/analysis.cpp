#include <Arduino.h>
#include <LittleFS.h>
#include <arduinoFFT.h>

#include "analysis.h"
#include "sensor_utils.h"

#define FFT_N 1024
#define FFT_WINDOW FFT_WIN_TYP_HANN

// ======================= Time-domain analysis =======================
void handleApiAnalyze()
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

  if (g_recording)
  {
    server.send(409, "text/plain", "Recording in progress");
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

  const uint32_t headerBytes = h.header_bytes;
  const uint32_t sampleBytes = sizeof(Sample6);
  const uint32_t maxPossibleSamples = (uint32_t)((f.size() - headerBytes) / sampleBytes);
  uint32_t n = h.samples;
  if (n == 0 || n > maxPossibleSamples)
    n = maxPossibleSamples;

  const uint32_t MAXPTS = 2000;
  uint32_t pts = (n <= MAXPTS) ? n : MAXPTS;
  if (pts < 2)
    pts = n;

  const double step = (pts > 0) ? ((double)n / (double)pts) : 1.0;

  float *sumX = (float *)calloc(pts, sizeof(float));
  float *sumY = (float *)calloc(pts, sizeof(float));
  float *sumZ = (float *)calloc(pts, sizeof(float));
  uint16_t *cnt = (uint16_t *)calloc(pts, sizeof(uint16_t));

  if (!sumX || !sumY || !sumZ || !cnt)
  {
    if (sumX) free(sumX);
    if (sumY) free(sumY);
    if (sumZ) free(sumZ);
    if (cnt) free(cnt);
    f.close();
    server.send(500, "text/plain", "OOM");
    return;
  }

  float minX = +INFINITY, minY = +INFINITY, minZ = +INFINITY;
  float maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;
  double ssX = 0, ssY = 0, ssZ = 0;

  Sample6 s{};
  uint32_t i = 0;
  while (i < n && f.read((uint8_t *)&s, sizeof(s)) == sizeof(s))
  {
    float gx = rawAlignedToG(s.ax, h.res_bits, h.fs_g);
    float gy = rawAlignedToG(s.ay, h.res_bits, h.fs_g);
    float gz = rawAlignedToG(s.az, h.res_bits, h.fs_g);

    gx = applyCal1(gx, h.cal_offset_g[0], h.cal_scale[0]);
    gy = applyCal1(gy, h.cal_offset_g[1], h.cal_scale[1]);
    gz = applyCal1(gz, h.cal_offset_g[2], h.cal_scale[2]);

    if (gx < minX) minX = gx;
    if (gx > maxX) maxX = gx;
    ssX += (double)gx * (double)gx;
    if (gy < minY) minY = gy;
    if (gy > maxY) maxY = gy;
    ssY += (double)gy * (double)gy;
    if (gz < minZ) minZ = gz;
    if (gz > maxZ) maxZ = gz;
    ssZ += (double)gz * (double)gz;

    uint32_t b = (pts <= 1) ? 0 : (uint32_t)floor((double)i / step);
    if (b >= pts) b = pts - 1;
    sumX[b] += gx;
    sumY[b] += gy;
    sumZ[b] += gz;
    if (cnt[b] < 65535) cnt[b]++;

    i++;
    if ((i & 0xFF) == 0) delay(0);
  }
  f.close();

  const uint32_t usedN = i;
  const float rmsX = rmsFromSumSq(ssX, usedN);
  const float rmsY = rmsFromSumSq(ssY, usedN);
  const float rmsZ = rmsFromSumSq(ssZ, usedN);

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Type", "application/json");
  server.sendHeader("Connection", "close");
  server.send(200);

  String head;
  head.reserve(1024);
  head += "{";
  head += "\"file\":\"" + jsonEscape(path) + "\",";
  head += "\"rate_hz\":" + String(h.rate_hz) + ",";
  head += "\"record_s\":" + String(h.record_s) + ",";
  head += "\"samples_header\":" + String(h.samples) + ",";
  head += "\"samples_used\":" + String(usedN) + ",";
  head += "\"fs_g\":" + String(h.fs_g) + ",";
  head += "\"res_bits\":" + String(h.res_bits) + ",";
  head += "\"q_bits\":" + String(h.q_bits) + ",";
  head += "\"min\":[" + String(minX, 6) + "," + String(minY, 6) + "," + String(minZ, 6) + "],";
  head += "\"max\":[" + String(maxX, 6) + "," + String(maxY, 6) + "," + String(maxZ, 6) + "],";
  head += "\"rms\":[" + String(rmsX, 6) + "," + String(rmsY, 6) + "," + String(rmsZ, 6) + "],";
  head += "\"pts\":" + String(pts) + ",";
  float effHz = (pts > 1 && usedN > 1) ? (float)h.rate_hz * ((float)pts / (float)usedN) : (float)h.rate_hz;
  head += "\"eff_hz\":" + String(effHz, 4) + ",";
  head += "\"ax\":[";
  server.sendContent(head);

  auto sendFloatArray = [&](float *sum, uint16_t *c, bool closeArr, const char *nextKey)
  {
    String chunk;
    chunk.reserve(2048);
    char buf[16];
    for (uint32_t k = 0; k < pts; k++)
    {
      float v = (c[k] ? (sum[k] / (float)c[k]) : 0.0f);
      snprintf(buf, sizeof(buf), "%.6f", v);
      chunk += buf;
      if (k + 1 < pts) chunk += ",";
      if (chunk.length() > 1800)
      {
        server.sendContent(chunk);
        chunk = "";
        delay(0);
      }
    }
    if (chunk.length()) server.sendContent(chunk);

    if (closeArr)
    {
      server.sendContent("]");
      if (nextKey)
      {
        server.sendContent(String(",\"") + nextKey + "\":[");
      }
    }
  };

  sendFloatArray(sumX, cnt, true, "ay");
  sendFloatArray(sumY, cnt, true, "az");
  sendFloatArray(sumZ, cnt, true, nullptr);

  server.sendContent("}");
  server.sendContent("");

  free(sumX);
  free(sumY);
  free(sumZ);
  free(cnt);
}

// ======================= FFT analysis =======================
void handleApiFFT()
{
  if (!server.hasArg("file") || !server.hasArg("axis"))
  {
    server.send(400, "text/plain", "Missing file or axis");
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

  if (g_recording)
  {
    server.send(409, "text/plain", "Recording in progress");
    return;
  }

  String axisArg = server.arg("axis");
  if (axisArg.length() == 0)
  {
    server.send(400, "text/plain", "Bad axis");
    return;
  }
  char axis = axisArg[0];
  int axisIdx = (axis == 'x') ? 0 : (axis == 'y') ? 1
                                : (axis == 'z')   ? 2
                                                  : -1;
  if (axisIdx < 0)
  {
    server.send(400, "text/plain", "Bad axis");
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

  const uint32_t headerBytes = h.header_bytes;
  const uint32_t sampleBytes = sizeof(Sample6);
  const uint32_t maxPossibleSamples = (uint32_t)((f.size() - headerBytes) / sampleBytes);
  uint32_t headerSamples = h.samples;
  if (headerSamples == 0 || headerSamples > maxPossibleSamples)
    headerSamples = maxPossibleSamples;

  uint32_t maxSamples = min((uint32_t)FFT_N, headerSamples);
  // Round down to nearest power of 2 (arduinoFFT requires it)
  {
    uint32_t p2 = 1;
    while (p2 * 2 <= maxSamples) p2 *= 2;
    maxSamples = p2;
  }
  if (maxSamples < 16)
  {
    f.close();
    server.send(400, "text/plain", "Too few samples");
    return;
  }

  double *vReal = (double *)malloc(FFT_N * sizeof(double));
  double *vImag = (double *)calloc(FFT_N, sizeof(double));
  if (!vReal || !vImag)
  {
    free(vReal);
    free(vImag);
    f.close();
    server.send(500, "text/plain", "OOM");
    return;
  }

  Sample6 s;
  for (uint32_t i = 0; i < maxSamples; i++)
  {
    if (f.read((uint8_t *)&s, sizeof(s)) != sizeof(s))
      break;

    int16_t raw =
        axisIdx == 0 ? s.ax : axisIdx == 1 ? s.ay
                                           : s.az;

    float g = rawAlignedToG(raw, h.res_bits, h.fs_g);
    g = applyCal1(g, h.cal_offset_g[axisIdx], h.cal_scale[axisIdx]);

    vReal[i] = g;
  }
  f.close();

  arduinoFFT FFT(vReal, vImag, maxSamples, h.rate_hz);
  FFT.Windowing(FFT_WINDOW, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  // Scale by 2/N for single-sided amplitude spectrum (g amplitude)
  const double fftScale = 2.0 / (double)maxSamples;
  uint32_t bins = maxSamples / 2;
  double df = (double)h.rate_hz / (double)maxSamples;

  double peakMag = 0;
  double peakHz = 0;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json");

  server.sendContent("{");
  server.sendContent("\"axis\":\"");
  server.sendContent(String(axis));
  server.sendContent("\",\"rate_hz\":");
  server.sendContent(String(h.rate_hz));
  server.sendContent(",\"df\":");
  server.sendContent(String(df, 6));
  server.sendContent(",\"fft\":[");

  for (uint32_t i = 1; i < bins; i++)
  {
    double mag = vReal[i] * fftScale;
    double hz = i * df;

    if (mag > peakMag)
    {
      peakMag = mag;
      peakHz = hz;
    }

    server.sendContent(String(mag, 6));
    if (i + 1 < bins)
      server.sendContent(",");
  }

  server.sendContent("],");
  server.sendContent("\"peak_hz\":");
  server.sendContent(String(peakHz, 3));
  server.sendContent(",");
  server.sendContent("\"peak_mag\":");
  server.sendContent(String(peakMag, 6));
  server.sendContent("}");

  free(vReal);
  free(vImag);
}
