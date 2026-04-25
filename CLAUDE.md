# accMeter2 - Project Instructions

## Project Overview
ESP32 + LIS2DW12 MEMS accelerometer based semi-professional machine vibration meter and data logger.
Designed for ISO 20816 class vibration monitoring on rotating machinery (motors, pumps, fans, compressors).

## Current Version
`3.9.4`

## Tech Stack
- **MCU:** ESP32 (DOIT DevKit V1)
- **Sensor:** ST LIS2DW12 3-axis MEMS accelerometer (I2C, 0x18)
- **Framework:** PlatformIO + Arduino
- **RTOS:** FreeRTOS (tasks, mutexes, hardware timers)
- **Storage:** LittleFS (binary .dat recordings)
- **Connectivity:** WiFi STA + fallback AP mode
- **Web UI:** Embedded HTML/JS served from ESP32 WebServer
- **Analysis:** arduinoFFT v1.6.2

## Architecture

### Source Files
| File | Role |
|------|------|
| `src/main.cpp` | WiFi/AP setup, I2C init, server loop |
| `src/api_handlers.cpp` | All HTTP handlers, recording task, calibration, FFT, live preview, OTA |
| `src/api_handlers.h` | Public API: `registerRoutes()`, `rebuildListCache()`, `setRawMaskBits()` |
| `src/app_state.h` | Global state declarations, structs (FileHeaderV3, Sample6, RecConfig) |
| `src/app_state.cpp` | Global variable definitions |
| `src/html_pages.cpp` | Embedded HTML/CSS/JS (main UI + OTA update page) |
| `src/html_pages.h` | PROGMEM page declarations |
| `src/config.h` | APP_VERSION, BUILD_HASH macros |
| `src/config.cpp` | WiFi credentials (WIFI_SSID, WIFI_PASS) |
| `lib/LIS2DW12_ESP32/` | Custom sensor driver library (I2C register access, calibration, NVS) |

### Key Patterns
- **I2C mutex:** All sensor access protected by `g_i2cMutex` with 5s timeout (`pdMS_TO_TICKS(5000)`)
- **Recording:** FreeRTOS task (`recordTask`) with hardware timer interrupt for precise sample timing
- **File format:** Binary `FileHeaderV3` (48 bytes) + `Sample6` (6 bytes/sample) records
- **Live preview:** Cached sensor object (`g_liveSensor`) to avoid reinit on every `/api/live` call
- **HTTP methods:** State-changing endpoints use POST; read-only use GET
- **JS helpers:** `getText()` for GET, `postText()` for POST (form-urlencoded)
- **FFT arrays:** Heap-allocated (malloc/free), not static

### API Endpoints
| Method | Path | Purpose |
|--------|------|---------|
| GET | `/api/info` | Recording status, config |
| GET | `/api/list` | File list (cached) |
| GET | `/api/fsinfo` | LittleFS usage |
| GET | `/api/live` | Live preview (acc, vel, disp) |
| GET | `/api/version` | Firmware version/build info |
| GET | `/api/analyze` | Time-domain analysis of recording |
| GET | `/api/fft` | FFT analysis of recording |
| GET | `/download` | Download .dat binary |
| GET | `/download_csv` | Download as .csv |
| POST | `/api/start` | Start recording |
| POST | `/api/stop` | Stop recording |
| POST | `/api/delete` | Delete recording file |
| POST | `/api/calibrate_static` | Single-position calibration |
| POST | `/api/calibrate6` | 6-position calibration |
| POST | `/api/realtime` | Enable/disable ISO 20816 mode |
| POST | `/api/rawmask` | Set LSB noise masking bits |
| POST | `/api/reset` | Factory reset (clear NVS + files) |
| POST | `/update` | OTA firmware upload |

## Build & Upload
```bash
pio run                    # compile
pio run -t upload          # flash via USB (COM3)
pio device monitor         # serial monitor
```

## Conventions
- Versioning follows `versiyonlama.md` (MAJOR.MINOR.RevID)
- Version string updated in `src/config.h` (`APP_VERSION`) and this file
- Changelog maintained in `changelog.md` (newest first)
- Communication language: Turkish
- Commit messages: English
- No test suite (hardware-dependent, verified on device)

## Important Notes
- `src/config.cpp` contains WiFi credentials - do NOT commit real passwords
- LittleFS partition: 4MB with OTA (`partitions_4mb_ota_littlefs.csv`)
- Sensor max rates: High-Perf 1600Hz, Low-Power 200Hz
- Recording files: `/accelYYMMDDHHMMSS.dat`
- I2C pins: SDA=21, SCL=22, 400kHz
