---------------------------------------------------------
Rev. ID    : 2
Rev. Date  : 25.04.2026
Rev. Time  : 09:45:12
Rev. Prompt: Faz 3 (UX/davranis duzeltmeleri) uygulamasi

Rev. Report: (
Faz 3 - kullanici davranisi/tutarlilik iyilestirmeleri.

- Firmware: g_rawMaskBits default 3 -> 0; LSB maskeleme artik kullanici
  acikca secmeden uygulanmiyor. UI dropdown ve JS state default'u da senkron.
- Sensor: isSafeAccelFile() siki dogrulama. Yalniz "/accelYYMMDDHHMMSS.dat"
  veya "/accelYYMMDDHHMMSS_NN.dat" formatinda kabul; "/accel.dat",
  "/accelABC.dat" gibi yanlis isimler reddediliyor.
- Firmware: makeNewFileNameFromUI() 99 ayni-saniye cakismasinda artik
  millis() fallback'i yerine bos string donuyor; recordTask bunu yakalayip
  temiz cikiyor (UI'da bozuk dosya adi olusmaz).
- API: /api/rawmask lambda -> handleApiRawMask() ayri fonksiyon
  (file_handlers.cpp/h); diger handler'larla stil tutarli.
- Sensor + API: Calibration NVS blob'u boot'ta yukleniyor (loadCalibrationAtBoot),
  g_calibPresent global flag'i set ediliyor, /api/info JSON'a "calibrated"
  alani eklendi. recordTask NVS yukleme basarisizliginda log basiyor.
- Web UI: Status barina "CAL"/"UNCAL" rozeti eklendi; uncalibrated cihaz
  bariz sekilde isaretleniyor.
- Firmware: APP_VERSION V3.9.2

Build: pio run -> SUCCESS, RAM 14.1%, Flash 69.0% (+508 byte vs V3.9.1).
)
---------------------------------------------------------
Rev. ID    : 1
Rev. Date  : 25.04.2026
Rev. Time  : 09:37:30
Rev. Prompt: Kodlari detayli analiz et, hatali gordugun yerleri duzeltmek icin planlama yap

Rev. Report: (
Faz 1 (kritik fonksiyonel buglar) ve Faz 2 (versiyon/repo duzeni) uygulandi.

- Build: lib/LIS2DW12_ESP32/library..json -> library.json (cift nokta typo);
  PlatformIO metadata'si dogru yuklenecek
- Build: platformio.ini'den `-D APP_VERSION` build flag kaldirildi;
  tek surum kaynagi src/config.h oldu (V3.7 vs V3.6 vs 3.9.0 cakismasi giderildi)
- Build: upload_port = COM3 / monitor_port = COM3 yorumlandi
  (cross-platform; macOS/Linux'da auto-detect)
- Sensor: Wire.setClock(1000000) -> 400000 (recording.cpp + live_preview.cpp);
  LIS2DW12 datasheet max 400 kHz Fast Mode, 1 MHz spec ihlali idi
- API: handleApiFFT artik dosya open / size / header read / magic kontrolu yapiyor
  (handleApiAnalyze ile asimetri giderildi); recording sirasinda 409 dondurur
- API: CSV download `t_ms` 1600 Hz'de 0'a duvarlaniyordu (integer truncation);
  mikrosaniye tabani uzerinden hesaplanip "ms.uuu" formatinda yaziliyor
- Firmware: src/olides/ legacy snapshot'lari (main.cV2, main.V3_6, *.x)
  archive/legacy/olides/ altina tasindi; src/ daha temiz
- Firmware: APP_VERSION V3.9.1'e bump edildi
)
---------------------------------------------------------
