---------------------------------------------------------
Rev. ID    : 4
Rev. Date  : 25.04.2026
Rev. Time  : 10:21:27
Rev. Prompt: Faz 5A - file header v4 + parsed header infrastructure (grab mode hazirligi)

Rev. Report: (
Faz 5A - dosya formati v4'e bump edildi. Trigger info + per-recording
metadata header'a eklendi; v3 dosyalar geriye donuk uyumlu okunuyor.

- Firmware: FileHeaderV4 (214 byte) tanimlandi (app_state.h). V3 (46 byte)
  legacy olarak korunuyor, sadece okuma. Yeni eklenen alanlar:
    flags (bit0 triggered, bit1 truncated)
    pre_samples, threshold_used, trig_mode, trig_mult
    serial_no[16], device_name[24], meas_point[24], scan_dir[12],
    operator_name[16], notes[64]
  static_assert ile boyutlar derleme aninda dogrulaniyor.
- Firmware: ParsedHeader version-agnostic struct + readParsedHeader()
  helper eklendi (sensor_utils). V3 ve V4 tek kapidan akiyor.
- Firmware: Yeni global'ler:
    g_recMeta (RecMetadata) - per-recording metadata buffer
    g_device_serial[16], g_device_name[24] - 5B'de doldurulacak
- Sensor: rewriteHeaderSamples() artik hem V3 hem V4 dosyalari guncelliyor
  (versiyona gore branch). Mevcut V3 kayit dosyalarini legal halde tutar.
- API: handleApiAnalyze + handleApiFFT + CSV download artik
  readParsedHeader kullaniyor; V3 ve V4 dosyalar ayni kapidan akiyor.
- API: CSV header preamble V4 dosyalarda metadata satirlarini emit ediyor
  (# meas_point=, # scan_dir=, # serial_no=, vb.).
- Sensor: isSafeAccelFile() hem /accel*.dat (manuel) hem /grab*.dat
  (trigger) kabul ediyor; her ikisi de strict format dogrulamasi.
- Recording: recordTask V3 yerine V4 yaziyor; manual recording'de
  trig_mode = 0xFF (N/A), pre_samples = 0.
- Build: pio run -> SUCCESS, RAM 14.1%, Flash 69.5% (+3964 byte vs V3.9.3).
- Recording max sec'i UI dropdown'unda 180 -> 120 olarak guncellendi
  (recording.cpp allowedSec[] ile birlikte; 1600 Hz x 180 s LittleFS'e
   sigmiyordu).

NOT: Bu commit sadece infrastructure. Trigger mode logic, device identity
NVS yuklemesi, UI split sonraki fazlarda (5B-5E) gelir.

- Firmware: APP_VERSION V3.9.4
)
---------------------------------------------------------
Rev. ID    : 3
Rev. Date  : 25.04.2026
Rev. Time  : 09:55:25
Rev. Prompt: Faz 4 (fizik duzeltmeleri) - vel/disp DC sizintisi + olculmus dt; tum cikti RMS

Rev. Report: (
Faz 4 - canli onizleme ve realtime ISO20816 yolunda fizik dogrulugu
iyilestirildi. BREAKING: ax/ay/az ve vx/vy/vz, dx/dy/dz alanlari artik
ANLIK ortalama yerine AC RMS degeri donuyor.

- API: /api/live cevap semantigi degisti (BREAKING).
  Eski: ax = pencere ortalamasi (DC + AC karisik); vel/disp = pencere sonu
        kumulatif degeri.
  Yeni: ax = AC RMS m/s^2 (mean-removed, LPF sonrasi RMS).
        vx_mmps = vel RMS mm/s (3-eksen, ISO 20816 uyumlu).
        dx_mm = disp RMS mm.
- API: Yeni alanlar /api/live cevabinda: "dt_us" (gercek olculen ornekleme
  periyodu, mikrosaniye), "eff_hz" (1e6/dt_us). Hem non-realtime hem
  realtime kolu doner.
- Sensor: handleApiLive 200 ornek heap buffer kullaniyor, t_start/t_end
  micros() ile gercek dt olculuyor. Eski sabit dt = 1/800 (=1.25 ms)
  varsayimi vs. gercek ~1.5-1.8 ms hatasi giderildi.
- Sensor: Per-axis mean (DC removal) entegrasyondan once uygulaniyor;
  gravity tilt'in vel/disp'e yapay sizinti yapmasi durduruldu (eski:
  0.05g residual -> 250 ms'de 122 mm/s yapay vel).
- Sensor: measureRealtimeNoise() artik proper mean-removed AC RMS
  hesabi yapiyor (eski: ||a|| - g, kucuk tilt icin yetersiz).
- Sensor: Realtime ISO20816 yolu artik per-axis mean-removed integrasyon
  yapip 3-eksen vector magnitude RMS doner. Noise floor cikarmasi
  quadrature mantigi ile: rms = sqrt(rms_total^2 - rms_noise^2).
- Web UI: "Live RMS (250 ms window @800 Hz target)" basligi; her satirda
  "RMS" ibaresi; gercek ornekleme hizi (eff_hz) ve dt_us kucuk fontla
  gosteriliyor.
- Firmware: g_live_dt_us yeni global eklendi (app_state.h/cpp);
  resetLivePreviewState bunu da sifirliyor.
- Firmware: APP_VERSION V3.9.3

Build: pio run -> SUCCESS, RAM 14.1%, Flash 69.2% (+3012 byte vs V3.9.2,
RMS hesabi + heap buffer logic).
)
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
