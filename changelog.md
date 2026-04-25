---------------------------------------------------------
Rev. ID    : 7
Rev. Date  : 25.04.2026
Rev. Time  : 10:41:29
Rev. Prompt: Faz 5D - UI bolunmesi (Grab Mode default + Live View ikinci sayfa)

Rev. Report: (
Web arayuzu iki ayri sayfaya bolundu. Default sayfa "Grab Mode" - olcum
metadata'si girip motion-triggered kayit baslatma odakli. Ikinci sayfa
"Live View" - mevcut canli onizleme + dosya analizi + kalibrasyon vb.

- Web UI: GRAB_HTML yeni sayfa (~14 KB), default route "/" buraya gider
    Bolumler:
    - Device: serial (auto), name (NVS'de persisted, inline kaydet),
      kalibrasyon rozeti (CAL/UNCAL)
    - Measurement Metadata: tarih/saat (auto from browser), olcum noktasi
      (zorunlu), tarama yonu (Radial-H/Radial-V/Axial), operator (opsiyonel),
      notes (opsiyonel)
    - Acquisition Settings: hz (100..1600), fs (2/4/8/16 g), max duration
      (15..120 s)
    - Trigger: pre-roll (1..10 s), post-roll (1..30 s), threshold mode
      (Auto baseline x N veya Manual m/s^2)
    - Action: START GRAB (ARM), DISARM
    - Status: state metni (IDLE/ARMED/TRIGGERED/POST-TAIL) + canli detay
      (baseline, threshold, current RMS, elapsed)
    - Last saved: kayit bitince dosya adi + Live View linki
- Web UI: INDEX_HTML -> LIVE_HTML olarak rename edildi; uste navigation bar
  eklendi (Grab Mode | Live View | Firmware Update). Versiyon banner'da
  /api/version'dan cekiliyor.
- API: /api/info polling Grab Mode'da 1s interval; trigger durumu canli
  takip ediliyor. Idle'a donusunde otomatik "Saved: ..." toast.
- Routing: "/" -> GRAB_HTML, "/live" -> LIVE_HTML, "/update" -> UPDATE_HTML
  (degismedi).
- HTML form -> /api/trigger_arm: tum params (hz, fs, max_s, pre_s, post_s,
  mode, mult, abs_thr, ts, meas_point, scan_dir, operator, notes).
  Browser-side validation: meas_point ve scan_dir zorunlu.

NOT: Live View'da manuel record icin metadata input henuz yok (5E polish'te
gelir). Live View'daki manuel /api/start cagrisi su an metadata gondermiyor;
g_recMeta bos string'lerle dolduruluyor.

Build: pio run -> SUCCESS, RAM 14.1%, Flash 71.5% (+14272 byte vs V3.9.6).
- Firmware: APP_VERSION V3.9.7
)
---------------------------------------------------------
Rev. ID    : 6
Rev. Date  : 25.04.2026
Rev. Time  : 10:37:02
Rev. Prompt: Faz 5C - trigger ("grab") mode firmware (state machine + ring buffer + RMS detector)

Rev. Report: (
"Ready-to-grab" akilli kayit modu firmware tarafinda tamam. Cihaz hareket
algilayinca pre-roll ile birlikte otomatik kayit baslatir, sabit gelinceye
kadar kaydeder, sonrasinda post-roll suresince devam edip kapatir.

- Firmware: yeni global'ler (app_state.h/cpp):
    enum TrigState (Idle, Armed, Triggered, PostTail) + g_trigState
    g_trigPreS / g_trigPostS / g_trigMaxS / g_trigMode / g_trigMult /
    g_trigManualThr  (kullanici parametreleri)
    g_trigBaseline / g_trigCurrentRms / g_trigEffThreshold (canli durum)
    g_trigArmedAtMs / g_trigFiredAtMs / g_trigPostStartMs (zaman damgalari)
    g_trigDisarmRequested (clean shutdown sinyali)
- Recording: recording.cpp'deki HW timer ortak helper'lara cikarildi
    (consumeTimerDue, resetTimerDue) - trigger task ayni zamanlama
    altyapisini paylasiyor. Manual record + grab mode mutex.
- Sensor: trigger.cpp icinde state machine:
    BASELINE (1 s) -> ortam gurultusunden esik turetilir (auto x N modu)
    ARMED -> 100 ms pencerelerde AC RMS hesabi; 3 ardisik pencere esigi
             asarsa TRIGGERED'a gec; 5 dk timeout veya disarm ile kapan
    TRIGGERED -> dosya acilir (/grabYYMMDDHHMMSS.dat), pre-roll buffer
             kronolojik sirayla yazilir, live data devam eder; 5 ardisik
             pencere esik altinda ise POST_TAIL'a gec; max_s asilirsa
             truncated flag set + POST_TAIL'a gec
    POST_TAIL -> post_s saniye daha kayit, sonra dosya kapanir
- Firmware: pre-roll ring buffer heap'te (pre_s x hz x 6 byte); 100 ms
  detection window heap'te (3 x N x float). 1600 Hz x 3 s = 28.8 KB +
  100 ms x 1600 Hz x 12 byte = 1.9 KB toplam ~31 KB; ESP32 RAM bol.
- API: yeni endpoint'ler
    POST /api/trigger_arm    hz, fs, max_s, pre_s, post_s, mode, mult,
                             abs_thr, ts, meas_point, scan_dir, operator,
                             notes -> 200 "OK ARMED"
    POST /api/trigger_disarm -> 200 "OK disarm requested"
- API: /api/info JSON'a trigger alanlari eklendi:
    trigState (string), trigBaseline, trigCurrentRms, trigThreshold,
    trigArmedMs, trigFiredMs
- Firmware: tum busy check'lere (recording, calibration, OTA, live
  preview, reset, device-name save) g_trigState != Idle eklendi;
  state'ler birbirini kilitliyor.
- Sensor: dosya v4 header'a yazilirken trigger info doldurulur
  (pre_samples, threshold_used, trig_mode, trig_mult, flags bit0=triggered,
  bit1=truncated). Header sonradan rewrite ediliyor (samples count + flags).
- Filename: trigger recording'leri /grabYYMMDDHHMMSS.dat (manuel record
  /accel... olarak kaliyor); isSafeAccelFile() ikisini de kabul ediyor.

NOT: UI henuz bolunmedi (5D'de gelir). Su anki UI sadece manual recording
gosteriyor; trigger mode curl/Postman ile test edilebilir:
  curl -X POST -d "hz=800&fs=2&max_s=60&pre_s=3&post_s=5&mode=0&mult=3
                   &ts=260425103700&meas_point=test&scan_dir=RADIAL_H" \
       http://<ip>/api/trigger_arm

Build: pio run -> SUCCESS, RAM 14.1%, Flash 70.4% (+9876 byte vs V3.9.5).
- Firmware: APP_VERSION V3.9.6
)
---------------------------------------------------------
Rev. ID    : 5
Rev. Date  : 25.04.2026
Rev. Time  : 10:28:41
Rev. Prompt: Faz 5B - device identity (serial + isim) NVS persistence

Rev. Report: (
- Firmware: loadDeviceIdentity() boot'ta cagrilip g_device_serial'i MAC'in
  son 12 hex karakteriyle, g_device_name'i ise NVS namespace="device"
  key="name"'den dolduruyor. Bos durumda default "accMeter-XXXXXX"
  (XXXXXX = MAC son 6 hex). Ilk acilista her cihaz benzersiz isim alir.
- Firmware: saveDeviceName() NVS'ye yazip global'i guncelleyen helper.
- API: GET /api/device -> {serial, name}
- API: POST /api/device name=... -> NVS save (isim 1..23 karakter)
- API: /api/info JSON cevabi "serial" ve "deviceName" alanlarini iceriyor
  (UI bu degerleri header'da goruntulemeli).
- Recording: V4 header bu degerleri alip dosyaya yaziyor (5A'da hazirlanmisti).

Build: pio run -> SUCCESS, RAM 14.1%, Flash 69.7% (+2328 byte vs V3.9.4).
- Firmware: APP_VERSION V3.9.5
)
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
