# Functional Requirements Document (FRD)
# accMeter2 - Yari Profesyonel Makine Titresim Olcer ve Loglama Cihazi

**Proje:** accMeter2
**Donanim:** ESP32 + LIS2DW12 MEMS Accelerometer
**Tarih:** 08.03.2026
**Versiyon:** 1.0

---

## 1. Amac ve Kapsam

### 1.1 Amac
Donen makinalarin (motor, pompa, fan, kompresor vb.) titresim seviyelerini olcen, kaydeden ve analiz eden tasinabilir, WiFi baglantili, yari profesyonel bir titresim olcum ve loglama cihazi gelistirmek.

### 1.2 Hedef Kullanici
- Sanayi bakim teknisyenleri
- Makine muhendisleri
- Prediktif bakim uygulayicilari

### 1.3 Kullanim Senaryolari
- Makine uzerinde periyodik titresim olcumu
- Uzun sureli titresim kaydi (veri loglama)
- Anlamsiz frekans analizi (FFT)
- ISO 20816 siniflandirmasina gore makine durum degerlendirmesi

---

## 2. Donanim Gereksinimleri

### 2.1 Islemci
| Ozellik | Deger |
|---------|-------|
| MCU | ESP32 (Dual-core Xtensa LX6, 240 MHz) |
| RAM | 520 KB SRAM |
| Flash | 4 MB (OTA + LittleFS paylasimli) |
| WiFi | 802.11 b/g/n (STA + AP modu) |

### 2.2 Sensor
| Ozellik | Deger |
|---------|-------|
| Model | ST LIS2DW12 |
| Tip | 3-eksenli MEMS ivmeolcer |
| Arayuz | I2C (adres: 0x18) |
| Olcum araligi | +-2g, +-4g, +-8g, +-16g (secimli) |
| Cozunurluk | 12-bit veya 14-bit |
| Ornekleme hizi | 1.6 Hz - 1600 Hz |
| Calima modlari | High Performance, Low Power |
| Bant genisligi filtresi | ODR/2, ODR/4, ODR/10, ODR/20 |
| Gurultu yogunlugu | 90 ug/sqrt(Hz) @ 14-bit HP |

### 2.3 Baglanti
| Ozellik | Deger |
|---------|-------|
| I2C pinleri | SDA=GPIO21, SCL=GPIO22 |
| I2C hiz | 400 kHz (Fast Mode) |
| USB | Mikro-USB (programlama + seri monitor) |

---

## 3. Fonksiyonel Gereksinimler

### 3.1 Sensor Yonetimi

#### FR-3.1.1 Sensor Baslangic
- Sistem acilista I2C uzerinden LIS2DW12 sensor ile iletisim kurar
- WHO_AM_I register'i (0x0F) okunarak sensor dogrulanir (beklenen: 0x44)
- BDU (Block Data Update) ve IF_ADD_INC (auto increment) etkinlestirilir

#### FR-3.1.2 Sensor Konfigurasyonu
- Kullanici web arayuzunden asagidaki parametreleri secebilir:
  - **Ornekleme hizi (Hz):** 1.6, 12.5, 25, 50, 100, 200, 400, 800, 1600
    - UI'da tamsayi olarak gonderilir: 2→1.6 Hz (LP ozel), 13→12.5 Hz; diger degerler birebir
  - **Tam olcek (g):** 2, 4, 8, 16
  - **Calima modu:** High Performance / Low Power
- Konfigrasyon kayit baslatildiginda sensore uygulanir

#### FR-3.1.3 I2C Mutex
- Tum sensor erisimi FreeRTOS mutex ile korunur
- Mutex timeout: 5 saniye (deadlock onleme)
- Kayit task'i, kalibrasyon task'i ve live preview ayni anda sensor erisemez

### 3.2 Veri Kaydi (Recording)

#### FR-3.2.1 Kayit Baslatma
- Kullanici web arayuzunden kayit baslatir (POST `/api/start`)
- Parametreler: ornekleme hizi (Hz), tam olcek (g), sure (saniye), zaman damgasi
- Ayri bir FreeRTOS task'i (`recordTask`) olusturulur
- Donanim timer kesme ile hassas zamanlama saglanir

#### FR-3.2.2 Kayit Formati
- Ikili (binary) dosya formati:
  - **Header (FileHeaderV3, 48 byte):**
    - Magic: "LIS2DW12" (8 byte)
    - Versiyon: 3
    - Ornekleme hizi, kayit suresi, toplam ornekler
    - Tam olcek (g), cozunurluk (bit), quantization (bit)
    - Kalibrasyon verileri (offset + scale, 3 eksen)
  - **Veri (Sample6, 6 byte/ornek):**
    - ax, ay, az (int16 hizalanmis ham degerler)
- Dosya adi: `/accelYYMMDDHHMMSS.dat`
- Cakisma varsa `_01`, `_02` ... eki eklenir

#### FR-3.2.3 Kayit Durdurma
- Kayit belirlenen surede otomatik durur veya kullanici manuel durdurur (POST `/api/stop`)
- Header'daki `samples` alani gercek yazilan ornekler ile guncellenir
- Dosya listesi cache'i yenilenir

#### FR-3.2.4 Kayit Durumu
- Kayit ilerlemesi web arayuzunden izlenebilir (GET `/api/info`)
- Gosterilen bilgiler: ornekler, gecen sure, max backlog, dosya adi

### 3.3 Dosya Yonetimi

#### FR-3.3.1 Dosya Listeleme
- Kayitli dosyalar listelenebilir (GET `/api/list`)
- JSON formatinda: dosya adi + boyut
- Cache mekanizmasi: 5 saniye TTL

#### FR-3.3.2 Dosya Indirme
- Ham ikili dosya indirilebilir (GET `/download?file=...`)
- CSV formatinda disari aktarim (GET `/download_csv?file=...`)
- CSV header'inda: ornekleme hizi, kalibrasyon, metadata
- CSV sutunlari: `t_ms, ax_raw, ay_raw, az_raw`

#### FR-3.3.3 Dosya Silme
- Tekil dosya silinebilir (POST `/api/delete`)
- Guvenlik: yalnizca `/accel*.dat` dosyalari silinebilir (path traversal korunmasi)

#### FR-3.3.4 Depolama Bilgisi
- Toplam/kullanilan/bos alan sorgulanabilir (GET `/api/fsinfo`)

### 3.4 Kalibrasyon

#### FR-3.4.1 Statik Kalibrasyon (Tek Pozisyon)
- Cihaz duz yuzey uzerine yerlestirilir (Z+ yukari)
- 600 ornek ortalamasiyla offset ve scale hesaplanir
- Z ekseni referansi: 1g (yercekim)
- Sonuc NVS'ye kaydedilir (POST `/api/calibrate_static`)

#### FR-3.4.2 6-Pozisyon Kalibrasyon
- 6 yonlendirme adimi (X+, X-, Y+, Y-, Z+, Z-)
- Her pozisyonda 700 ornek toplanir
- 3 eksen icin bagimsiz offset ve scale hesabi
- Sonuc NVS'ye kaydedilir (POST `/api/calibrate6`)
- Web arayuzunden adim adim yonlendirilir

#### FR-3.4.3 Kalibrasyon Surekliligi
- Kalibrasyon verileri ESP32 NVS (Non-Volatile Storage) icinde saklanir
- Acilista otomatik yuklenir
- Fabrika sifirlamayla temizlenebilir (POST `/api/reset`)

### 3.5 Canli Onizleme (Live Preview)

#### FR-3.5.1 Canli Olcum
- Cihazdan anlik titresim verileri okunabilir (GET `/api/live`)
- 200 ornek @800Hz (~250ms) toplanip islenir
- Gosterilen degerler:
  - **Ivme (acceleration):** 3 eksen + buyukluk (g ve m/s2)
  - **Hiz (velocity):** 3 eksen + buyukluk (mm/s) - zaman entegrasyonu
  - **Yer degistirme (displacement):** 3 eksen + buyukluk (mm) - cift entegrasyon

#### FR-3.5.2 Dusuk Gecis Filtresi
- Configurable low-pass filtre kesim frekansi (varsayilan: 200 Hz)
- Onizleme verilerine uygulanir

#### FR-3.5.3 LSB Gurultu Maskeleme
- Sensor ham verilerinin alt bitlerini maskeleyerek gurultu azaltma
- Secenekler: Kapal (0), 2-bit, 3-bit, 4-bit maskeleme
- POST `/api/rawmask` ile ayarlanir

### 3.6 Gercek Zamanli ISO 20816 Izleme

#### FR-3.6.1 Realtime Mod
- Surekli titresim izleme modu (POST `/api/realtime`)
- Enable/disable kontrolu
- Aktifken periyodik olarak olcum yapilir

#### FR-3.6.2 Gurultu Tabanl
- Mod aktif edildiginde ortam gurultu seviyesi olculur
- Olcumlerde gurultu esigi referans olarak kullanilir

#### FR-3.6.3 Tarihce
- Son 10 olcum sonucu bellekte tutulur
- Ivme, hiz, yer degistirme RMS degerleri
- Trend analizi icin web arayuzune sunulur

### 3.7 Frekans Analizi (FFT)

#### FR-3.7.1 FFT Hesaplama
- Kayitli dosyalar uzerinde FFT analizi (GET `/api/fft`)
- Parametreler: dosya yolu, eksen (x/y/z)
- FFT boyutu: 1024 nokta (Hann penceresi)
- Cikti: frekans spektrumu, baskin frekans, baskin buyukluk

#### FR-3.7.2 Zaman Alanl Analiz
- Kayitli dosyalarda istatistiksel analiz (GET `/api/analyze`)
- Hesaplanan degerler: min, max, ortalama, RMS (3 eksen)
- Downsampling destegi (buyuk dosyalar icin)

### 3.8 Baglanti

#### FR-3.8.1 WiFi STA Modu
- Onceden tanimlanmis SSID/sifre ile WiFi agina baglanir
- Baglanti timeout: 12 saniye
- DHCP ile IP adresi alinir

#### FR-3.8.2 WiFi AP Fallback
- STA baglantisi basarisizsa otomatik AP moduna gecer
- SSID: `LIS2DW12-XXXXXX` (MAC adresinden turetilir)
- Sifresiz acik erisim noktasi
- IP: 192.168.4.1

#### FR-3.8.3 HTTP Sunucu
- Port 80 uzerinden web sunucu
- REST API (JSON format)
- Embedded web arayuzu (HTML/CSS/JS)

### 3.9 Web Arayuzu

#### FR-3.9.1 Ana Sayfa
- Kayit baslatma/durdurma kontrolleri
- Parametre secimi (Hz, g, sure)
- Kayit ilerlemesi
- Canli onizleme gorunumu
- Dosya listesi (indir, CSV, sil, FFT)
- Kalibrasyon kontrolleri
- ISO 20816 realtime panel

#### FR-3.9.2 Firmware Guncelleme Sayfasi
- OTA (Over-The-Air) firmware yukleme (`/update`)
- .bin dosya secimi ve yukleme
- Ilerleme cubugu
- Mevcut versiyon gosterimi
- Yukleme sonrasi otomatik yeniden baslatma

### 3.10 OTA Firmware Guncelleme

#### FR-3.10.1 OTA Yukleme
- Web arayuzu uzerinden firmware yuklenebilir
- Chunk bazli aktarim (ilerleme takibi)
- Basarili yukleme sonrasi otomatik reboot
- Hata durumunda mevcut firmware korunur

---

## 4. Performans Gereksinimleri

| Gereksinim | Hedef |
|------------|-------|
| Ornekleme dogrulugu | +-1 sample jitter @ hardware timer |
| Maksimum kayit hizi | 1600 Hz (High Perf mod) |
| Canli onizleme gecikmesi | < 500 ms |
| Web arayuzu yanit suresi | < 200 ms (API cagrilari) |
| Depolama kapasitesi | ~1.5 MB (LittleFS partisyonu) |
| Ornek basi boyut | 6 byte (3 eksen x 16-bit) |
| FFT cozunurlugu | 1024 nokta |

---

## 5. Guvenilirlik Gereksinimleri

| Gereksinim | Uygulama |
|------------|----------|
| Deadlock korunma | I2C mutex 5s timeout |
| Dosya butunlugu | Header samples guncelleme kayit sonunda |
| Path traversal korunma | `isSafeAccelFile()` dogrulama |
| Watchdog uyumluluk | Her 256 ornekte `delay(0)` yield |
| Bellek yonetimi | FFT dizileri heap'te (malloc/free) |
| JSON enjeksiyonu | `jsonEscape()` ile dosya adi sanitize |

---

## 6. Kisitlamalar ve Sinirlamalar

- LIS2DW12 MEMS sensoru endüstriyel piezoelektrik ivmeolcerlere kiyasla sinirli hassasiyet sunar
- Cok dusuk frekanslarda (< 2 Hz) sensor gurultusu baskin olabilir
- WiFi ve HTTP sunucu calisirken kayit periyodik kisa gecikmeler yasayabilir
- LittleFS depolama alani sinirli (~1.5 MB); uzun sureli yuksek hizli kayitlar icin yeterli olmayabilir
- Tek kullanici erisimi (WebServer ayni anda bir istemci destekler)
- Kalibrasyon, ortam sicakligina duyarlidir (sicaklik kompanzasyonu yok)

---

## 7. Gelecek Gelistirmeler (Backlog)

- [ ] SD kart destegi (uzun sureli kayit)
- [ ] MQTT/InfluxDB entegrasyonu (uzaktan izleme)
- [ ] Coklu sensor destegi (I2C bus uzerinde birden fazla LIS2DW12)
- [ ] Zaman senkronizasyonu (NTP)
- [ ] Alarm esikleri ve bildirim (ISO 20816 zone A/B/C/D)
- [ ] Arkaplan kayit task'i sirasinda canli onizleme (async)
- [ ] Batarya ile calisma ve derin uyku modu
- [ ] Bluetooth Low Energy (BLE) baglanti alternatifi
- [ ] Waterfall/spectrogram gorsellestirme (web UI)
- [ ] Sicaklik sensoru entegrasyonu (kompanzasyon)
