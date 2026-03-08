# Versiyonlama ve Changelog Kurallari

Bu dosya, accMeter2 projesinde uygulanacak versiyonlama ve changelog standartlarini tanimlar.

Version bilgisi `src/config.h` dosyasindaki `APP_VERSION` makrosunda tutulur ve web arayuzunde gosterilir.
Her kod degisikliginde commit yapilacak, push ancak kullanici talebi ile olacak.

---

## Versiyon Formati

**MAJOR.MINOR.RevID**

| Parca   | Aciklama                                    | Ornek |
|---------|---------------------------------------------|-------|
| MAJOR   | Ana surum (buyuk mimari degisiklikler)      | 3     |
| MINOR   | Alt surum (ozellik gruplari)                | 9     |
| RevID   | Changelog'daki Rev. ID (her commit'te artar) | 1     |

Ornek: `3.9.1`

RevID changelog'daki son Rev. ID ile birebir eslesir.

---

## Versiyon Guncellenecek Dosyalar

Her versiyon artisinda su dosyalar guncellenir:

| Dosya              | Alan                          |
|--------------------|-------------------------------|
| `src/config.h`     | `APP_VERSION` makrosu         |
| `CLAUDE.md`        | Current Version satiri        |

---

## Changelog Formati

Dosya: `changelog.md` (proje root'unda)

Her degisiklik kaydi su formatta eklenir:

```
---------------------------------------------------------
Rev. ID    : {COUNTER}
Rev. Date  : DD.MM.YYYY
Rev. Time  : HH:MM:SS
Rev. Prompt: {Kullanicinin verdigi prompt/istek ozeti}

Rev. Report: (
{Yapilan degisikliklerin madde madde ozet raporu}
- Firmware: ...
- Web UI: ...
- API: ...
- Sensor: ...
)
---------------------------------------------------------
```

### Kurallar

1. **Newest first (prepend):** Yeni kayit dosyanin **en ustune** eklenir, en yeni kayit her zaman en ustte
2. **Lokal saat:** Rev. Time lokal zaman dilimini kullanir
3. **Rev. ID:** Onceki kaydin ID'si + 1 (sirali, bosluksuz)
4. **Dosya yoksa:** Yeni `changelog.md` olustur
5. **Mevcut kayitlar degistirilmez**

### Arsivleme

- `changelog.md` en fazla **11 kayit** tutar
- 11'i asinca en eski 10 kayit arsivlenir: `archives/changelog_{START}-{END}.md`
  - Ornek: `archives/changelog_001-010.md`, `archives/changelog_011-020.md`
- Arsiv dosyalarinda da **newest first** siralama
- Kalan kayitlar (en yeniler) `changelog.md`'de kalir

---

## Kod Degisikligi Sonrasi Standart Akis (ZORUNLU)

Her kod degisikliginden sonra bu sira **eksiksiz** takip edilir:

```
1. BUILD          -> `pio run` ile derleme kontrolu
2. changelog.md   -> Changelog kaydi ekle -- en uste (Rev. ID, Date, Time, Prompt, Report)
3. Version bump   -> src/config.h APP_VERSION guncelle
4. CLAUDE.md      -> Versiyon numarasini guncelle
5. Commit         -> Degisiklikleri commit et (push sadece kullanici talebiyle)
```

---

## Degisiklik Kategorileri

Bu projede changelog raporlarinda su kategoriler kullanilir:

| Kategori   | Kapsam                                              |
|------------|------------------------------------------------------|
| Firmware   | main.cpp, app_state, FreeRTOS task, I2C, timer       |
| API        | api_handlers.cpp endpoint degisiklikleri              |
| Web UI     | html_pages.cpp icindeki HTML/JS degisiklikleri        |
| Sensor     | LIS2DW12 kutuphane veya kalibrasyon degisiklikleri    |
| Build      | platformio.ini, build flag, dependency degisiklikleri |

---

## Ornek Akis

Kullanici: "FFT analizine Hanning penceresi ekle"

```
1. Kod degisikliklerini yap (api_handlers.cpp)
2. `pio run` ile derleme kontrolu
3. changelog.md'nin en ustune ekle:
   ---------------------------------------------------------
   Rev. ID    : 2
   Rev. Date  : 08.03.2026
   Rev. Time  : 14:30:00
   Rev. Prompt: FFT analizine Hanning penceresi ekle

   Rev. Report: (
   - API: handleApiFft() icinde Hanning window uygulamasi eklendi
   - Web UI: FFT grafiginde pencere tipi gosterimi eklendi
   )
   ---------------------------------------------------------

4. src/config.h: #define APP_VERSION "V3.9.2"
5. CLAUDE.md: Current Version: `3.9.2`
6. Commit (push sadece kullanici talebiyle)
```
