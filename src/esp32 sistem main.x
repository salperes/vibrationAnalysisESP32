#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000); // Seri portun oturması için bekleme

  Serial.println("\n--- ESP32 PlatformIO Sistem Raporu ---");

  // --- 1. Çip Bilgileri ---
  Serial.printf("Çip Modeli: %s (Rev: %d)\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("Çip Çekirdek Sayısı: %d\n", ESP.getChipCores());

  // --- 2. Flash Bellek (NVRAM/Depolama) ---
  uint32_t flash_size = ESP.getFlashChipSize();
  uint32_t flash_speed = ESP.getFlashChipSpeed();
  
  Serial.println("\n[FLASH BELLEK]");
  Serial.printf("Fiziksel Boyut: %d MB\n", flash_size / (1024 * 1024));
  Serial.printf("Hız: %d MHz\n", flash_speed / 1000000);
  
  // Flash Modu (QIO, DIO, vb.) - Performans için önemlidir
  FlashMode_t ideMode = ESP.getFlashChipMode();
  Serial.printf("Flash Modu: %s\n", (ideMode == FM_QIO ? "QIO" : (ideMode == FM_QOUT ? "QOUT" : (ideMode == FM_DIO ? "DIO" : (ideMode == FM_DOUT ? "DOUT" : "UNKNOWN")))));

  // --- 3. RAM (SRAM) ---
  Serial.println("\n[RAM DURUMU]");
  Serial.printf("Toplam Heap: %d KB\n", ESP.getHeapSize() / 1024);
  Serial.printf("Boş (Kullanılabilir) Heap: %d KB\n", ESP.getFreeHeap() / 1024);
  Serial.printf("En Büyük Boş Blok: %d KB\n", ESP.getMaxAllocHeap() / 1024);

  // --- 4. PSRAM (Varsa Harici RAM) ---
  if(psramFound()){
    Serial.printf("\n[PSRAM - Harici RAM]\n");
    Serial.printf("Toplam PSRAM: %d MB\n", ESP.getPsramSize() / (1024 * 1024));
    Serial.printf("Boş PSRAM: %d KB\n", ESP.getFreePsram() / 1024);
  } else {
    Serial.println("\n[PSRAM]: Yok");
  }

  Serial.println("--------------------------------------");
}

void loop() {
  // Tekrar etmesine gerek yok
  delay(10000);
}