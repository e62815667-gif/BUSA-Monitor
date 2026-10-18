// Standalone low-voltage USB diagnostic. NO RELAYS, NO SENSORS, NO OUTPUTS.
#include <Arduino.h>
#include <esp_system.h>
#if defined(BOARD_HAS_PSRAM)
#error "Build this diagnostic with PSRAM disabled"
#endif
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("BUSA-1M USB DIAG / NO PSRAM / NO OUTPUTS");
}
void loop() {
  Serial.printf("Chip=%s Rev=%u Cores=%u Flash=%lu Heap=%lu Reset=%u Uptime=%lu PSRAM=OFF Outputs=OFF\n",
    ESP.getChipModel(), (unsigned)ESP.getChipRevision(),
    (unsigned)ESP.getChipCores(), (unsigned long)ESP.getFlashChipSize(),
    (unsigned long)ESP.getFreeHeap(), (unsigned)esp_reset_reason(),
    (unsigned long)millis());
  delay(2000);
}
