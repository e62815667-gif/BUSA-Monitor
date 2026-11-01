#pragma once
// BUSA V14.7 N16R8 staged boot: software watchdog + RTC diagnostic marker.
// The first stage starts TFT splash; if a peripheral repeatedly crashes,
// the next boots skip only that peripheral so local AP and /api/status survive.
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_err.h>
struct BusaStartupMarker {
  uint32_t magic;
  uint32_t phase;  // 0=idle,1=TFT,3=I2C
  uint32_t tftFailures;
  uint32_t i2cFailures;
};
RTC_DATA_ATTR BusaStartupMarker busaStartupMarker={0,0,0,0};
constexpr uint32_t BUSA_BOOT_MAGIC=0x42555341U;
bool busaTftBypass=false,busaI2cBypass=false;
uint32_t busaLastResetReason=0,busaLastFailurePhase=0;
uint32_t busaTftInitMs=0,busaI2cInitMs=0;
bool busaWdtRegistered=false;
void busaBootBegin(){
  esp_reset_reason_t reason=esp_reset_reason();
  busaLastResetReason=uint32_t(reason);
  const bool crash=reason==ESP_RST_PANIC || reason==ESP_RST_TASK_WDT ||
    reason==ESP_RST_INT_WDT || reason==ESP_RST_WDT;
  if(busaStartupMarker.magic!=BUSA_BOOT_MAGIC){
    busaStartupMarker={BUSA_BOOT_MAGIC,0,0,0};
  }else if(crash){
    busaLastFailurePhase=busaStartupMarker.phase;
    if(busaStartupMarker.phase==1)++busaStartupMarker.tftFailures;
    if(busaStartupMarker.phase==3)++busaStartupMarker.i2cFailures;
  }else{
    // Explicit power/USB reset gives both interfaces a new chance.
    busaStartupMarker.tftFailures=0;
    busaStartupMarker.i2cFailures=0;
  }
  busaTftBypass=busaStartupMarker.tftFailures>=2;
  busaI2cBypass=busaStartupMarker.i2cFailures>=2;
  busaStartupMarker.phase=0;
  Serial0.printf("BOOT reset=%lu prior_phase=%lu tft_fail=%lu i2c_fail=%lu bypass_tft=%d bypass_i2c=%d\n",
    (unsigned long)busaLastResetReason,(unsigned long)busaLastFailurePhase,
    (unsigned long)busaStartupMarker.tftFailures,(unsigned long)busaStartupMarker.i2cFailures,
    (int)busaTftBypass,(int)busaI2cBypass);
  // ESP-IDF 4.4 / Arduino-ESP32 2.0.17 watchdog. Return codes are diagnostic:
  // it may already have been initialized by the ESP32 runtime.
  const esp_err_t init=esp_task_wdt_init(12,true);
  const esp_err_t add=esp_task_wdt_add(NULL);
  busaWdtRegistered=add==ESP_OK || add==ESP_ERR_INVALID_STATE;
  Serial0.printf("BOOT watchdog init=%d add=%d\n",(int)init,(int)add);
}
void busaBootPhase(uint32_t phase){busaStartupMarker.phase=phase;}
void busaBootFeed(){if(busaWdtRegistered)esp_task_wdt_reset();}
String busaBootStatus(){
  String s="{\"startup\":\"TFT_SPLASH_THEN_AP_THEN_MODULES\",\"reset_reason\":";
  s+=String(busaLastResetReason);
  s+=",\"last_failure_phase\":"+String(busaLastFailurePhase);
  s+=",\"current_phase\":"+String(busaStartupMarker.phase);
  s+=",\"tft_failure_count\":"+String(busaStartupMarker.tftFailures);
  s+=",\"i2c_failure_count\":"+String(busaStartupMarker.i2cFailures);
  s+=",\"tft_bypassed\":"+(busaTftBypass?String("true"):String("false"));
  s+=",\"i2c_bypassed\":"+(busaI2cBypass?String("true"):String("false"));
  s+=",\"tft_init_ms\":"+String(busaTftInitMs);
  s+=",\"i2c_init_ms\":"+String(busaI2cInitMs)+"}";
  return s;
}
