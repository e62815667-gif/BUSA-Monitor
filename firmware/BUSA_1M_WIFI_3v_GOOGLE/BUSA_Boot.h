#pragma once
// Bounded diagnostics on UART0 and native USB; never wait for a USB host.
class BusaDiagnosticLog : public Print {
 public:
  using Print::write;
  size_t write(uint8_t value) override {
    Serial0.write(value);
    if(Serial && Serial.availableForWrite()>0)Serial.write(value);
    return 1;
  }
};
BusaDiagnosticLog busaLog;
// A retained marker lets the next watchdog/panic boot omit the peripheral
// whose initialization crashed. Normal power-on/reset always retries it.
// Crash phase is retained only across watchdog/panic resets; one transient
// peripheral failure must not permanently hide an approved module.
struct BusaBootMarker {
  uint32_t magic;
  uint32_t phase;
  uint32_t bypass;
  uint32_t failuresTft;
  uint32_t failuresI2c;
};
RTC_DATA_ATTR BusaBootMarker busaBootMarker={0,0,0,0,0};
bool busaSkipTft=false,busaSkipI2c=false;
uint32_t busaLastResetReason=0,busaLastCrashPhase=0;
void busaBootBegin(){
  const esp_reset_reason_t reason=esp_reset_reason();
  busaLastResetReason=(uint32_t)reason;
  const bool crash=reason==ESP_RST_PANIC || reason==ESP_RST_INT_WDT ||
                   reason==ESP_RST_TASK_WDT || reason==ESP_RST_WDT;
  const bool prior=crash && busaBootMarker.magic==0x42555342U;
  busaLastCrashPhase=prior?busaBootMarker.phase:0;
  uint32_t tft=prior?busaBootMarker.failuresTft:0;
  uint32_t i2c=prior?busaBootMarker.failuresI2c:0;
  if(prior && busaBootMarker.phase==1 && tft<3)++tft;
  if(prior && busaBootMarker.phase==2 && i2c<3)++i2c;
  // A second consecutive crash in a single peripheral enters diagnostic
  // bypass. This preserves Wi-Fi/USB access without skipping on first failure.
  busaSkipTft=prior && (tft>=2 || (busaBootMarker.bypass&1U));
  busaSkipI2c=prior && (i2c>=2 || (busaBootMarker.bypass&2U));
  busaBootMarker={0x42555342U,0,(busaSkipTft?1U:0U)|(busaSkipI2c?2U:0U),tft,i2c};
  busaLog.printf("BOOT reset=%lu prior_phase=%lu TFT_fail=%lu I2C_fail=%lu TFT_bypass=%d I2C_bypass=%d\n",
    (unsigned long)busaLastResetReason,(unsigned long)busaLastCrashPhase,
    (unsigned long)tft,(unsigned long)i2c,(int)busaSkipTft,(int)busaSkipI2c);
}
void busaBootPhase(uint32_t phase){busaBootMarker.phase=phase;}
String busaBootJSON(){
  return String("{\"build\":\"15.6-MODULE-BOOTFIX\",\"network_before_peripherals\":true,\"reset_reason\":")+
    String(busaLastResetReason)+",\"last_crash_phase\":"+String(busaLastCrashPhase)+
    ",\"tft_failure_count\":"+String(busaBootMarker.failuresTft)+
    ",\"i2c_failure_count\":"+String(busaBootMarker.failuresI2c)+
    ",\"tft_bypassed\":"+(busaSkipTft?"true":"false")+
    ",\"i2c_bypassed\":"+(busaSkipI2c?"true":"false")+"}";
}
