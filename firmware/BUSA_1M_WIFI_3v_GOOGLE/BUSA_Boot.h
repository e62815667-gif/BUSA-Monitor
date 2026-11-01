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
struct BusaBootMarker { uint32_t magic; uint32_t phase; uint32_t bypass; };
RTC_DATA_ATTR BusaBootMarker busaBootMarker={0,0,0};
bool busaSkipTft=false,busaSkipI2c=false;
void busaBootBegin(){
  const esp_reset_reason_t reason=esp_reset_reason();
  const bool crash=reason==ESP_RST_PANIC || reason==ESP_RST_INT_WDT ||
                   reason==ESP_RST_TASK_WDT || reason==ESP_RST_WDT;
  if(crash && busaBootMarker.magic==0x42555341U){
    busaSkipTft=(busaBootMarker.bypass&1U) || busaBootMarker.phase==1;
    busaSkipI2c=(busaBootMarker.bypass&2U) || busaBootMarker.phase==2;
  }
  busaBootMarker={0x42555341U,0,(busaSkipTft?1U:0U)|(busaSkipI2c?2U:0U)};
  busaLog.printf("BOOT reset=%u TFT_bypass=%d I2C_bypass=%d\n",
    (unsigned)reason,(int)busaSkipTft,(int)busaSkipI2c);
}
void busaBootPhase(uint32_t phase){busaBootMarker.phase=phase;}
String busaBootJSON(){
  return String("{\"build\":\"15.3-RECOVERY2\",\"network_before_peripherals\":true,\"tft_bypassed\":")+
    (busaSkipTft?"true":"false")+",\"i2c_bypassed\":"+(busaSkipI2c?"true":"false")+"}";
}
