#pragma once
// V15 relay state model: virtual diagnostics only, no GPIO output.
// Safety-bounded virtual relay diagnostics ONLY. No GPIO writes, no PCA PWM.
// This mode never energizes SRD-12VDC-SL-C coils; there is no feedback sensor.
static_assert(!RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,
              "Dry-run build cannot enable external control");
constexpr uint32_t RELAY_SIM_MAX_MS=10000U;
uint8_t relaySimChannel=0;
uint32_t relaySimStart=0,relaySimUntil=0;
String relaySimReason="IDLE";
void relaySimStop(const char* why){
  relaySimChannel=0;relaySimStart=0;relaySimUntil=0;
  relaySimReason=why;
}
void relaySimTick(uint32_t now){
  if(relaySimChannel && (int32_t)(now-relaySimUntil)>=0){
    relaySimStop("TIME_LIMIT");
  }
  if(relaySimChannel && WiFi.softAPgetStationNum()==0){
    relaySimStop("AP_CLIENT_LOST");
  }
}
String relaySimJSON(){
  relaySimTick(millis());
  String s; s.reserve(550);
  s="{\"dry_run_only\":true,\"physical_gpio_enabled\":false";
  s+=",\"pca_pwm_enabled\":false,\"coil_12v_measured\":false";
  s+=",\"contact_feedback_available\":false,\"physical_contacts\":null";
  s+=",\"simulated_channel\":"+String(relaySimChannel);
  s+=",\"remaining_ms\":"+String(relaySimChannel?(uint32_t)(relaySimUntil-millis()):0);
  s+=",\"reason\":"+quoteJson(relaySimReason);
  s+=",\"simulated_channels\":[";
  for(int i=0;i<4;i++){if(i)s+=",";s+=(relaySimChannel==i+1?"true":"false");}
  return s+"]}";
}
