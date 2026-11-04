from pathlib import Path
p=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
r=p/"BUSA_RainMaker.h"
m=p/"BUSA_Main.h"
s=r.read_text()
a=s.index("static void busaRmProvisionEvent(arduino_event_t *event){")
b=s.index("static bool busaRmFindSensor(",a)
s=s[:a]+"""static void busaRmProvisionEvent(arduino_event_t *event){
  switch(event->event_id){
    case ARDUINO_EVENT_PROV_START: busaRmBleAdvertising=true;break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS: busaRmBleCredentialsAccepted=true;break;
    case ARDUINO_EVENT_PROV_CRED_FAIL: busaRmBleCredentialsRejected=true;break;
    case ARDUINO_EVENT_PROV_END:
      busaRmBleAdvertising=false;
      busaRmBleEnded=true;break;
    default:break;
  }
}
"""+s[b:]
s=s.replace("  RMaker.enableOTA(OTA_USING_TOPICS);\n  RMaker.enableTZService();\n","")
assert "RMaker.enableOTA" not in s
r.write_text(s)
s=m.read_text()
old='const char VERSION[]="БУСА 1М WiFi 14v9-N16R8-RM-BLE-ONCE-20261105";'
assert old in s
s=s.replace(old,'const char VERSION[]="БУСА 1М WiFi 14v9-N16R8-RM-BLE-BOOT-20261105";')
# Static boot flag: Wi-Fi code remains unchanged.
anchor='#include "BUSA_RainMaker.h"'
assert s.count(anchor)==1
s=s.replace(anchor,anchor+"""
static bool busaRmBleBootMode=false;
static bool busaRmAutoCloud=false;
static uint32_t busaRmCloudBootAt=0;
""")
# Start pairing in a clean boot BEFORE any regular Wi-Fi setup.
anchor='  prefs.begin("busa-r04",false);'
assert s.count(anchor)==1
s=s.replace(anchor,anchor+"""
  if(prefs.getBool("rmPairNext",false)){
    prefs.putBool("rmPairNext",false);
    prefs.putString("rmStage","BLE_BOOT");
    busaRmBleBootMode=true;
    busaRainMakerStart();
    if(busaRmStarted)busaRainMakerProvision();
    else prefs.putString("rmStage","BLE_RMAKER_ERROR");
    return;
  }
  if(prefs.getBool("rmCloudBusy",false)){
    prefs.putBool("rmCloudBusy",false);
    prefs.putBool("rmCloudOn",false);
    prefs.putString("rmStage","CLOUD_RECOVERED");
  }else if(prefs.getBool("rmCloudOn",false)){
    prefs.putBool("rmCloudBusy",true);
    busaRainMakerStart();
    busaRmAutoCloud=busaRmStarted;
    busaRmCloudBootAt=millis();
  }
""")
# Request pairing from local app; reboot into an isolated BLE session.
anchor='    busaRmStartRequested=true;'
assert s.count(anchor)==1
s=s.replace(anchor,'    prefs.putBool("rmPairNext",true);\n'+anchor)
a=s.index('  // RainMaker BLE is started ONLY after explicit local app authorization.')
b=s.index('  // Display updates take priority over peripheral diagnostics.',a)
s=s[:a]+"""  // Pairing uses a clean boot. The original Wi-Fi procedures are untouched.
  static uint32_t bleRestartAt=0;
  if(busaRmStartRequested){
    busaRmStartRequested=false;
    bleRestartAt=millis();
  }
  if(bleRestartAt && elapsed(millis(),bleRestartAt,1400)){
    soundOutputsOff();
    ESP.restart();
  }
  if(busaRmAutoCloud && prefs.getBool("rmCloudBusy",false) &&
     elapsed(millis(),busaRmCloudBootAt,30000)){
    prefs.putBool("rmCloudBusy",false);
  }
"""+s[b:]
# Early return in pairing mode loop before normal hardware / network service.
anchor='void loop(){\n  uint32_t now=millis();'
assert s.count(anchor)==1
s=s.replace(anchor,"""void loop(){
  if(busaRmBleBootMode){
    static uint32_t endingAt=0;
    if(busaRmBleCredentialsAccepted && !endingAt &&
       millis()-busaRmBleSessionAt>=8000)endingAt=millis();
    if(busaRmBleEnded && !endingAt)endingAt=millis();
    if(millis()-busaRmBleSessionAt>180000 && !endingAt){
      wifi_prov_mgr_stop_provisioning();
      endingAt=millis();
    }
    if(endingAt && millis()-endingAt>4000){
      prefs.putBool("rmCloudOn",busaRmBleCredentialsAccepted);
      prefs.putBool("rmCloudBusy",false);
      prefs.putString("rmStage",busaRmBleCredentialsAccepted?"PAIRED":"PAIR_TIMEOUT");
      ESP.restart();
    }
    delay(20);
    return;
  }
  uint32_t now=millis();""")
assert "WiFi.scanNetworks" in s and s.count("WiFi.scanNetworks")==1
assert 'id="disconnectWifi"' in s
m.write_text(s)
print("BLE cold-start fix applied; WiFi code preserved.")
