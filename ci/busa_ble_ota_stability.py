"""Post-process the approved BUSA BLE boot build.
Only OTA boot confirmation, BLE session recovery, and diagnostics are changed.
"""
from pathlib import Path

root = Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
main = root / "BUSA_Main.h"
header = root / "BUSA_RainMaker.h"
s = main.read_text(encoding="utf-8")
h = header.read_text(encoding="utf-8")

def once(old, new):
    global s
    assert s.count(old) == 1, f"Missing or ambiguous source anchor: {old[:100]!r}"
    s = s.replace(old, new, 1)

once("#include <Update.h>", "#include <Update.h>\n#include <esp_ota_ops.h>")

once(
    "static uint32_t busaRmCloudBootAt=0;",
    """static uint32_t busaRmCloudBootAt=0;
static bool busaRmBleBootError=false;
static esp_err_t busaOtaConfirmResult=ESP_ERR_NOT_FOUND;
static esp_err_t busaOtaSelectResult=ESP_OK;
static String busaOtaRunning="unknown",busaOtaBoot="unknown";
static void busaConfirmRunningImage(){
  const esp_partition_t* running=esp_ota_get_running_partition();
  const esp_partition_t* boot=esp_ota_get_boot_partition();
  if(!running){busaOtaConfirmResult=ESP_ERR_NOT_FOUND;return;}
  busaOtaRunning=String(running->label);
  busaOtaBoot=boot?String(boot->label):String("unknown");
  esp_ota_img_states_t state=ESP_OTA_IMG_UNDEFINED;
  const esp_err_t rc=esp_ota_get_state_partition(running,&state);
  if(rc==ESP_OK && state==ESP_OTA_IMG_PENDING_VERIFY){
    busaOtaConfirmResult=esp_ota_mark_app_valid_cancel_rollback();
  }else{
    busaOtaConfirmResult=rc;
  }
  // Select the image that has already brought up the normal local HTTP
  // interface. Never change partitions before successful local boot.
  if(boot && running->address!=boot->address &&
     (busaOtaConfirmResult==ESP_OK || rc==ESP_ERR_NOT_FOUND)){
    busaOtaSelectResult=esp_ota_set_boot_partition(running);
    if(busaOtaSelectResult==ESP_OK)busaOtaBoot=String(running->label);
  }
  Serial0.printf("BUSA OTA running=%s boot=%s validate=0x%X select=0x%X\n",
    busaOtaRunning.c_str(),busaOtaBoot.c_str(),
    (unsigned)busaOtaConfirmResult,(unsigned)busaOtaSelectResult);
}
"""
)

once(
    '  if(prefs.getBool("rmPairNext",false)){',
    '  if(prefs.getBool("rmPairNext",false)){'
)
once(
    '    busaRmBleBootMode=true;\n    busaRainMakerStart();',
    '    busaRmBleBootMode=true;\n    busaRmBleSessionAt=millis();\n    busaRainMakerStart();'
)
once(
    '    else prefs.putString("rmStage","BLE_RMAKER_ERROR");',
    '    else {prefs.putString("rmStage","BLE_RMAKER_ERROR");busaRmBleBootError=true;}'
)
once(
    '''  }else if(prefs.getBool("rmCloudOn",false)){
    prefs.putBool("rmCloudBusy",true);
    busaRainMakerStart();
    busaRmAutoCloud=busaRmStarted;
    busaRmCloudBootAt=millis();
  }''',
    '''  }else if(prefs.getBool("rmCloudOn",false)){
    // Only start cloud services after local Wi-Fi and HTTP are ready.
    busaRmAutoCloud=true;
    busaRmCloudBootAt=millis();
  }'''
)
once(
    '    prefs.putBool("rmPairNext",true);\n    busaRmStartRequested=true;',
    '    prefs.putBool("rmPairNext",true);\n    prefs.putString("rmStage","BLE_REQUESTED");\n    busaRmStartRequested=true;'
)
once(
    '  server.begin();\n  busaReportStage("HTTP_READY");',
    '  server.begin();\n  busaConfirmRunningImage();\n  busaReportStage("HTTP_READY");'
)
once(
    '  startedAt=millis();\n  tftProfile=1; // ST7789 only',
    '''  snprintf(busaRmService,sizeof(busaRmService),"BUSA_%04X",(unsigned)(ESP.getEfuseMac()&0xFFFF));
  startedAt=millis();
  tftProfile=1; // ST7789 only'''
)
once(
    '''  if(busaRmBleBootMode){
    static uint32_t endingAt=0;''',
    '''  if(busaRmBleBootMode){
    static uint32_t endingAt=0;
    static bool advertisementRecorded=false;
    if(busaRmBleAdvertising && !advertisementRecorded){
      advertisementRecorded=true;
      prefs.putString("rmStage","BLE_ADVERTISING");
    }
    if(busaRmBleBootError && !endingAt)endingAt=millis();
    if(!busaRmBleAdvertising && !busaRmBleBootError && !endingAt &&
       millis()-busaRmBleSessionAt>30000){
      prefs.putString("rmStage","BLE_NOT_ADVERTISING");
      if(busaRmBleSessionStarted)wifi_prov_mgr_stop_provisioning();
      endingAt=millis();
    }'''
)
once(
    '      prefs.putString("rmStage",busaRmBleCredentialsAccepted?"PAIRED":"PAIR_TIMEOUT");',
    '''      if(busaRmBleCredentialsAccepted)prefs.putString("rmStage","PAIRED");
      else if(prefs.getString("rmStage","")=="BLE_ADVERTISING")
        prefs.putString("rmStage","BLE_TIMEOUT");'''
)
once(
    '''  if(busaRmAutoCloud && prefs.getBool("rmCloudBusy",false) &&
     elapsed(millis(),busaRmCloudBootAt,30000)){
    prefs.putBool("rmCloudBusy",false);
  }''',
    '''  static bool busaCloudAttempted=false;
  if(busaRmAutoCloud && !busaCloudAttempted &&
     elapsed(millis(),busaBootAt,20000) && WiFi.status()==WL_CONNECTED &&
     !otaActive && !busaWifiScanRunning){
    busaCloudAttempted=true;
    prefs.putBool("rmCloudBusy",true);
    busaRainMakerStart();
    busaRmCloudBootAt=millis();
    if(!busaRmStarted){
      prefs.putBool("rmCloudBusy",false);
      prefs.putBool("rmCloudOn",false);
      prefs.putString("rmStage","CLOUD_INIT_ERROR");
    }
  }
  if(busaCloudAttempted && busaRmStarted && prefs.getBool("rmCloudBusy",false) &&
     elapsed(millis(),busaRmCloudBootAt,30000)){
    prefs.putBool("rmCloudBusy",false);
  }'''
)
once(
    '  s+=",\\"ota\\":"+otaJSON();\n  s+=",\\"bluetooth_enabled\\":false";',
    '''  s+=",\\"ota\\":"+otaJSON();
  s+=",\\"ota_boot\\":{\\"running\\":"+quoteJson(busaOtaRunning)+
    ",\\"selected\\":"+quoteJson(busaOtaBoot)+
    ",\\"verification_rc\\":"+String((int)busaOtaConfirmResult)+
    ",\\"selection_rc\\":"+String((int)busaOtaSelectResult)+"}";
  // Normal operation intentionally has BLE switched off. Last BLE outcome
  // persists independently of this live field.
  s+=",\\"bluetooth_enabled\\":false";
  s+=",\\"ble_last_stage\\":"+quoteJson(prefs.getString("rmStage","IDLE"));'''
)
once(
    '''       ",\\"feedback\\":"+quoteJson(busaRmBleFeedback)+
       ",\\"automatic_start\\":false''',
    '''       ",\\"feedback\\":"+quoteJson(busaRmBleFeedback)+
       ",\\"last_stage\\":"+quoteJson(prefs.getString("rmStage","IDLE"))+
       ",\\"automatic_start\\":false'''
)
assert "BUSA_RM_FIXED_POP=\"123456789\"" in h
assert "ARDUINO_EVENT_PROV_INIT" in h
assert "WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE" in h
assert "WiFi.softAP(\"BUSA-1M-001-R04\"" in s
assert "esp_ota_mark_app_valid_cancel_rollback()" in s
assert "esp_ota_set_boot_partition(running)" in s
s=s.replace("14v9-N16R8-RM-BLE-BOOT-20261105",
            "14v9-N16R8-RM-BLE-FIX2-20261105")
assert "14v9-N16R8-RM-BLE-FIX2-20261105" in s
main.write_text(s,encoding="utf-8")
print("BUSA OTA and BLE diagnostics and recovery patch applied.")
