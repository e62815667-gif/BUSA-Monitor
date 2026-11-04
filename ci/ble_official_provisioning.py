"""Apply Espressif Arduino-ESP32 2.0.17 BLE provisioning event flow after BLE boot patch."""
from pathlib import Path

p = Path("firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_RainMaker.h")
source = p.read_text(encoding="utf-8")
first = source.index("static void busaRmProvisionEvent(arduino_event_t *event){")
last = source.index("static bool busaRmFindSensor(", first)

events = """static void busaRmProvisionEvent(arduino_event_t *event){
  switch(event->event_id){
    case ARDUINO_EVENT_PROV_START:
      busaRmBleAdvertising=true;
      break;
    case ARDUINO_EVENT_PROV_INIT:
      wifi_prov_mgr_disable_auto_stop(10000);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      busaRmBleCredentialsAccepted=true;
      wifi_prov_mgr_stop_provisioning();
      break;
    case ARDUINO_EVENT_PROV_CRED_FAIL:
      busaRmBleCredentialsRejected=true;
      break;
    case ARDUINO_EVENT_PROV_END:
      busaRmBleAdvertising=false;
      busaRmBleEnded=true;
      break;
    default:
      break;
  }
}
"""
source = source[:first] + events + source[last:]
assert 'BUSA_RM_FIXED_POP="123456789"' in source
assert 'WiFiProv.beginProvision' in source
assert 'ARDUINO_EVENT_PROV_INIT' in source
p.write_text(source, encoding="utf-8")
print("Official RainMaker BLE provisioning events applied.")
