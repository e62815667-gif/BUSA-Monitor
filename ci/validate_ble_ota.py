"""Validate the canonical FIX3 sources without rewriting a single source byte."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT/'firmware/BUSA_1M_WIFI_3v_GOOGLE'


def validate():
    main=(SKETCH/'BUSA_Main.h').read_text()
    rain=(SKETCH/'BUSA_RainMaker.h').read_text()
    ota=(SKETCH/'BUSA_OtaBoot.h').read_text()
    assert main.count('14v9-N16R8-RM-BLE-FIX3-20261105')==1
    assert 'static bool busaRmBleBootMode=false;' in main
    assert 'busaRmPairDeferred=' in main and '&&busaOtaPendingVerification()' in main
    assert 'busaConfirmRunningImage(busaNvsReady&&wifiRunning)' in main
    assert 'if(busaRmStartRequested&&!busaOtaPendingVerification())' in main
    assert 'esp_ota_set_boot_partition(' not in main+ota
    assert 'esp_ota_mark_app_valid_cancel_rollback()' in ota
    assert 'ESP_OTA_IMG_PENDING_VERIFY' in ota
    assert 'PROV_BUSA_%04X' in main and 'PROV_BUSA_%04X' in rain
    assert 'case ARDUINO_EVENT_PROV_INIT:' in rain
    assert 'wifi_prov_mgr_disable_auto_stop(10000)' in rain
    assert 'case ARDUINO_EVENT_PROV_CRED_SUCCESS:' in rain
    assert 'busaRmStopProvisioning();' in rain
    assert 'busaRmBleStopRequested.exchange(true)' in rain
    assert re.search(r'WiFiProv.beginProvision\(WIFI_PROV_SCHEME_BLE,\s*WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,\s*WIFI_PROV_SECURITY_1,',rain)
    assert 'RMaker.enableOTA(' not in rain  # Existing local OTA only; no new cloud installer.
    assert 'const bool paired=busaRmBleCredentialsAccepted&&busaRmBleEnded;' in main
    assert 'busaSaveProvisionedStation()' in main
    for feature in ['WiFi.softAP(', 'launchGoogleCloud()', 'sampleFlow(now)',
                    'sampleTemperatures(now)', 'startScreen()', 'pollKeys(now)',
                    'void uploadOTA()', 'server.on("/api/status"', 'busaRainMakerApply()']:
        assert feature in main,feature
    assert main.count('WiFi.scanNetworks')==1
    assert 'KPIN[4]={7,6,5,4}' in main
    assert 'busaRainMakerPoll(now)' in main
    assert (SKETCH/'partitions.csv').read_text().count('0x300000')==2
    print('PASS canonical FIX3: official BLE flow, deferred pending-OTA pairing, no boot reselection, approved features retained')


if __name__=='__main__':
    validate()
