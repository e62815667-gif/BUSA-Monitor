"""Integration guards for radio ownership, OTA and the pinned framework fixes."""
from pathlib import Path
import importlib.util,tempfile
m=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
w=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_WifiRecovery.h').read_text()
g=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Google.h').read_text()
assert 'WiFi.begin(' not in m and w.count('WiFi.begin(')==1
assert 'WiFi.reconnect(' not in m+w
assert 'WiFi.setAutoReconnect(true)' not in m+w
assert 'esp_wifi_scan_start(' not in m+w # only Arduino owns asynchronous results
assert 'scanNetworks(true,true,false,120,wifiScanChannel)' in w
assert 'if(local)' not in w and 'BUSA_LOCAL_QUIET_MS' not in w
assert 'BUSA_BOOT_GRACE_MS=3000U' in w and 'BUSA_HTTP_REPLY_GRACE_MS=1500U' in w
assert 'ap_sta_simultaneous' in w
assert 'отключи телефон от сети БУСА минимум' not in m
assert 'esp_wifi_set_channel(' not in w
assert 'wifiScheduleConnect(millis());' in m and 'server.send(202,' in m
assert 'if(otaActive||otaRestart)' in w
assert 'wifiScanBusy()' in m[m.index('void uploadOTA()'):m.index('#include "BUSA_Google.h"')]
assert 'otaActive||otaRestart||wifiScanBusy()' in m
assert 'wifiCloudPaused.load()' in g
assert 'web_health' in m
assert 'RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false' in m
for unsafe in ['digitalWrite(', 'pinMode(KPIN', 'ledcWrite(', 'WiFi.softAPdisconnect(']:assert unsafe not in m+w
# HTTP request body only queues radio work; it cannot migrate the channel before replying.
handler=m[m.index('server.on("/api/wifi",'):m.index('server.on("/api/page",')]
assert 'WiFi.disconnect(' not in handler and 'wifiCancelScan();' not in handler
# Apply the exact build patch twice; reject unexpected upstream versions.
spec=importlib.util.spec_from_file_location('patcher','tools/patch_arduino_2_0_17.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
with tempfile.TemporaryDirectory() as d:
    core=Path(d);(core/'libraries/WiFi/src').mkdir(parents=True);(core/'libraries/WebServer/src').mkdir(parents=True)
    (core/'platform.txt').write_text('version=2.0.17\n')
    (core/'libraries/WiFi/src/WiFiGeneric.cpp').write_text('else if(first_connect) {\n')
    header=core/'libraries/WebServer/src/WebServer.h'
    header.write_text('\n'.join(f'#define {x} {y}' for x,y in [('HTTP_MAX_DATA_WAIT',5000),('HTTP_MAX_POST_WAIT',5000),('HTTP_MAX_SEND_WAIT',5000),('HTTP_MAX_CLOSE_WAIT',2000)]))
    p.patch(core);p.patch(core)
    assert 'first_connect && WiFi.getAutoReconnect()' in (core/'libraries/WiFi/src/WiFiGeneric.cpp').read_text()
    assert '#define HTTP_MAX_POST_WAIT 2000' in header.read_text()
    (core/'platform.txt').write_text('version=3.0.0')
    try:p.patch(core);raise RuntimeError('Unpinned core accepted')
    except AssertionError:pass
print('PASS: integration: one radio owner, deferred HTTP join, OTA/output locks, pinned framework patch')
