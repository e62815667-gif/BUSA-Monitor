"""Regression safety checks for V14.28 memory and cloud release fixes.

Static invariants complement native build and existing behavioral tests.
Full heap/stack stability requires physical ESP32 soak testing.
"""
from pathlib import Path
import re

root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
main=(root/"BUSA_Main.h").read_text(encoding="utf-8")
google=(root/"BUSA_Google.h").read_text(encoding="utf-8")
temp=(root/"BUSA_Temperature.h").read_text(encoding="utf-8")
wifi=(root/"BUSA_WifiRecovery.h").read_text(encoding="utf-8")
assert "14v28-N8R2-HEAP-GOOGLE-AUDIT" in main
assert "s.reserve(8192)" in main
assert "s.reserve(2300)" in temp
assert "s.reserve(1800)" in temp
assert "setInterval(update,3000)" in main
assert "largest_free_8bit_block" in main
assert "heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)" in main
assert "tempSyncLogical(now);" in temp and "elapsed(now,lastSync,500U)" in temp
assert 'StaticJsonDocument<2048> doc;' in google
assert '"busa-google",20480' in google
assert "packet.reserve(snapshot.length()+256)" in google
assert "cloudReleaseSnapshot()" in google
assert "portENTER_CRITICAL(&cloudReleaseMux)" in google
assert "portEXIT_CRITICAL(&cloudReleaseMux)" in google
assert "cloudReleaseData=next;" in google
assert "cloudReleaseAppSize()" in main and "cloudReleaseAppSize()" in google
assert "cloudReleasePresent" not in google
assert 'const char *headers[]={"X-Busa-Token"};' in main
assert main.index("server.collectHeaders(headers,1);") < main.index("server.begin();")
assert "WiFi.mode(WIFI_AP_STA)" in main and "wifiRecoveryTick(now)" in main
assert 'RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false' in main
assert "digitalWrite(" not in main and "ledcWrite(" not in main
# Static size controls remain in place; no remote OTA.
assert "if(request.getSize()>2048)return false" in google
assert 'remote_install_enabled' in main
assert "wifiCloudPaused" in wifi
print("PASS V14.28: memory preallocation, 3s poll, cloud release atomic snapshot, 20 KB TLS stack, 2 KB JSON pool, prebegin OTA headers, output lock")
