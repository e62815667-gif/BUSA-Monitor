"""Checks the exact requested V14.27 patch is present without enabling outputs."""
from pathlib import Path
root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
main=(root/"BUSA_Main.h").read_text(encoding="utf-8")
main+="".join((root/n).read_text(encoding="utf-8") for n in ("BUSA_V15_Board.h","BUSA_V15_RelayModel.h","BUSA_V15_Status.h","BUSA_V15_Runtime.h"))
temps=(root/"BUSA_Temperature.h").read_text(encoding="utf-8")
wifi=(root/"BUSA_WifiRecovery.h").read_text(encoding="utf-8")
google=(root/"BUSA_Google.h").read_text(encoding="utf-8")
assert "15v0-N8R2-MODULAR-MONITOR" in main
assert 'String statusJSON(){' in main
assert 's.reserve(6000);' in main
assert 's.reserve(800);' in temps
assert 's.reserve(1500);' in temps
assert 'xTaskCreatePinnedToCore(googleWorker,"busa-google",20480' in google
assert 'StaticJsonDocument<2048> doc;' in google
assert 'snprintf(pwd,sizeof(pwd),"BUSA%016llX",(unsigned long long)mac)' in main
assert 'BUSA_SCAN_SETTLE_MS=500U,BUSA_SCAN_HOME_MS=100U' in wifi
assert 'if(wifiHadScan && !elapsed(now,wifiLastScanRequest,3000U))' in wifi
assert 'for(let i=0;i<25&&j.scanning;i++)' in main
for text in ("j.error==='scan_cooldown'","if(j.scanning)","else if(j.error)","else if(count)","Сетей 2,4 ГГц не найдено."):
 assert text in main,text
assert 'server.collectHeaders(headers,1);\n  server.begin();' in main
assert 'RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false' in main
assert 'kPhysicalOutputsApproved=false' in (root/'BUSA_PCA9685_Leds.h').read_text()
assert 'digitalWrite(' not in main and 'ledcWrite(' not in main
assert 'setInsecure(' not in main+google
print('PASS V15 migrated V14.27 fixes: reserve/stack/TLS/AP/Wi-Fi/front-end and unchanged physical output lock')
