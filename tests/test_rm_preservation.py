"""FIX3 byte-level preservation contract, independent of Git availability."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
wifi_fix='#include "BUSA_WifiScan.h"' in (sketch/'BUSA_Main.h').read_text()
calc='RM-CALC' in (sketch/'BUSA_Main.h').read_text()
manifest=json.loads((root/'tests/fixtures/rm_fix3_preserved.json').read_text())
audited=json.loads((root/'tests/fixtures/ui12_audited_changes.json').read_text()) if any(v in (sketch/'BUSA_Main.h').read_text() for v in ['WIFI1-UI12-','WIFI1-UI13-','WIFI1-UI14-','WIFI1-UI15-','WIFI1-UI16-']) else {'files':{},'functions':{}}
manifest.update(audited['files'])
for name,digest in manifest.items():
 if (wifi_fix and name.endswith('BUSA_KeypadTFT_UI.h')):continue
 if name=='main_before_normalization' or (calc and name.endswith(('BUSA_Landscape_V14.h','BUSA_ApprovedUI_V13.h'))):continue
 assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
s=(sketch/'BUSA_Main.h').read_text().replace('14v9-N16R8-RM-TELEM2-20261105','14v9-N16R8-RM-BLE-FIX3-20261105')
s='\n'.join(line for line in s.split('\n') if 'busaRainMakerTelemetryJSON()' not in line)
if not calc:assert hashlib.sha256(s.encode()).hexdigest()==manifest['main_before_normalization'],'Unexpected Main modification'
else:
 def function(src,sig):
  start=src.index(sig);pos=src.index('{',start)+1;depth=1
  while depth:
   if src[pos]=='{':depth+=1
   elif src[pos]=='}':depth-=1
   pos+=1
  return src[start:pos]
 original=(sketch/'BUSA_Main.h').read_text()
 critical=json.loads((root/'tests/fixtures/calc_telem2_functions.json').read_text());critical.update(audited['functions'])
 for sig,digest in critical.items():
  assert hashlib.sha256(function(original,sig).encode()).hexdigest()==digest,sig
s=(sketch/'BUSA_RainMaker.h').read_text()
start=s[s.index('static void busaRainMakerStart(){'):s.index('static void busaRainMakerProvision(){')]
if calc:start=start.replace('  busaRmAddCalculators(node);\n','').replace('busaRmScreen,RM_PAGE,(int)currentPage,0,7','busaRmScreen,RM_PAGE,(int)currentPage,0,6')
# CALC4 intentionally groups existing settings into one Device; normalize only
# those exact authorized substitutions before comparing the full FIX3 startup.
if 'RM-CALC4-' in original:
 start=start.replace('new Device("Настройки","esp.device.sensor")','new Device("Экран БУСА","esp.device.sensor")')
 start=start.replace('busaRmKeys=busaRmScreen;','busaRmKeys=new Device("Клавиатура БУСА","esp.device.sensor");')
 start=start.replace('busaRmHealth=busaRmScreen;','busaRmHealth=new Device("Состояние БУСА","esp.device.sensor");')
 start=start.replace('    busaRmScreen->addCb(busaRmWrite);','    busaRmScreen->addCb(busaRmWrite);\n    node.addDevice(*busaRmScreen);')
 start=start.replace('    busaRmKeys->addCb(busaRmWrite);','    busaRmKeys->addCb(busaRmWrite);\n    node.addDevice(*busaRmKeys);')
assert start.replace('  busaRmTelemetryInit();\n','')==(root/'tests/fixtures/rm_fix3_start.cpp').read_text(),'Registered device/parameter schema or startup changed'
print('PASS preservation: original hardware/partition/BLE/OTA/RainMaker contracts; UI12 audited sensor/Google/remote-save/pulse fixes explicitly pinned and tested' if audited['files'] else ('PASS preserved hardware/Google/BLE/OTA/partitions/NVS; critical Main functions; unchanged RainMaker schema/startup; authorized calculator UI additions' if calc else 'PASS FIX3 preservation'))
