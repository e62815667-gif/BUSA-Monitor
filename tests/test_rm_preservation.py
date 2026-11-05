"""FIX3 byte-level preservation contract, independent of Git availability."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
manifest=json.loads((root/'tests/fixtures/rm_fix3_preserved.json').read_text())
for name,digest in manifest.items():
 if name=='main_before_normalization':continue
 assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
s=(sketch/'BUSA_Main.h').read_text().replace('14v9-N16R8-RM-TELEM2-20261105','14v9-N16R8-RM-BLE-FIX3-20261105')
s='\n'.join(line for line in s.split('\n') if 'busaRainMakerTelemetryJSON()' not in line)
assert hashlib.sha256(s.encode()).hexdigest()==manifest['main_before_normalization'],'Unexpected Main modification'
s=(sketch/'BUSA_RainMaker.h').read_text()
start=s[s.index('static void busaRainMakerStart(){'):s.index('static void busaRainMakerProvision(){')]
assert start.replace('  busaRmTelemetryInit();\n','')==(root/'tests/fixtures/rm_fix3_start.cpp').read_text(),'Registered device/parameter schema or startup changed'
print('PASS FIX3 hardware/Google/BLE/OTA/partitions/NVS sources byte-identical; Main only version/diagnostic; all cloud parameter names/types and node startup unchanged')
