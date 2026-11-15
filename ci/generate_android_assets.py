"""Byte-identical offline fallback from the shipping ESP32 UI, no mock data."""
from pathlib import Path
import hashlib, json, re, sys

root=Path(__file__).resolve().parents[1]
source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0].encode()
version=re.search(r'const char VERSION\[\]\s*=\s*"([^"]+)"',source).group(1)
files={'index.html':html,'calculator.js':(root/'calculator.js').read_bytes(),
       'calculator.css':(root/'calculator.css').read_bytes()}
manifest={'firmware':version,'sha256':{name:hashlib.sha256(data).hexdigest() for name,data in files.items()}}
files['SOURCE.json']=(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode()
dest=root/'android/app/src/main/assets/local'
for name,data in files.items():
    path=dest/name
    if '--check' in sys.argv:assert path.read_bytes()==data,'Stale Android fallback: '+name
    else:
        dest.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
print('PASS Android offline fallback equals the firmware UI and calculator bytes')
