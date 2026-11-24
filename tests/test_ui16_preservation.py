"""Pin UI16's boundary: presentation and release version only, offline."""
from pathlib import Path
import base64, hashlib, json, re

root=Path(__file__).resolve().parents[1]
source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
baseline=json.loads((root/'tests/fixtures/ui16_preservation.json').read_text())
# COL1 is authorized to change the pulse input, calibration and status only.
# Reverse each exact reviewed change before applying the original byte-level guard.
normalized=source
for old,new in reversed(json.loads((root/'tests/fixtures/collection_gpio1_delta.json').read_text())):
    assert normalized.count(new)==1, 'Missing or ambiguous approved COL1 change'
    normalized=normalized.replace(new,old,1)
backend=re.sub(r'R"HTMLBUSA\(.*?\)HTMLBUSA"','<WEB_PAGE>',normalized,flags=re.S)
for release in ['UI16-CUBE1-20261121-GEOM1-MOTION1-20261122','UI16-CUBE1-20261121-GEOM1']:
    backend=backend.replace(release,'<UI_RELEASE>')
# The only authorized Main change is one NVS load immediately after prefs.begin.
assert backend.count('  busaCalcLoadGeometry();\n')==1
backend=backend.replace('  busaCalcLoadGeometry();\n','',1)
assert backend.count('static bool busaNvsReady=false;\n')==1
backend=backend.replace('Preferences prefs;\nstatic bool busaNvsReady=false;\n', 'Preferences prefs;\n',1)
backend=backend.replace('static bool busaRmPairDeferred=false;\n','static bool busaRmPairDeferred=false;\nstatic bool busaNvsReady=false;\n',1)
assert hashlib.sha256(backend.encode()).hexdigest()==baseline['normalized_backend_sha256'],'Backend changed beyond release version'
html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
primary=re.search(r'<script>(.*?)</script>',html,re.S).group(1)
original=json.loads((root/'tests/fixtures/ui13_local_changes.json').read_text())['authorized']
assert hashlib.sha256(primary.encode()).hexdigest()==original['primary_browser_script'],'Controller browser script changed'
preserved=json.loads((root/'tests/fixtures/ui10_preserved.json').read_text())
for name in ['calculator.css','calculator/core.js']:
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==preserved[name],name
image=re.search(r'<img class="approvedRigArt" src="data:image/webp;base64,([^"]+)"',html).group(1)
assert base64.b64decode(image)==(root/'web/assets/holographic-apparatus.webp').read_bytes()
assert 'cubeDims' in (root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_CalculatorUI.h').read_text()
assert 'dimensions.open' in (root/'calculator/ui.js').read_text()
print('PASS UI16: backend normalized to approved source with exact geometry NVS exception; original calculator math, display assets and browser script unchanged')
