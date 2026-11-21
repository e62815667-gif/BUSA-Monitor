"""Pin UI16's boundary: presentation and release version only, offline."""
from pathlib import Path
import base64, hashlib, json, re

root=Path(__file__).resolve().parents[1]
source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
baseline=json.loads((root/'tests/fixtures/ui16_preservation.json').read_text())
backend=re.sub(r'R"HTMLBUSA\(.*?\)HTMLBUSA"','<WEB_PAGE>',source,flags=re.S)
backend=backend.replace('UI16-CUBE1-20261121','<UI_RELEASE>')
assert hashlib.sha256(backend.encode()).hexdigest()==baseline['normalized_backend_sha256'],'Backend changed beyond release version'
html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
primary=re.search(r'<script>(.*?)</script>',html,re.S).group(1)
original=json.loads((root/'tests/fixtures/ui13_local_changes.json').read_text())['authorized']
assert hashlib.sha256(primary.encode()).hexdigest()==original['primary_browser_script'],'Controller browser script changed'
preserved=json.loads((root/'tests/fixtures/ui10_preserved.json').read_text())
for name in ['calculator.js','calculator.css','calculator/ui.js','calculator/core.js']:
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==preserved[name],name
image=re.search(r'<img class="approvedRigArt" src="data:image/webp;base64,([^"]+)"',html).group(1)
assert base64.b64decode(image)==(root/'web/assets/holographic-apparatus.webp').read_bytes()
print('PASS UI16: entire backend except version, primary browser script and calculator fixed to approved CUBE1 checksums; embedded apparatus matches asset')
