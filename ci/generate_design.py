"""Embed approved local design assets. --check never writes files."""
from pathlib import Path
import base64,re,sys
root=Path(__file__).resolve().parents[1]
p=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h'
original=p.read_text();s=original
asset=root/'web/assets/holographic-apparatus.webp'
if asset.exists():
    url='data:image/webp;base64,'+base64.b64encode(asset.read_bytes()).decode()
    s,count=re.subn(r'(<img class="approvedRigArt" src=")[^"]+("[^>]*>)',lambda m:m.group(1)+url+m.group(2),s,count=1)
    assert count==1,'Expected one live apparatus image'
for kind,tag,name in [('CSS','style','design.css'),('JS','script','design.js')]:
    start='/* BUSA_DESIGN_'+kind+'_BEGIN */';end='/* BUSA_DESIGN_'+kind+'_END */'
    content=(root/'web'/name).read_text()
    if kind=='CSS':content+='\n'+(root/'web/holographic.css').read_text()
    block=start+'\n'+content+'\n'+end
    if start in s:s=re.sub(re.escape(start)+'.*?'+re.escape(end),lambda _:block,s,flags=re.S)
    elif kind=='CSS':s=s.replace('</style>',block+'\n</style>',1)
    else:s=s.replace('</body>','<script>\n'+block+'\n</script></body>',1)
if '--check' in sys.argv:assert original==s,'Local design assets are stale'
else:p.write_text(s)
print('PASS approved local design assets'+(' (read-only)' if '--check' in sys.argv else ' embedded'))
