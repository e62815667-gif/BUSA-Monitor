from pathlib import Path
from PIL import Image
import re
root=Path(__file__).resolve().parents[1]
im=Image.open(root/'web/assets/holographic-apparatus.webp').convert('RGBA')
im.thumbnail((110,124),Image.Resampling.LANCZOS)
p=root/'display_delivery/microSD/busa/rig.rgb565a'
b=p.read_bytes()
assert len(b)==im.width*im.height*3
assert b[2::3]==im.getchannel('A').tobytes()
h=(root/'firmware/BUSA_Display_N4R8/BUSA_DisplayAsset.h').read_text()
assert f'BUSA_ASSET_W={im.width}, BUSA_ASSET_H={im.height}' in h
assert bytes(map(int,re.findall(r'\d+',h.split('BUSA_ASSET[] = {')[1])))==b
old=bytearray()
for r,g,bl,a in im.getdata():
 c=(((r*31+127)//255)<<11)|(((g*63+127)//255)<<5)|((bl*31+127)//255)
 old.extend((c&255,c>>8,a))
assert b!=old
print('PASS approved asset dimensions, identical alpha silhouette, embedded/SD equality and revised RGB565 quantization')
