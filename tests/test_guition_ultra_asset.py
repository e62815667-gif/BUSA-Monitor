from pathlib import Path
from PIL import Image
import re, subprocess, sys, tempfile
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

# Execute the real generator on a uniform opaque source. Ordered dithering
# creates multiple RGB565 codes on this fixture and must fail this test.
with tempfile.TemporaryDirectory() as tmp:
    work=Path(tmp)
    (work/'web/assets').mkdir(parents=True)
    (work/'firmware/BUSA_Display_N4R8').mkdir(parents=True)
    Image.new('RGBA',(16,16),(21,63,85,255)).save(work/'web/assets/holographic-apparatus.webp',lossless=True)
    subprocess.run([sys.executable,str(root/'ci/generate_guition_asset.py')],cwd=work,check=True)
    pixels=(work/'display_delivery/microSD/busa/rig.rgb565a').read_bytes()
    expected=(((21*31+127)//255)<<11)|(((63*63+127)//255)<<5)|((85*31+127)//255)
    assert pixels==bytes((expected&255,expected>>8,255))*256
print('PASS uniform RGB565 source remains spatially uniform; no ordered pattern')
