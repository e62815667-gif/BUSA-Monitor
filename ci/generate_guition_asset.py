from pathlib import Path
from PIL import Image
import hashlib
src=Path('web/assets/holographic-apparatus.webp')
dest=Path('firmware/BUSA_Display_N4R8')
sd=Path('display_delivery/microSD/busa')
sd.mkdir(parents=True,exist_ok=True)
im=Image.open(src).convert('RGBA')
im.thumbnail((110,132),Image.Resampling.LANCZOS)
buf=bytearray()
for r,g,b,a in im.getdata():
    c=(((r*31+127)//255)<<11)|(((g*63+127)//255)<<5)|((b*31+127)//255)
    buf.extend((c&255,c>>8,a))
assert len(buf)==im.width*im.height*3
(sd/'rig.rgb565a').write_bytes(buf)
lines=['#pragma once','#include <stdint.h>','#include <stddef.h>',f'constexpr int BUSA_ASSET_W={im.width}, BUSA_ASSET_H={im.height};',f'constexpr size_t BUSA_ASSET_SIZE={len(buf)};','static const uint8_t BUSA_ASSET[] = {']
for i in range(0,len(buf),24):lines.append(','.join(str(v) for v in buf[i:i+24])+',')
lines.append('};')
(dest/'BUSA_DisplayAsset.h').write_text('\n'.join(lines)+'\n')
print('Approved source asset SHA256',hashlib.sha256(src.read_bytes()).hexdigest())
print('RGB565A asset SHA256',hashlib.sha256(buf).hexdigest(),im.size)
