"""Antialiased 22x20 native RGB565A navigation icons; no new dependencies."""
from pathlib import Path
from PIL import Image, ImageDraw
W,H,S=22,20,4
icons=[]
def render(fn):
    im=Image.new('L',(W*S,H*S));d=ImageDraw.Draw(im)
    def line(points,w=1.65):
        pts=[(round(x*S),round(y*S)) for x,y in points];d.line(pts,fill=255,width=round(w*S),joint='curve')
        r=round(w*S/2)
        for x,y in (pts[0],pts[-1]):d.ellipse((x-r,y-r,x+r,y+r),fill=255)
    def ellipse(box,w=1.5):d.ellipse(tuple(round(x*S) for x in box),outline=255,width=round(w*S))
    fn(line,ellipse);a=im.resize((W,H),Image.Resampling.LANCZOS)
    # White source is tinted at runtime for light/dark themes.
    icons.append(bytes(v for alpha in a.getdata() for v in (255,255,alpha)))
render(lambda l,e:(l([(2,9),(11,1.5),(20,9)]),l([(5,8),(5,18),(9,18),(9,12),(13,12),(13,18),(17,18),(17,8)])))
render(lambda l,e:(l([(8,12),(8,4),(9,2),(13,2),(14,4),(14,12)]),e((6,11,16,19)),l([(11,6),(11,15)],1.5),e((9.5,13.5,12.5,16.5),1.2)))
render(lambda l,e:(l([(11,1.5),(6,8),(3.5,13),(4.5,16),(7.5,18),(11,18.5),(14.5,18),(17.5,16),(18.5,13),(16,8),(11,1.5)]),l([(7,12),(7,14.5),(9,16)],1.2)))
render(lambda l,e:(l([(3,2),(3,18),(19,18)]),l([(5.5,13.5),(9,9),(13,12),(19,4)]),l([(15.5,4),(19,4),(19,7.5)],1.3)))
render(lambda l,e:(l([(5,1.5),(17,1.5),(17,18.5),(5,18.5),(5,1.5)]),l([(7.5,5),(14.5,5)],1.3),*[l([(x,y),(x+1.5,y)],1.5) for y in (9,12.5,16) for x in (7.5,12.5)]))
render(lambda l,e:(l([(8,2),(14,2),(15,5),(18,5.5),(19.5,8),(17.5,10),(19.5,12),(18,15),(15,15),(14,18),(8,18),(7,15),(4,15),(2.5,12),(4.5,10),(2.5,8),(4,5.5),(7,5),(8,2)],1.4),e((8,7,14,13),1.6)))
assert len(icons)==6 and all(len(a)==W*H*3 for a in icons)
assert all(any(0<a<255 for a in icon[2::3]) for icon in icons)
out=['#pragma once','#include <stdint.h>',f'constexpr int BUSA_NAV_W={W},BUSA_NAV_H={H};',f'static const uint8_t BUSA_NAV_ICONS[6][{W*H*3}]={{']
for a in icons:
    out.append('{');out.extend(','.join(str(v) for v in a[i:i+30])+',' for i in range(0,len(a),30));out.append('},')
out.append('};')
Path('firmware/BUSA_Display_N4R8/BUSA_NavIcons.h').write_text('\n'.join(out)+'\n')
print('PASS: six native antialiased RGB565A icons')
