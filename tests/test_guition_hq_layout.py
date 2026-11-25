"""Static layout/contract guard; physical typography and touch still need testing."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
u=(root/'firmware/BUSA_Display_N4R8/BUSA_TouchUI.h').read_text()
home=u[u.index('    void home(){'):u.index('    void temperatures(){')]
assert 'panel(middle,151,144)' not in home, 'legacy opaque temperature cards'
nums=dict((k,int(v)) for k,v in re.findall(r'\b(TOP|NAV|PAD|GAP|SCENE|METRICS|RELAYS)=(\d+)',u))
assert nums['TOP']+nums['NAV']+2*nums['PAD']+nums['SCENE']+nums['METRICS']+nums['RELAYS']+2*nums['GAP']==272
assert 'lv_img_set_antialias(im,true)' in home
for text in ('ТЭН откл.','Кран закрыт','Кран открыт','Авария','Израсходовано','Отбор'):
    assert text in home
assert 'Тихо' not in u
assert 'lv_obj_clear_flag(body_,LV_OBJ_FLAG_SCROLLABLE)' in u
assert 'busa_font_small' in u and 'busa_font_value' in u
assert 'for(int i=0;i<9;)' in u
assert 'lv_obj_set_height(self->body_,HEIGHT-TOP-128)' in u
print('PASS: native 480x272 layout contract, labels, fonts, and keyboard bounds')
