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
assert 'homeArt_=plain(lv_scr_act(),BUSA_ASSET_W,BUSA_ASSET_H)' in home, 'full size art must be independent of clipped scene'
assert 'lv_obj_set_pos(homeArt_,(WIDTH-BUSA_ASSET_W)/2,7)' in home, 'approved image is centered within the screen'
assert 'lv_obj_move_foreground(touchModal_)' in home, 'modal must stay above image'
assert 'if(homeArt_){lv_obj_del(homeArt_);homeArt_=nullptr;}' in u, 'art must not leak across pages'
assert nums['METRICS']>=30 and nums['RELAYS']==0 and nums['SCENE']>=150
assert 'setBg(statusDots_[i]' in u and 'homeTempFrames_[i]' in u and 'homeAlarm_' in u
for text in ('ТЭН откл.','ТЭН вкл.','Кран закрыт','Кран открыт','Авария','Объём','Отбор'):
    assert text in u
assert 'Тихо' not in u
assert 'lv_obj_clear_flag(body_,LV_OBJ_FLAG_SCROLLABLE)' in u
assert 'busa_font_small' in u and 'busa_font_value' in u
assert 'for(int i=0;i<10;)' in u
assert 'lv_keyboard_set_textarea(self->keyboard_,self->touchEditor_)' in u
assert 'lv_obj_set_height(self->body_,HEIGHT-TOP-128)' not in u
print('PASS: native 480x272 layout contract, labels, fonts, and keyboard bounds')
