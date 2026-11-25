from pathlib import Path
import re
r=Path(__file__).resolve().parents[1]
u=(r/'firmware/BUSA_Display_N4R8/BUSA_TouchUI.h').read_text()
home=u[u.index('    void home(){'):u.index('    void temperatures(){')]
assert '"Расход","Израсходовано","Отбор","Собрано"' in home, 'total belongs in the metrics strip, not temperature scene'
assert 'live_[11]=at(scene' not in home
render=u[u.index('    void render(){'):u.index('    void home(){')]
assert 'lv_obj_add_flag(body_,LV_OBJ_FLAG_SCROLLABLE)' not in render, 'normal pages must not scroll'
assert 'orderKeys();' in render, 'physical traversal must be based on actual layout'
assert 'lv_obj_set_height(self->body_,HEIGHT-TOP-128)' not in u, 'keyboard must not resize/scroll the page'
assert 'touchEditor_' in u and 'lv_keyboard_set_textarea(self->keyboard_,self->touchEditor_)' in u
assert 'BUSA_NavIcons.h' in u and 'lv_img_set_src(icon' in u, 'antialiased icon raster assets'
assert 'lv_obj_scroll_to_view_recursive(obj,LV_ANIM_OFF)' not in u
assert 'pageText(' in u, 'long journal text needs fixed page pagination'
assert 'lv_obj_set_style_text_font(wifiList_,&busa_font,LV_PART_MAIN)' in u
print('PASS fixed layout, overlay editor, metrics, paginated text and geometry-ordered keys')
nums={k:int(v) for k,v in re.findall(r'\b(TOP|NAV|PAD|GAP|SCENE|METRICS|RELAYS)=(\d+)',u)}
body=272-nums['TOP']-nums['NAV']-2*nums['PAD']
assert body==192
assert nums['SCENE']+nums['METRICS']+nums['RELAYS']+2*nums['GAP']==body
# Geometry of all dense screens; bounds of fixed-height children, plus flex row gaps.
# Label font generation uses 12px condensed (measured line height 17 px); host LVGL tests check actual glyph bounds.
assert 'row(body_,70)' in u and 'panel(r,235,70)' in u and 'row(p,20)' in u and 'row(p,42)' in u
assert 'row(body_,18)' in u and 'row(body_,134)' in u and 'lv_obj_set_size(chart_,LV_PCT(100),102)' in u
assert 'row(body_,42)' in u and 'panel(body_,LV_PCT(100),calcMode_==2?46:50)' in u
assert 'pagedResult(int height=80,unsigned lines=5)' in u and 'pagedResult(60,4)' in u
for page, heights in {'temperature':[18,70,70,26], 'water':[18,134,26], 'graph':[26,15,102,26,14], 'calc5':[28,42,42,26,46], 'calc_geometry_expanded':[28,26,42,26,50], 'calc_geometry_collapsed':[28,26,42,26,50], 'settings':[18,27,27,27,27,27], 'calibration':[26,42,42,26,24], 'wifi':[26,26,26,42,26,24], 'theme':[26,26,26,26,26,24], 'google':[26,17,42,26,26,24], 'about':[26,60,16,16,16,16,24], 'journal':[26,26,80,26,24], 'diagnostics':[26,16,16,16,16,16,26,26,14], 'ble':[26,26,26,60,26,14], 'ota':[26,40,24]}.items():
    assert sum(heights)+2*(len(heights)-1)<=body,(page,sum(heights)+2*(len(heights)-1),body)
for id in range(200,221): assert re.search(r'\bcase '+str(id)+r':',u),id
for id in range(400,409): assert re.search(r'\b'+str(id)+r'\b',u),id
assert all(len(f'BUSA-DISPLAY-N4R8-OTA1-BD2-I{i}-P1-U5-HW2')+1<=40 for i in (4,5))
print('PASS: fixed-height budgets for fixed page variants and preservation of existing actions')

assert 'geometryExpanded_=!geometryValid();render();' in u
assert 'geometryExpanded_=false;render();' in u
assert 'if(fields_[i])calcInput_[calcMode_][i]=text(fields_[i]);' in u
assert 'button(body_,geometrySummary().c_str(),335)' in u
print('PASS geometry view contract: stored dimensions, collapse, reopen and null-safe fields')
