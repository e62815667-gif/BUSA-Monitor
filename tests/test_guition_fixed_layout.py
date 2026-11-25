from pathlib import Path
import re
r=Path(__file__).resolve().parents[1]
u=(r/'firmware/BUSA_Display_N4R8/BUSA_TouchUI.h').read_text()
home=u[u.index('    void home(){'):u.index('    void temperatures(){')]
assert '"Расход","Израсходовано","Отбор","Собрано"' in home, 'total belongs in the metrics strip, not temperature scene'
assert 'live_[11]=at(scene' not in home
homeTemps=u[u.index('    void homeTemperatures('):u.index('    void home(){')]
assert 'homeTemperatures(scene)' in home
assert 'ВКЛ' not in homeTemps and 'ВЫКЛ' not in homeTemps and 'live_[12+i]' not in u
assert 'LV_TEXT_ALIGN_RIGHT' in homeTemps
assert 'set(live_[4+i],measured(d["temperatures"][i]["c"]," °C",1))' in u
assert 'ВКЛ, °C' in u and 'ВЫКЛ, °C' in u
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
assert 'row(body_,18)' in u and 'row(body_,172)' in u and 'lv_obj_set_size(c,width-axisWidth-3,height)' in u
assert 'row(body_,42)' in u and 'panel(body_,LV_PCT(100),calcMode_==2?36:50)' in u
assert 'pagedResult(int height=80,unsigned lines=5)' in u and 'pagedResult(60,4)' in u
for page, heights in {'temperature':[18,70,70,26], 'water':[18,172], 'liquid_settings':[26,28,76,26,24], 'graph_temp':[24,83,83], 'graph_flow':[24,15,149], 'calc5':[42,42,42,22,36], 'calc_geometry_expanded':[42,26,42,22,50], 'calc_geometry_collapsed':[42,26,42,22,50], 'settings':[18,27,27,27,27,27], 'calibration':[26,42,42,26,24], 'wifi':[26,26,26,42,26,24], 'theme':[26,26,26,26,26,24], 'google':[26,17,42,26,26,24], 'about':[26,60,16,16,16,16,24], 'journal':[26,26,80,26,24], 'diagnostics':[26,16,16,16,16,16,26,26,14], 'ble':[26,26,26,60,26,14], 'ota':[26,40,24]}.items():
    gap=1 if page=='graph_temp' else 2
    assert sum(heights)+gap*(len(heights)-1)<=body,(page,sum(heights)+gap*(len(heights)-1),body)
for id in range(200,221): assert re.search(r'\bcase '+str(id)+r':',u),id
for id in list(range(400,409))+[412,413,414,222]: assert re.search(r'\b'+str(id)+r'\b',u),id
assert all(len(f'BUSA-DISPLAY-N4R8-OTA1-BD2-I{i}-P1-UH-HW2')+1<=40 for i in (4,5))
print('PASS: fixed-height budgets for fixed page variants and preservation of existing actions')

assert 'geometryExpanded_=!geometryValid();render();' in u
assert 'geometryExpanded_=false;render();' in u
assert 'if(fields_[i])calcInput_[calcMode_][i]=text(fields_[i]);' in u
assert 'button(body_,geometrySummary().c_str(),335)' in u
print('PASS geometry view contract: stored dimensions, collapse, reopen and null-safe fields')

# Browser labels, notes, result labels/units are the reference, not duplicated guesses.
web=(r/'calculator/ui.js').read_text()
calc=u[u.index('    static const char* calcTitle'):u.index('    void pageText')]
result=u[u.index('    void calculate('):u.index('    void handleResult')]
for line in web.splitlines():
    if not line.strip().startswith('{name:'): continue
    name=re.search(r"name:'([^']+)'",line).group(1)
    assert '"'+name+'"' in calc,name
    for label,key in re.findall(r"\['([^']+)','([^']+)'\]",line.split('note:')[0]):
        assert '"'+label+'"' in calc,label
    note=re.search(r"note:'([^']+)'",line).group(1)
    assert '"'+note+'"' in calc,note
    for label,key,unit in re.findall(r"\['([^']+)','([^']+)','([^']+)'\]",line.split('results:')[1]):
        assert label+':' in result,label
        assert 'fmt(r.'+key+')' in result,key
        assert '" '+unit+'"' in result,unit
print('PASS exact browser calculator terminology, notes and result units')

assert 'for(int n=0;n<2;++n)' in calc and 'int i=n*2+col' in calc
assert 'static_assert(BUSA_NAV_W==BUSA_NAV_H' in u
print('PASS square icons and 2x2 calculator tabs contract')

assert 'graphDirty_||now-lastGraph_>=2000' in u
assert 'static constexpr int graphMinutes_=5;' in u
assert 'graphRange_' not in u and 'id>=310&&id<=314' not in u
assert 'String(graphMinutes_)' in u and 'graphKind_,graphMinutes_,scale' in u
assert 'pendingGeneration_!=viewGeneration_' in u
assert 'makeGraph(p,k,235,65,colors_[k])' in u and 'temperatureBounds(arr)' in u
print('PASS graph switching, periods, generation isolation and distinct series contract')

graphs=u[u.index('    void graphs(){'):u.index('    void requestGraph(){')]
assert 'spec.title' not in graphs and 'pageStatus_=' not in graphs
assert 'lv_obj_set_style_pad_row(body_,1,0)' in graphs
assert 'lv_obj_set_style_text_align(graphLegend_[k],LV_TEXT_ALIGN_CENTER,0)' in graphs
assert 'graphTimeLabels' not in u and 'graphX_' not in u
assert 'color(0x000000)' in u and 'lv_chart_set_div_line_count(c,graphKind_==0?5:9,graphKind_==0?11:23)' in u
assert 'graphKind_==0?"%.0f":"%.1f"' in u
assert 'static String fmt(double v){String s(v,1);' in u
assert 'makeGraph(body_,0,474,149,spec.color)' in graphs
assert 'lv_obj_set_style_line_width(c,graphKind_==0?2:3,LV_PART_ITEMS)' in u
assert 'if(route!=Route::chartHistory)set(pageStatus_,"Выполняется…")' in u
print('PASS expanded plot, unobtrusive status and stronger traces')
