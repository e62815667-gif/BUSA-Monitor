"""Host regression of production key dispatch/focus, with synthetic LVGL geometry.
Page rebuilding and device transport are stubs; physical LCD/keys are separate.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
ui = root / 'firmware/BUSA_Display_N4R8'
u = (ui / 'BUSA_TouchUI.h').read_text()

def between(a, b):
    return u[u.index(a):u.index(b, u.index(a))]

# Compile the real focus and normal (non-editing) physical-key branch, not a copy.
focus = between('    struct KeyControl', '    int fieldIndex(')
dispatch = between('        using busa_focus::Nav;', '        if(k==0){')
section_actions = between('        if(id>=100&&id<106)', '        hideKeyboard();')
restore_focus = between('        orderKeys();\n', '        refresh();')
code = r'''
#include "BUSA_FocusOrder.h"
#include <cassert>
#include <cstdio>
struct lv_area_t { int x1,y1,x2,y2; };
struct lv_obj_t { lv_area_t area; bool focused=false; };
constexpr int LV_STATE_FOCUSED=1;
void lv_obj_get_coords(lv_obj_t* o,lv_area_t* a){*a=o->area;}
void lv_obj_set_style_outline_color(lv_obj_t*,int,int){}
void lv_obj_set_style_outline_width(lv_obj_t*,int,int){}
void lv_obj_set_style_outline_pad(lv_obj_t*,int,int){}
void lv_obj_clear_state(lv_obj_t* o,int){o->focused=false;}
void lv_obj_add_state(lv_obj_t* o,int){o->focused=true;}
struct Harness {
    static int color(int c){return c;}
    void focusStyle(lv_obj_t*,uint8_t){}
''' + focus + r'''
    KeyControl keyControls_[64]={};
    lv_obj_t controls[10]={};
    int keyCount_=0,keyFocus_=-1,page_=0,sub_=-1,requestedTheme_=0,theme_=0,graphKind_=0,calcMode_=0;
    bool sectionEntered_=false,collectionConfig_=false;
    uint32_t viewGeneration_=0;
    int pageOffset_=0,lastAction=-1,edits=0;
    void rememberCalc(){} void pageText(){}
    void beginKeyEdit(const KeyControl&){++edits;}
    void add(int id,int x,int y,uint8_t type=0){
        auto& o=controls[keyCount_];o.area={x,y,x+99,y+19};o.focused=false;
        registerKey(&o,type,id);
    }
    void render(){
        for(auto& o:controls)o.focused=false;
        keyCount_=0;keyFocus_=-1;++viewGeneration_;
        if(page_==4){add(320,0,0);add(321,120,0);add(322,0,30);add(323,120,30);add(0,0,65,1);}
        else if(page_==5&&sub_<0){add(400,0,0);add(401,120,0);add(402,0,30);add(403,120,30);}
        else if(page_==5){add(409,0,0);add(0,0,30,1);}
        else if(page_==1){add(0,0,0,1);add(0,120,0,1);}
''' + restore_focus + r'''
    }
    void action(int id){lastAction=id;
''' + section_actions + r'''
    }
    void key(uint8_t k){
''' + dispatch + r'''
    }
    int focusedId(){assert(keyFocus_>=0);assert(keyControls_[keyFocus_].obj->focused);return keyControls_[keyFocus_].id;}
};
int main(){
    Harness h;h.render();
    h.key(18);assert(h.page_==1&&!h.sectionEntered_);
    h.key(18);assert(h.page_==2&&!h.sectionEntered_);
    h.key(11);assert(!h.sectionEntered_&&h.keyFocus_==-1); // read-only page: do not trap
    h.key(19);assert(!h.sectionEntered_&&h.keyFocus_==-1);
    h.key(18);h.key(18);assert(h.page_==4&&!h.sectionEntered_);
    h.key(11);assert(h.sectionEntered_&&h.focusedId()==320); // enter without ENT
    h.key(18);assert(h.page_==4&&h.focusedId()==321);
    h.key(11);assert(h.focusedId()==323);
    h.key(16);assert(h.focusedId()==322);
    h.key(7);assert(h.focusedId()==320);
    h.key(18);h.key(19);assert(h.calcMode_==1&&h.focusedId()==321); // selection survives render
    h.key(15);assert(!h.sectionEntered_&&h.keyFocus_==-1);
    h.key(18);assert(h.page_==5&&!h.sectionEntered_);
    h.key(7);assert(h.sectionEntered_&&h.focusedId()==403); // enter from bottom
    h.key(16);assert(h.focusedId()==402);
    h.key(7);assert(h.focusedId()==400);
    h.key(18);assert(h.focusedId()==401);
    h.key(19);assert(h.sub_==1&&h.sectionEntered_&&h.focusedId()==409);
    h.key(15);assert(h.sub_==-1&&h.sectionEntered_&&h.focusedId()==401);
    h.key(15);assert(!h.sectionEntered_&&h.keyFocus_==-1);
    h.key(16);assert(h.page_==4&&!h.sectionEntered_);
    h.key(19);assert(h.sectionEntered_&&h.focusedId()==320); // ENT still enters
    h.key(11);h.key(11);assert(h.focusedId()==0);
    h.key(19);assert(h.edits==1); // ENT activates edit, not a section switch
    std::puts("PASS actual navigation dispatch: section selection, direct UP/DOWN entry, all four directions, activate, ESC hierarchy and read-only pages");
}
'''
with tempfile.TemporaryDirectory() as d:
    cpp = Path(d) / 'nav.cpp'
    exe = Path(d) / 'nav'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-I'+str(ui), str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
