"""Host LVGL 8.4 layout regression test with generated production fonts.
Builds the exact production widget helpers and page builders, with only
transport and calculator arithmetic stubbed (not layout or font metrics).
The hardware's LCD/touch/photographic acceptance remains separate.
"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import subprocess, sys, tempfile
root=Path(__file__).resolve().parents[1]
sketch=root/'firmware/BUSA_Display_N4R8'
u=(sketch/'BUSA_TouchUI.h').read_text()
# Reuse exact production implementations instead of duplicating their dimensions.
def between(a,b):
    return u[u.index(a):u.index(b,u.index(a))]
helpers=between('    static const char* sensor(', '    static void flush(')
widgets=between('    lv_obj_t* label(', '    void navIcon(')
status=between('    lv_obj_t* statusLabel(', '    void water()')
calc=between('    static const char* calcTitle(', '    void rememberCalc()')+between('    void calculator(){','    void pageText(){')
prelude=r'''
#include <lvgl.h>
#include "BUSA_TouchKeys.h"
#include "BUSA_NavIcons.h"
#include "BUSA_GraphView.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
using String=std::string;
constexpr int WIDTH=480,HEIGHT=272;
LV_FONT_DECLARE(busa_font);
LV_FONT_DECLARE(busa_font_small);
LV_FONT_DECLARE(busa_font_value);
LV_FONT_DECLARE(busa_font_nav);
struct JsonStub { JsonStub operator[](const char*) const {return {};} JsonStub operator[](int) const {return {};} };
struct SessionStub {JsonStub snapshot()const{return {};}};
struct UiTest {
    static constexpr uint32_t colors_[4]={0x38bdf8,0x53d486,0xf4c84a,0xff6473};
    uint8_t theme_=2; int calcMode_=0; bool geometryExpanded_=true;
    String geometrySummary()const{return "Размеры куба: 40 × 80 см · изменить";}
    lv_obj_t *body_=nullptr,*nav_=nullptr,*pageStatus_=nullptr,*resultLabel_=nullptr;
    lv_obj_t *fields_[8]={},*live_[16]={}; String calcInput_[4][5]; SessionStub session_;
    lv_obj_t *touchModal_=nullptr,*touchTitle_=nullptr,*touchEditor_=nullptr,*keyboard_=nullptr;
    static void keyboardEvent(lv_event_t*){}
    lv_img_dsc_t navArt_[6]{};lv_obj_t* navButtons_[6]={};
    std::vector<lv_obj_t*> controls;
    static void event(lv_event_t*){} static void textFocus(lv_event_t*){}
    void registerKey(lv_obj_t* o,int,int){controls.push_back(o);}
    static String value(JsonStub){return {};}
    void calculate(bool){}
    int graphKind_=0;static constexpr int graphMinutes_=5;bool graphDirty_=false;
    lv_obj_t *chart_=nullptr,*graphNotice_=nullptr,*graphUnit_=nullptr,*graphLegend_[4]={},*graphPlots_[4]={},*graphY_[4][4]={},*graphX_[4]={};lv_chart_series_t* series_[4]={};
    void requestGraph(){} void set(lv_obj_t* l,const String& s){if(l)lv_label_set_text(l,s.c_str());}
'''
checks=r'''
    void init(){
        lv_obj_t* root=lv_scr_act();lv_obj_clean(root);lv_obj_remove_style_all(root);
        lv_obj_set_style_bg_color(root,color(paper()),0);lv_obj_set_style_bg_opa(root,LV_OPA_COVER,0);
        lv_obj_set_style_text_font(root,&busa_font,0);lv_obj_clear_flag(root,LV_OBJ_FLAG_SCROLLABLE);
        body_=lv_obj_create(root);lv_obj_set_pos(body_,0,TOP);lv_obj_set_size(body_,WIDTH,HEIGHT-TOP-NAV);
        lv_obj_set_style_border_width(body_,0,0);lv_obj_set_style_radius(body_,0,0);
        lv_obj_set_style_pad_all(body_,PAD,0);lv_obj_set_style_pad_row(body_,2,0);
        lv_obj_set_flex_flow(body_,LV_FLEX_FLOW_COLUMN);lv_obj_clear_flag(body_,LV_OBJ_FLAG_SCROLLABLE);
        nav_=nullptr;controls.clear();for(auto& p:fields_)p=nullptr;for(auto& p:live_)p=nullptr;
    }
    void assertFits(lv_obj_t* o){
        lv_area_t a;lv_obj_get_coords(o,&a);
        for(auto* p=lv_obj_get_parent(o);p;p=lv_obj_get_parent(p)){
            lv_area_t b;lv_obj_get_content_coords(p,&b);
            if(a.x1<b.x1||a.y1<b.y1||a.x2>b.x2||a.y2>b.y2){
                std::fprintf(stderr,"CLIPPED mode=%d theme=%d object=[%d,%d,%d,%d] parent=[%d,%d,%d,%d]\n",calcMode_,theme_,a.x1,a.y1,a.x2,a.y2,b.x1,b.y1,b.x2,b.y2);
                std::abort();
            }
            if(p==body_)break;
        }
    }
    void labelFits(lv_obj_t* l){
        assertFits(l);const char* text=lv_label_get_text(l);auto* f=lv_obj_get_style_text_font(l,0);
        const int width=lv_obj_get_content_width(l),height=lv_obj_get_content_height(l);
        lv_point_t size;lv_txt_get_size(&size,text,f,lv_obj_get_style_text_letter_space(l,0),lv_obj_get_style_text_line_space(l,0),width,LV_TEXT_FLAG_NONE);
        if(size.y>height||std::strstr(text,"...")!=nullptr){
            std::fprintf(stderr,"TEXT CLIPPED '%s' need %dpx high, have %dpx; width %d\n",text,size.y,height,width);std::abort();
        }
    }
    void verifyGraphs(){
        for(int mode=0;mode<3;++mode){
            init();graphKind_=mode;resultLabel_=nullptr;pageStatus_=nullptr;graphUnit_=nullptr;
            for(auto& p:graphPlots_)p=nullptr;for(auto& p:graphLegend_)p=nullptr;for(auto& p:graphX_)p=nullptr;
            graphs();verify();
            assert(controls.size()==3);
            assert(lv_obj_get_child_cnt(body_)==3);
            const int n=mode==0?4:1;
            for(int k=0;k<n;++k){
                assertFits(graphPlots_[k]);
                assert(lv_chart_get_series_next(graphPlots_[k],nullptr)==series_[k]);
                assert(!lv_chart_get_series_next(graphPlots_[k],series_[k]));
                assert(lv_obj_get_height(graphPlots_[k])==(mode==0?65:123));
                assert(lv_obj_get_style_line_width(graphPlots_[k],LV_PART_ITEMS)==(mode==0?2:3));
                for(int j=0;j<(mode==0?2:4);++j)labelFits(graphY_[k][j]);
                if(mode==0){lv_label_set_text(graphLegend_[k],k==3?"Авария 26,8 °C":k==0?"Вода 28,1 °C":k==1?"Колонна 26,6 °C":"Куб 30,9 °C");lv_obj_update_layout(body_);labelFits(graphLegend_[k]);
                    assert(lv_obj_get_style_text_align(graphLegend_[k],0)==LV_TEXT_ALIGN_CENTER);
                    lv_area_t title,plot;lv_obj_get_content_coords(graphLegend_[k],&title);lv_obj_get_coords(graphPlots_[k],&plot);
                    assert(abs((title.x1+title.x2)-(plot.x1+plot.x2))<=2);}
            }
            if(mode==0){
                lv_area_t a,b,c,d;lv_obj_get_coords(graphPlots_[0],&a);lv_obj_get_coords(graphPlots_[2],&b);lv_obj_get_coords(graphPlots_[1],&c);lv_obj_get_coords(graphPlots_[3],&d);
                assert(a.y1==b.y1&&c.y1==d.y1&&a.y2<c.y1&&a.x2<b.x1&&a.x1==c.x1&&b.x1==d.x1);
            }else{assert(!graphPlots_[1]);labelFits(graphUnit_);}
            assert(!pageStatus_);
            for(int k=0;k<4;++k){if(mode==0)assert(!graphX_[k]);else labelFits(graphX_[k]);}
            labelFits(graphNotice_);
            graphNotice("");assert(lv_obj_has_flag(graphNotice_,LV_OBJ_FLAG_HIDDEN));
            graphNotice("Нет связи");assert(!lv_obj_has_flag(graphNotice_,LV_OBJ_FLAG_HIDDEN));lv_obj_update_layout(chart_);labelFits(graphNotice_);
        }
        std::puts("PASS native LVGL graph pages, distinct series, larger plot, axis geometry, unobtrusive status and trace width");
    }
    void verifyNavigation(){
        init();buildNavigation(lv_scr_act());lv_obj_update_layout(nav_);
        for(int i=0;i<6;++i){
            auto* b=navButtons_[i];auto* l=lv_obj_get_child(b,0);auto* im=lv_obj_get_child(b,1);
            assertFits(b);labelFits(l);assertFits(im);
            assert(lv_obj_get_width(im)==20&&lv_obj_get_height(im)==20);
            assert(lv_img_get_zoom(im)==LV_IMG_ZOOM_NONE);
            lv_area_t x,y,z;lv_obj_get_coords(im,&x);lv_obj_get_coords(l,&y);lv_obj_get_coords(b,&z);
            assert(y.y1>x.y2);
            assert(abs((x.x1+x.x2)-(z.x1+z.x2))<=2);
            if(i){lv_area_t prev;lv_obj_get_coords(navButtons_[i-1],&prev);assert(z.x1-prev.x2>=4);}
        }
        std::puts("PASS native navigation: six equal controls, square images, text separation and gaps");
    }
    void verifyKeyboard(){
        init();buildTouchModal(lv_scr_act());lv_obj_clear_flag(touchModal_,LV_OBJ_FLAG_HIDDEN);
        lv_obj_update_layout(touchModal_);
        for(unsigned m=0;m<4;++m){
            auto mode=lv_keyboard_mode_t(m);
            lv_keyboard_set_mode(keyboard_,mode);
            const auto* f=mode==LV_KEYBOARD_MODE_NUMBER?&busa_font_value:&busa_font;
            lv_obj_set_style_text_font(keyboard_,f,LV_PART_ITEMS);
            lv_obj_update_layout(keyboard_);
            auto* matrix=reinterpret_cast<lv_btnmatrix_t*>(keyboard_);
            assert(matrix->btn_cnt>0);
            for(unsigned i=0;i<matrix->btn_cnt;++i){
                const char* key=lv_btnmatrix_get_btn_text(keyboard_,i);assert(key);
                uint32_t at=0;while(key[at]){uint32_t cp=_lv_txt_encoded_next(key,&at);lv_font_glyph_dsc_t d;assert(lv_font_get_glyph_dsc(f,&d,cp,0));}
                const auto& a=matrix->button_areas[i];
                assert(lv_txt_get_width(key,strlen(key),f,0,LV_TEXT_FLAG_NONE)<=a.x2-a.x1+1);
                assert(lv_font_get_line_height(f)<=a.y2-a.y1+1);
            }
        }
        lv_keyboard_set_mode(keyboard_,LV_KEYBOARD_MODE_NUMBER);
        lv_textarea_set_accepted_chars(touchEditor_,"0123456789.-,");
        lv_textarea_set_text(touchEditor_,"");
        for(auto* k:{"4","0",",","5",","})busa_touch_keys::apply(keyboard_,touchEditor_,k);
        assert(!strcmp(lv_textarea_get_text(touchEditor_),"40,5"));
        busa_touch_keys::apply(keyboard_,touchEditor_,"+/-");assert(!strcmp(lv_textarea_get_text(touchEditor_),"-40,5"));
        busa_touch_keys::apply(keyboard_,touchEditor_,"+/-");assert(!strcmp(lv_textarea_get_text(touchEditor_),"40,5"));
        busa_touch_keys::apply(keyboard_,touchEditor_,"Удалить");assert(!strcmp(lv_textarea_get_text(touchEditor_),"40,"));
        busa_touch_keys::apply(keyboard_,touchEditor_,"Очистить");assert(!strlen(lv_textarea_get_text(touchEditor_)));
        lv_textarea_set_accepted_chars(touchEditor_,nullptr);lv_keyboard_set_mode(keyboard_,LV_KEYBOARD_MODE_TEXT_LOWER);
        for(auto* k:{"a","ABC","B","123","@","Пробел","7"})busa_touch_keys::apply(keyboard_,touchEditor_,k);
        assert(!strcmp(lv_textarea_get_text(touchEditor_),"aB@ 7"));
        assert(lv_obj_get_height(touchModal_)==HEIGHT-TOP);
        assert(lv_obj_get_height(touchEditor_)==36);
        std::puts("PASS modal keyboard: all modes, glyph coverage, key bounds, decimal, sign, deletion and text");
    }
    void verify(){
        lv_obj_update_layout(body_);
        for(auto* o:controls){
            assertFits(o);
            if(lv_obj_check_type(o,&lv_textarea_class)){
                auto* box=lv_obj_get_parent(o);auto* caption=lv_obj_get_child(box,0);labelFits(caption);
                const char* title=lv_label_get_text(caption);auto* f=lv_obj_get_style_text_font(caption,0);
                assert(lv_txt_get_width(title,strlen(title),f,0,LV_TEXT_FLAG_NONE)<=lv_obj_get_content_width(caption));
                assert(lv_obj_get_height(o)==24);
                assert(lv_obj_get_content_height(o)>=lv_font_get_line_height(lv_obj_get_style_text_font(o,0)));
            }else labelFits(lv_obj_get_child(o,0));
        }
        if(resultLabel_)labelFits(resultLabel_);
    }
};
constexpr uint32_t UiTest::colors_[4];
static lv_color_t pixels[WIDTH*24];
static lv_disp_draw_buf_t buf;
static lv_disp_drv_t drv;
static void flush(lv_disp_drv_t* d,const lv_area_t*,lv_color_t*){lv_disp_flush_ready(d);}
int main(){
    lv_init();lv_disp_draw_buf_init(&buf,pixels,nullptr,WIDTH*24);lv_disp_drv_init(&drv);
    drv.hor_res=WIDTH;drv.ver_res=HEIGHT;drv.draw_buf=&buf;drv.flush_cb=flush;assert(lv_disp_drv_register(&drv));
    UiTest ui;
    for(int theme=0;theme<3;++theme){ui.theme_=theme;
        for(int mode=0;mode<4;++mode){ui.calcMode_=mode;ui.init();ui.calculator();ui.verify();
            {lv_area_t a,b,c,d;lv_obj_get_coords(ui.controls[0],&a);lv_obj_get_coords(ui.controls[1],&b);lv_obj_get_coords(ui.controls[2],&c);lv_obj_get_coords(ui.controls[3],&d);assert(a.y1==b.y1&&c.y1==d.y1&&a.y2<c.y1&&a.x1==c.x1&&b.x1==d.x1);}
            if(mode==2){lv_label_set_text(ui.resultLabel_,"Абсолютный спирт: 100,00 л АС   Головы: 10,00 л АС\nХвосты: 10,00 л АС   Выход продукта: 100,00 л");lv_obj_update_layout(ui.body_);ui.labelFits(ui.resultLabel_);}}
        ui.calcMode_=3;ui.geometryExpanded_=false;ui.init();ui.calculator();ui.verify();
        assert(!ui.fields_[0]&&!ui.fields_[1]&&ui.fields_[2]);
        ui.geometryExpanded_=true;ui.init();ui.calculator();ui.verify();
        assert(ui.fields_[0]&&ui.fields_[1]&&ui.fields_[2]);
        ui.init();ui.resultLabel_=nullptr;ui.temperatures();ui.verify();ui.verifyKeyboard();ui.verifyNavigation();ui.verifyGraphs();
    }
    ui.theme_=1;assert(ui.paper()==0x080808&&ui.surface()==0x202020);
    ui.theme_=2;assert(ui.paper()==0x060e15&&ui.surface()==0x0d2430);
    ui.theme_=0;assert(ui.paper()==0xf0f6f8&&ui.surface()==0xffffff);
    std::puts("PASS native LVGL: production calculator and temperature controls, text and font bounds in all three themes");
}
'''
if len(sys.argv)!=2: raise SystemExit('usage: test_guition_lvgl_layout.py /path/to/lvgl')
lvgl=Path(sys.argv[1]).resolve()
assert (lvgl/'lvgl.h').is_file(),lvgl
sources=sorted((lvgl/'src').rglob('*.c'))+[sketch/f'{name}.c' for name in ('busa_font','busa_font_small','busa_font_value','busa_font_nav')]
flags=['-DLV_CONF_SKIP','-DLV_MEM_SIZE=98304','-I'+str(lvgl),'-I'+str(lvgl/'src'),'-I'+str(sketch),'-I'+str(lvgl.parent/'ArduinoJson'/'src')]
with tempfile.TemporaryDirectory() as tmp:
    tmp=Path(tmp);cpp=tmp/'layout.cpp';cpp.write_text(prelude+helpers+widgets+status+calc+between('    void graphAxes(', '    void graphTimeLabels()')+between('    void graphNotice(', '    void requestGraph(){')+between('    void navIcon(', '    static String text(')+between('    void buildTouchModal(', '    void build(){')+checks)
    def compile_one(item):
        i,s=item;o=tmp/f'{i}.o';subprocess.run(['gcc','-std=gnu99','-O0',*flags,'-c',str(s),'-o',str(o)],check=True);return str(o)
    with ThreadPoolExecutor(max_workers=2) as pool: objects=list(pool.map(compile_one,enumerate(sources)))
    exe=tmp/'layout';subprocess.run(['g++','-std=c++11','-O0',*flags,str(cpp),*objects,'-lm','-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
