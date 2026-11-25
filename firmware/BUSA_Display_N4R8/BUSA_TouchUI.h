#pragma once
#include <lvgl.h>
#include <Preferences.h>
#include <initializer_list>
#include <BUSA_KeyEvents.h>
#include "BUSA_GuitionHardware.h"
#include "BUSA_DisplaySession.h"
#include "BUSA_TouchCalibration.h"
#include "BUSA_Calculator.h"
#include "BUSA_DisplayAsset.h"
#include "BUSA_FocusOrder.h"
#include "BUSA_TouchKeys.h"
#include "BUSA_NavIcons.h"
#include "BUSA_GraphView.h"
LV_FONT_DECLARE(busa_font);
LV_FONT_DECLARE(busa_font_small);
LV_FONT_DECLARE(busa_font_value);
LV_FONT_DECLARE(busa_font_nav);
namespace busa_guition {
class Interface {
public:
    Interface(Hardware& hw,DisplaySession& session):hw_(hw),session_(session){}
    bool begin(){
        if(!hw_.lcdReady()){Serial.println("[BOOT] LVGL SKIP LCD/CANVAS");return false;}
        lv_init();
        Serial.println("[BOOT] LVGL INIT OK");
        static_assert(LV_COLOR_DEPTH==16 && LV_COLOR_16_SWAP==0,"Native RGB565 required");
        auto* pixels=static_cast<lv_color_t*>(heap_caps_malloc(WIDTH*24*sizeof(lv_color_t),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
        if(!pixels){Serial.println("[BOOT] LVGL BUFFER FAIL");return false;}
        lv_disp_draw_buf_init(&drawBuffer_,pixels,nullptr,WIDTH*24);
        lv_disp_drv_init(&driver_);driver_.hor_res=WIDTH;driver_.ver_res=HEIGHT;driver_.draw_buf=&drawBuffer_;
        driver_.flush_cb=flush;driver_.user_data=this;
        if(!lv_disp_drv_register(&driver_)){Serial.println("[BOOT] LVGL DISPLAY FAIL");return false;}
        Serial.println("[BOOT] LVGL DISPLAY OK");
        lv_indev_drv_init(&input_);input_.type=LV_INDEV_TYPE_POINTER;input_.read_cb=readTouch;input_.user_data=this;
        if(!lv_indev_drv_register(&input_)||!input_.read_timer){Serial.println("[BOOT] LVGL INPUT FAIL");return false;}
        Serial.println("[BOOT] LVGL INPUT OK");
        lv_timer_set_period(input_.read_timer,busa_touch::readPeriodMs);
        prefs_.begin("busa-touch",false);
        calibrationValid_=prefs_.getBytesLength("affineC0")==sizeof(transform_)&&prefs_.getBytes("affineC0",&transform_,sizeof(transform_))==sizeof(transform_)&&validTransform();
        theme_=prefs_.getUChar("theme",2);if(theme_>2)theme_=2;
        asset_=static_cast<uint8_t*>(heap_caps_malloc(BUSA_ASSET_SIZE,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
        if(asset_){
            if(!hw_.readAsset("/busa/rig.rgb565a",asset_,BUSA_ASSET_SIZE))memcpy(asset_,BUSA_ASSET,BUSA_ASSET_SIZE);
            else if(memcmp(asset_,BUSA_ASSET,BUSA_ASSET_SIZE)!=0){hw_.invalidAsset();memcpy(asset_,BUSA_ASSET,BUSA_ASSET_SIZE);}
        }
        art_.header.always_zero=0;art_.header.cf=LV_IMG_CF_TRUE_COLOR_ALPHA;
        art_.header.w=BUSA_ASSET_W;art_.header.h=BUSA_ASSET_H;art_.data_size=BUSA_ASSET_SIZE;art_.data=asset_?asset_:BUSA_ASSET;
        lastTick_=millis();
        Serial.println("[BOOT] LVGL SCREEN BEGIN");
        if(calibrationValid_)build();else startCalibration();
        Serial.println("[BOOT] LVGL SCREEN OK");
        hw_.backlight(true);return true;
    }
    void poll(){
        const uint32_t now=millis();lv_tick_inc(now-lastTick_);lastTick_=now;
        if(calibrating_)calibrationPoll();
        if(testOverlay_&&int32_t(now-testUntil_)>=0){lv_obj_del(testOverlay_);testOverlay_=nullptr;}
        if(!session_.connected()){keyCursor_.reset();if(touchTarget_)hideKeyboard();if(displaySleeping_){hw_.backlight(true);displaySleeping_=false;}if(keyEdit_||keyConfirm_>=0)cancelKeyEdit(true);}
        int code;String body;
        if(session_.takeApiResult(code,body)){waiting_=false;handleResult(code,body);}
        if(!calibrating_){
            if(revision_!=session_.snapshotRevision()||wasConnected_!=session_.connected()){
                revision_=session_.snapshotRevision();wasConnected_=session_.connected();
                if(!themeSynced_&&session_.connected()&&!waiting_&&!keyEdit_&&!touchTarget_&&!calcNotes_&&!testOverlay_&&page_==0){
                    auto mode=session_.snapshot()["tft_theme_mode"];
                    if(mode.is<uint8_t>()&&mode.as<uint8_t>()<=2){themeSynced_=true;if(mode.as<uint8_t>()!=theme_){theme_=mode.as<uint8_t>();prefs_.putUChar("theme",theme_);build();}}
                }
                refresh();consumeKeys(now);
            }
            if(page_==0&&now-lastAnimation_>=60){lastAnimation_=now;animate(now);}
            if(page_==3&&!waiting_&&session_.connected()&&(graphDirty_||now-lastGraph_>=2000))requestGraph();
            if(scanPending_&&!waiting_&&now-lastScan_>=1200&&page_==5&&sub_==1){lastScan_=now;send(ApiRoute::wifiScan,{});}
        }
        lv_timer_handler();
    }
private:
    using Route=ApiRoute;
    struct Arg {String key,value;};
    static constexpr uint32_t colors_[4]={0x38bdf8,0x53d486,0xf4c84a,0xff6473};
    static const char* sensor(uint8_t i){static const char* names[]={"Вода","Колонна","Куб","Верх колонны"};return names[i];}
    static lv_color_t color(uint32_t c){return lv_color_hex(c);}
    uint32_t ink()const{return theme_==0?0x19384b:theme_==1?0xeeeeee:0xe6f2f7;}
    uint32_t paper()const{return theme_==0?0xf0f6f8:theme_==1?0x080808:0x060e15;}
    uint32_t surface()const{return theme_==0?0xffffff:theme_==1?0x202020:0x0d2430;}
    uint32_t line()const{return theme_==0?0xb6d2df:theme_==1?0x454545:0x4d8b9e;}
    uint32_t accent()const{return theme_==0?0x007bb3:theme_==1?0xaaaaaa:0x009fde;}
    uint32_t muted()const{return theme_==0?0x536e7f:theme_==1?0xb0b0b0:0x9cb4c2;}
    void glass(lv_obj_t* o,bool active=false){
        if(theme_!=2){lv_obj_set_style_bg_grad_dir(o,LV_GRAD_DIR_NONE,0);return;}
        if(!active)lv_obj_set_style_bg_color(o,color(0x153b4b),0);
        lv_obj_set_style_bg_grad_color(o,color(active?0x0088d3:0x081822),0);
        lv_obj_set_style_bg_grad_dir(o,LV_GRAD_DIR_VER,0);
    }
    static constexpr int TOP=30, NAV=44, PAD=3, GAP=3, SCENE=124, METRICS=32, RELAYS=30;
    static_assert(TOP+NAV+2*PAD+SCENE+METRICS+RELAYS+2*GAP==272,"Home must fit native display");
    static void flush(lv_disp_drv_t* d,const lv_area_t* a,lv_color_t* p){
        auto* self=static_cast<Interface*>(d->user_data);
        if(!self->frameOpen_){self->frameStart_=millis();self->frameOpen_=true;}
        self->hw_.panel().draw16bitRGBBitmap(a->x1,a->y1,reinterpret_cast<uint16_t*>(p),a->x2-a->x1+1,a->y2-a->y1+1);
        self->hw_.noteDirtyRows(a->y1,a->y2);
        if(lv_disp_flush_is_last(d)){self->hw_.present();self->hw_.frameComplete(self->frameStart_);self->frameOpen_=false;}
        lv_disp_flush_ready(d);
    }
    static void readTouch(lv_indev_drv_t* d,lv_indev_data_t* data){
        auto* self=static_cast<Interface*>(d->user_data);TS_Point raw;
        data->state=LV_INDEV_STATE_REL;
        if(self->calibrating_||self->displaySleeping_||!self->calibrationValid_||!self->hw_.readRawTouch(raw))return;
        auto p=busa_touch::apply(self->transform_,{raw.x,raw.y});
        data->point.x=constrain(p.x,0,WIDTH-1);data->point.y=constrain(p.y,0,HEIGHT-1);data->state=LV_INDEV_STATE_PR;
    }
    bool validTransform()const{
        const auto& t=transform_;
        return std::isfinite(t.a)&&std::isfinite(t.b)&&std::isfinite(t.c)&&std::isfinite(t.d)&&std::isfinite(t.e)&&std::isfinite(t.f)&&
          fabs(t.a)<2&&fabs(t.b)<2&&fabs(t.d)<2&&fabs(t.e)<2&&fabs(t.c)<10000&&fabs(t.f)<10000&&fabs(t.a*t.e-t.b*t.d)>0.0001;
    }
    void rootStyle(lv_obj_t* o){lv_obj_set_style_bg_color(o,color(paper()),0);lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);lv_obj_set_style_text_color(o,color(ink()),0);lv_obj_set_style_text_font(o,&busa_font,0);}
    void startCalibration(){
        cancelKeyEdit(false);keyCount_=0;keyFocus_=-1;keyCursor_.reset();
        calibrating_=true;calibrationValid_=false;calIndex_=0;held_=false;awaitRelease_=true;calSamples_=0;sumX_=sumY_=0;
        lastCalibrationRead_=millis()-busa_touch::readPeriodMs;
        auto* root=lv_scr_act();hideKeyboard();calcNotes_=nullptr;touchModal_=nullptr;touchTitle_=nullptr;keyboard_=nullptr;touchEditor_=nullptr;touchTarget_=nullptr;lv_obj_clean(root);rootStyle(root);
        auto* msg=lv_label_create(root);lv_label_set_text(msg,"Калибровка: нажмите и отпустите метку");lv_obj_align(msg,LV_ALIGN_CENTER,0,0);
        calMark_=lv_obj_create(root);lv_obj_remove_style_all(calMark_);lv_obj_set_size(calMark_,18,18);
        lv_obj_set_style_radius(calMark_,LV_RADIUS_CIRCLE,0);lv_obj_set_style_bg_color(calMark_,color(0x2087ed),0);lv_obj_set_style_bg_opa(calMark_,LV_OPA_COVER,0);placeMark();
    }
    busa_touch::Point target()const{static const busa_touch::Point p[]={{20,20},{460,20},{20,252},{460,252}};return p[calIndex_];}
    void placeMark(){auto p=target();lv_obj_set_pos(calMark_,p.x-9,p.y-9);}
    void calibrationPoll(){
        if(!busa_touch::sampleDue(millis(),lastCalibrationRead_))return;
        TS_Point p;bool down=hw_.readRawTouch(p);
        if(awaitRelease_){if(!down)awaitRelease_=false;return;}
        if(down){
            if(!held_){held_=true;calSamples_=0;sumX_=sumY_=0;}
            if(calSamples_<32){sumX_+=p.x;sumY_+=p.y;++calSamples_;}
            return;
        }
        if(!held_)return;
        held_=false;if(calSamples_<3){calSamples_=0;return;}
        busa_touch::Point value{sumX_/calSamples_,sumY_/calSamples_};calSamples_=0;
        if(calIndex_<3){raw_[calIndex_]=value;screen_[calIndex_]=target();++calIndex_;placeMark();return;}
        if(!busa_touch::fit(raw_,screen_,transform_)||!validTransform()){startCalibration();return;}
        const auto fitted=busa_touch::apply(transform_,value),expected=target();
        if(abs(fitted.x-expected.x)>18||abs(fitted.y-expected.y)>18){startCalibration();return;}
        calibrationValid_=prefs_.putBytes("affineC0",&transform_,sizeof(transform_))==sizeof(transform_);
        if(!calibrationValid_){startCalibration();return;}
        calibrating_=false;build();
    }
    static void event(lv_event_t* e){
        auto* self=static_cast<Interface*>(lv_event_get_user_data(e));
        auto* o=lv_event_get_target(e);int id=int(reinterpret_cast<intptr_t>(lv_obj_get_user_data(o)));
        self->cancelKeyEdit(false);self->action(id);
    }
    static void textFocus(lv_event_t* e){
        auto* self=static_cast<Interface*>(lv_event_get_user_data(e));auto* ta=lv_event_get_target(e);
        if(self->waiting_&&self->page_==4&&self->calcMode_==3)return;
        self->cancelKeyEdit(false);self->hideKeyboard();self->focusObject(ta);
        lv_textarea_set_accepted_chars(self->touchEditor_,lv_textarea_get_accepted_chars(ta));
        lv_textarea_set_max_length(self->touchEditor_,lv_textarea_get_max_length(ta));
        lv_textarea_set_password_mode(self->touchEditor_,lv_textarea_get_password_mode(ta));
        lv_textarea_set_password_show_time(self->touchEditor_,0);
        lv_textarea_set_text(self->touchEditor_,lv_textarea_get_text(ta));
        self->touchTarget_=ta;
        auto* caption=lv_obj_get_child(lv_obj_get_parent(ta),0);
        lv_label_set_text(self->touchTitle_,lv_label_get_text(caption));
        lv_textarea_set_cursor_pos(self->touchEditor_,LV_TEXTAREA_CURSOR_LAST);
        lv_keyboard_set_textarea(self->keyboard_,self->touchEditor_);
        const bool numeric=lv_textarea_get_accepted_chars(ta)!=nullptr;
        lv_keyboard_set_mode(self->keyboard_,numeric?LV_KEYBOARD_MODE_NUMBER:LV_KEYBOARD_MODE_TEXT_LOWER);
        lv_obj_set_style_text_font(self->keyboard_,numeric?&busa_font_value:&busa_font,LV_PART_ITEMS);
        lv_obj_clear_flag(self->touchModal_,LV_OBJ_FLAG_HIDDEN);lv_obj_move_foreground(self->touchModal_);
    }
    static void keyboardEvent(lv_event_t* e){
        auto* self=static_cast<Interface*>(lv_event_get_user_data(e));
        if(!self->touchTarget_)return;
        const auto id=lv_btnmatrix_get_selected_btn(self->keyboard_);
        if(id!=LV_BTNMATRIX_BTN_NONE)busa_touch_keys::apply(self->keyboard_,self->touchEditor_,lv_btnmatrix_get_btn_text(self->keyboard_,id));
    }
    void touchFinish(bool accept){
        if(!touchTarget_)return;
        if(accept)lv_textarea_set_text(touchTarget_,lv_textarea_get_text(touchEditor_));
        hideKeyboard();
        if(accept&&page_==4)calculate(false);
    }
    void hideKeyboard(){
        if(!keyboard_)return;touchTarget_=nullptr;lv_keyboard_set_textarea(keyboard_,nullptr);
        lv_obj_add_flag(touchModal_,LV_OBJ_FLAG_HIDDEN);lv_textarea_set_text(touchEditor_,"");
    }
    lv_obj_t* label(lv_obj_t* parent,const char* text){
        auto* o=lv_label_create(parent);lv_obj_set_style_text_font(o,&busa_font,0);lv_obj_set_style_text_color(o,color(ink()),0);lv_label_set_text(o,text);return o;
    }
    lv_obj_t* smallLabel(lv_obj_t* parent,const char* text){
        auto* o=label(parent,text);lv_obj_set_style_text_font(o,&busa_font_small,0);lv_obj_set_style_text_letter_space(o,0,0);lv_obj_set_style_text_line_space(o,0,0);return o;
    }
    lv_obj_t* plain(lv_obj_t* parent,int w,int h){
        auto* o=lv_obj_create(parent);lv_obj_remove_style_all(o);lv_obj_set_size(o,w,h);
        lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE);return o;
    }
    lv_obj_t* at(lv_obj_t* parent,const char* text,int x,int y,const lv_font_t* font=&busa_font){
        auto* o=label(parent,text);lv_obj_set_style_text_font(o,font,0);lv_obj_set_pos(o,x,y);return o;
    }
    lv_obj_t* button(lv_obj_t* parent,const char* text,int id,int width=LV_PCT(100),bool keyboardFocus=true){
        auto* o=lv_btn_create(parent);lv_obj_set_size(o,width,26);lv_obj_set_style_radius(o,6,0);
        lv_obj_set_style_bg_color(o,color(surface()),0);lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);glass(o);
        lv_obj_set_style_shadow_width(o,0,0);lv_obj_set_style_border_width(o,1,0);
        lv_obj_set_style_border_color(o,color(line()),0);lv_obj_set_style_pad_all(o,1,0);
        lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE);
        auto* l=label(o,text);lv_obj_set_style_text_color(l,color(ink()),0);
        lv_obj_set_width(l,LV_PCT(100));lv_label_set_long_mode(l,LV_LABEL_LONG_DOT);
        if(width>0&&width<=WIDTH&&lv_txt_get_width(text,strlen(text),&busa_font,0,LV_TEXT_FLAG_NONE)>width-6)lv_obj_set_style_text_font(l,&busa_font_small,0);
        lv_obj_set_style_text_align(l,LV_TEXT_ALIGN_CENTER,0);lv_obj_center(l);
        lv_obj_set_user_data(o,reinterpret_cast<void*>(intptr_t(id)));lv_obj_add_event_cb(o,event,LV_EVENT_CLICKED,this);if(parent!=nav_&&keyboardFocus)registerKey(o,0,id);return o;
    }
    void setBg(lv_obj_t* o,uint32_t c){if(lv_obj_get_style_bg_color(o,0).full!=color(c).full)lv_obj_set_style_bg_color(o,color(c),0);}
    void setInk(lv_obj_t* o,uint32_t c){if(lv_obj_get_style_text_color(o,0).full!=color(c).full)lv_obj_set_style_text_color(o,color(c),0);}
    void selected(lv_obj_t* o,bool on){
        lv_obj_set_style_bg_color(o,color(on?(theme_==0?0x007bb3:theme_==1?0x535353:0x00a9e6):surface()),0);glass(o,on);
        lv_obj_set_style_border_color(o,color(on?accent():line()),0);
        lv_obj_set_style_text_color(lv_obj_get_child(o,0),color(on?0xffffff:ink()),0);
    }
    lv_obj_t* panel(lv_obj_t* parent,int width=LV_PCT(100),int height=LV_SIZE_CONTENT){
        auto* o=lv_obj_create(parent);lv_obj_set_size(o,width,height);lv_obj_set_style_bg_color(o,color(surface()),0);lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);glass(o);
        lv_obj_set_style_border_color(o,color(line()),0);lv_obj_set_style_border_width(o,1,0);
        lv_obj_set_style_radius(o,7,0);lv_obj_set_style_pad_all(o,3,0);lv_obj_set_style_pad_row(o,2,0);
        lv_obj_set_flex_flow(o,LV_FLEX_FLOW_COLUMN);lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE);return o;
    }
    lv_obj_t* row(lv_obj_t* parent,int height=LV_SIZE_CONTENT){
        auto* o=plain(parent,LV_PCT(100),height);
        lv_obj_set_flex_flow(o,LV_FLEX_FLOW_ROW);lv_obj_set_flex_align(o,LV_FLEX_ALIGN_SPACE_BETWEEN,LV_FLEX_ALIGN_CENTER,LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_column(o,3,0);return o;
    }
    lv_obj_t* field(lv_obj_t* parent,const char* title,const String& value,bool numeric,bool secret=false,int width=LV_PCT(100)){
        auto* box=plain(parent,width,42);lv_obj_set_flex_flow(box,LV_FLEX_FLOW_COLUMN);lv_obj_set_style_pad_row(box,1,0);
        auto* caption=smallLabel(box,title);lv_obj_set_style_text_color(caption,color(muted()),0);
        lv_obj_set_size(caption,LV_PCT(100),17);lv_label_set_long_mode(caption,LV_LABEL_LONG_CLIP);
        auto* ta=lv_textarea_create(box);lv_obj_set_width(ta,LV_PCT(100));
        lv_obj_set_style_text_font(ta,&busa_font,0);lv_obj_set_style_text_color(ta,color(ink()),0);lv_obj_set_style_bg_color(ta,color(paper()),0);lv_obj_set_style_bg_opa(ta,LV_OPA_COVER,0);
        lv_obj_set_style_border_color(ta,color(line()),0);lv_obj_set_style_border_width(ta,1,0);lv_obj_set_style_radius(ta,5,0);lv_obj_set_style_pad_all(ta,1,0);lv_textarea_set_one_line(ta,true);lv_obj_set_height(ta,24);lv_obj_set_scrollbar_mode(ta,LV_SCROLLBAR_MODE_OFF);
        lv_textarea_set_max_length(ta,numeric?16:220);
        if(numeric)lv_textarea_set_accepted_chars(ta,"0123456789.-,");
        lv_textarea_set_password_mode(ta,secret);lv_textarea_set_text(ta,value.c_str());
        lv_obj_add_event_cb(ta,textFocus,LV_EVENT_CLICKED,this);registerKey(ta,1,0);return ta;
    }
    void navIcon(lv_obj_t* parent,int i){
        auto& art=navArt_[i];art.header.always_zero=0;art.header.cf=LV_IMG_CF_TRUE_COLOR_ALPHA;
        art.header.w=BUSA_NAV_W;art.header.h=BUSA_NAV_H;art.data_size=BUSA_NAV_W*BUSA_NAV_H*3;art.data=BUSA_NAV_ICONS[i];
        auto* icon=lv_img_create(parent);lv_img_set_src(icon,&art);lv_img_set_antialias(icon,true);
        static_assert(BUSA_NAV_W==BUSA_NAV_H,"Navigation art must be square");
        lv_obj_set_size(icon,BUSA_NAV_W,BUSA_NAV_H);
        lv_obj_set_style_img_recolor(icon,color(theme_==0?0x007bb3:theme_==1?0xdddddd:0x91ecf4),0);
        lv_obj_set_style_img_recolor_opa(icon,LV_OPA_COVER,0);lv_obj_align(icon,LV_ALIGN_TOP_MID,0,0);
        lv_obj_clear_flag(icon,LV_OBJ_FLAG_CLICKABLE);
    }
    static String text(lv_obj_t* ta){String s=lv_textarea_get_text(ta);s.replace(',','.');return s;}
    static String value(JsonVariantConst v,int digits=1){
        if(v.isNull()||!v.is<double>())return "";
        return String(v.as<double>(),digits);
    }
    String measured(JsonVariantConst v,const char* unit,int digits=1)const{
        String s=session_.connected()?value(v,digits):String("");
        if(s.isEmpty())return String("—");s.replace('.',',');return s+unit;
    }
    void set(lv_obj_t* o,const String& s){if(o&&strcmp(lv_label_get_text(o),s.c_str()))lv_label_set_text(o,s.c_str());}
    void metric(lv_obj_t* o,const String& s){
        const auto* font=lv_txt_get_width(s.c_str(),s.length(),&busa_font,0,LV_TEXT_FLAG_NONE)>112?&busa_font_small:&busa_font;
        if(lv_obj_get_style_text_font(o,0)!=font)lv_obj_set_style_text_font(o,font,0);set(o,s);
    }
    void buildTouchModal(lv_obj_t* root){
        touchModal_=plain(root,WIDTH,HEIGHT-TOP);lv_obj_set_pos(touchModal_,0,TOP);
        lv_obj_set_style_bg_color(touchModal_,color(paper()),0);lv_obj_set_style_bg_opa(touchModal_,LV_OPA_COVER,0);
        lv_obj_add_flag(touchModal_,LV_OBJ_FLAG_CLICKABLE);
        touchTitle_=at(touchModal_,"Ввод значения",8,1);lv_obj_set_width(touchTitle_,WIDTH-16);
        lv_label_set_long_mode(touchTitle_,LV_LABEL_LONG_DOT);
        touchEditor_=lv_textarea_create(touchModal_);lv_obj_set_width(touchEditor_,WIDTH-12);lv_obj_set_pos(touchEditor_,6,22);
        lv_textarea_set_one_line(touchEditor_,true);lv_obj_set_height(touchEditor_,36);lv_obj_set_style_text_font(touchEditor_,&busa_font_value,0);
        lv_obj_set_style_bg_color(touchEditor_,color(surface()),0);lv_obj_set_style_bg_opa(touchEditor_,LV_OPA_COVER,0);
        lv_obj_set_style_text_color(touchEditor_,color(ink()),0);lv_obj_set_style_border_color(touchEditor_,color(line()),0);
        lv_obj_set_style_border_width(touchEditor_,1,0);lv_obj_set_style_pad_all(touchEditor_,2,0);lv_obj_set_scrollbar_mode(touchEditor_,LV_SCROLLBAR_MODE_OFF);
        keyboard_=lv_keyboard_create(touchModal_);lv_obj_set_size(keyboard_,WIDTH-8,140);lv_obj_align(keyboard_,LV_ALIGN_TOP_LEFT,4,62);
        busa_touch_keys::configure(keyboard_);
        lv_obj_set_style_bg_color(keyboard_,color(paper()),LV_PART_MAIN);lv_obj_set_style_bg_opa(keyboard_,LV_OPA_COVER,LV_PART_MAIN);
        lv_obj_set_style_border_width(keyboard_,0,LV_PART_MAIN);lv_obj_set_style_pad_all(keyboard_,2,LV_PART_MAIN);
        lv_obj_set_style_pad_row(keyboard_,3,LV_PART_MAIN);lv_obj_set_style_pad_column(keyboard_,3,LV_PART_MAIN);
        lv_obj_set_style_radius(keyboard_,5,LV_PART_ITEMS);
        lv_obj_set_style_bg_color(keyboard_,color(surface()),LV_PART_ITEMS);
        lv_obj_set_style_bg_color(keyboard_,color(theme_==0?0xd9eaf2:theme_==1?0x424242:0x17445a),LV_PART_ITEMS|LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(keyboard_,color(surface()),LV_PART_ITEMS|LV_STATE_CHECKED);
        lv_obj_set_style_bg_opa(keyboard_,LV_OPA_COVER,LV_PART_ITEMS);
        lv_obj_set_style_text_color(keyboard_,color(ink()),LV_PART_ITEMS);
        lv_obj_set_style_text_color(keyboard_,color(ink()),LV_PART_ITEMS|LV_STATE_PRESSED);
        lv_obj_set_style_text_color(keyboard_,color(ink()),LV_PART_ITEMS|LV_STATE_CHECKED);
        lv_obj_set_style_border_width(keyboard_,1,LV_PART_ITEMS);lv_obj_set_style_border_color(keyboard_,color(line()),LV_PART_ITEMS);
        lv_obj_add_event_cb(keyboard_,keyboardEvent,LV_EVENT_VALUE_CHANGED,this);
        auto* cancel=button(touchModal_,"Отмена",340,231,false);lv_obj_set_pos(cancel,6,205);lv_obj_set_height(cancel,32);
        auto* ready=button(touchModal_,"Готово",341,231,false);lv_obj_set_pos(ready,243,205);lv_obj_set_height(ready,32);selected(ready,true);
        lv_obj_add_flag(touchModal_,LV_OBJ_FLAG_HIDDEN);
    }
    void buildNavigation(lv_obj_t* root){
        nav_=plain(root,WIDTH-6,NAV-2);lv_obj_set_pos(nav_,3,HEIGHT-NAV);
        lv_obj_set_style_radius(nav_,9,0);lv_obj_set_style_border_width(nav_,1,0);lv_obj_set_style_border_color(nav_,color(line()),0);
        lv_obj_set_style_bg_color(nav_,color(surface()),0);lv_obj_set_style_bg_opa(nav_,LV_OPA_COVER,0);glass(nav_);
        lv_obj_set_style_pad_all(nav_,1,0);lv_obj_set_style_pad_column(nav_,4,0);lv_obj_set_flex_flow(nav_,LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(nav_,LV_FLEX_ALIGN_SPACE_BETWEEN,LV_FLEX_ALIGN_CENTER,LV_FLEX_ALIGN_CENTER);
        static const char* titles[]={"Главная","Температуры","Вода и отбор","Графики","Калькулятор","Настройки"};
        for(int i=0;i<6;++i){auto* b=button(nav_,titles[i],100+i,74);lv_obj_set_height(b,38);
            lv_obj_set_style_pad_top(b,0,0);lv_obj_set_style_pad_bottom(b,0,0);
            auto* l=lv_obj_get_child(b,0);lv_obj_set_style_text_font(l,&busa_font_nav,0);lv_obj_set_style_text_letter_space(l,0,0);lv_obj_set_width(l,70);lv_obj_align(l,LV_ALIGN_BOTTOM_MID,0,0);
            navIcon(b,i);navButtons_[i]=b;}
    }
    void build(){
        cancelKeyEdit(false);keyCount_=0;keyFocus_=-1;
        auto* root=lv_scr_act();calcNotes_=nullptr;touchModal_=nullptr;touchTitle_=nullptr;keyboard_=nullptr;touchEditor_=nullptr;touchTarget_=nullptr;lv_obj_clean(root);rootStyle(root);lv_obj_clear_flag(root,LV_OBJ_FLAG_SCROLLABLE);
        auto* top=plain(root,WIDTH,TOP);lv_obj_set_pos(top,0,0);
        at(top,"BUSA MONITOR",6,0);clock_=at(top,"Дата и время: —",6,16,&busa_font_nav);
        lv_obj_set_style_text_color(clock_,color(muted()),0);
        for(int i=0;i<3;++i){static const char* names[]={"Wi-Fi","Google","ESP32"};auto* p=plain(top,65,18);lv_obj_set_pos(p,205+i*68,4);
            lv_obj_set_style_radius(p,9,0);lv_obj_set_style_border_width(p,1,0);lv_obj_set_style_border_color(p,color(line()),0);
            statusDots_[i]=plain(p,5,5);lv_obj_set_style_radius(statusDots_[i],LV_RADIUS_CIRCLE,0);lv_obj_set_style_bg_opa(statusDots_[i],LV_OPA_COVER,0);lv_obj_set_pos(statusDots_[i],5,6);
            at(p,names[i],15,2,&busa_font_small);}
        connection_=at(top,"UART",423,7,&busa_font_small);lv_obj_set_width(connection_,54);lv_obj_set_style_text_align(connection_,LV_TEXT_ALIGN_RIGHT,0);lv_label_set_long_mode(connection_,LV_LABEL_LONG_DOT);
        body_=lv_obj_create(root);lv_obj_set_pos(body_,0,TOP);lv_obj_set_size(body_,WIDTH,HEIGHT-TOP-NAV);
        lv_obj_set_style_bg_color(body_,color(paper()),0);lv_obj_set_style_border_width(body_,0,0);lv_obj_set_style_radius(body_,0,0);
        lv_obj_set_style_pad_all(body_,PAD,0);lv_obj_set_style_pad_row(body_,GAP,0);lv_obj_set_flex_flow(body_,LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(body_,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_scrollbar_mode(body_,LV_SCROLLBAR_MODE_OFF);
        buildNavigation(root);
        buildTouchModal(root);
        render();
    }
    void render(){
        cancelKeyEdit(false);keyCount_=0;keyFocus_=-1;
        if(calcNotes_){lv_obj_del(calcNotes_);calcNotes_=nullptr;}
        hideKeyboard();lv_obj_clean(body_);++viewGeneration_;formValid_=session_.connected();for(auto& p:fields_)p=nullptr;for(auto& p:live_)p=nullptr;for(auto& p:motion_)p=nullptr;
        wifiList_=nullptr;chart_=nullptr;for(auto& s:series_)s=nullptr;for(auto& l:graphLegend_)l=nullptr;resultLabel_=nullptr;pageStatus_=nullptr;pageCounter_=nullptr;pageContent_="";pageOffset_=0;scanPending_=false;
        for(int i=0;i<6;++i)selected(navButtons_[i],i==page_);
        lv_obj_clear_flag(body_,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_scrollbar_mode(body_,LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_style_pad_row(body_,page_==0?GAP:2,0);
        switch(page_){case 0:home();break;case 1:temperatures();break;case 2:water();break;case 3:graphs();break;case 4:calculator();break;case 5:settings();break;}
        lv_obj_update_layout(body_);
        for(auto* fieldObj:fields_)if(fieldObj){lv_area_t a,b;lv_obj_get_coords(fieldObj,&a);lv_obj_get_coords(lv_obj_get_parent(fieldObj),&b);
            if(a.x1<b.x1||a.x2>b.x2||a.y1<b.y1||a.y2>b.y2)Serial.printf("[UI] field bounds page=%d sub=%d\n",page_,sub_);}
        orderKeys();
        refresh();
        if(page_==4&&calcMode_==3&&!geometryReadAttempted_&&!waiting_&&session_.connected()){
            geometryReadAttempted_=send(Route::geometryRead,{});
        }
    }
    void home(){
        auto* scene=plain(body_,LV_PCT(100),SCENE);
        auto* center=plain(scene,BUSA_ASSET_W,BUSA_ASSET_H);lv_obj_align(center,LV_ALIGN_CENTER,0,0);
        auto* im=lv_img_create(center);lv_img_set_src(im,&art_);lv_img_set_antialias(im,true);lv_obj_center(im);
        for(unsigned i=0;i<7;++i){auto* dot=plain(center,3,3);
            lv_obj_set_style_bg_color(dot,color(i<4?0x38bdf8:0xc5fffa),0);lv_obj_set_style_bg_opa(dot,LV_OPA_COVER,0);lv_obj_set_style_radius(dot,LV_RADIUS_CIRCLE,0);motion_[i]=dot;}
        static const int xs[]={10,10,318,318},ys[]={65,7,65,7};
        for(int i=0;i<4;++i){at(scene,i==3?"Авария":sensor(i),xs[i],ys[i]);
            live_[4+i]=at(scene,"—",xs[i],ys[i]+17,&busa_font_value);lv_obj_set_style_text_color(live_[4+i],color(colors_[i]),0);
            live_[12+i]=at(scene,"ВКЛ — · ВЫКЛ —",xs[i],ys[i]+42,&busa_font_small);lv_obj_set_style_text_color(live_[12+i],color(muted()),0);}
        auto* m=plain(body_,LV_PCT(100),METRICS);
        lv_obj_set_style_bg_color(m,color(surface()),0);lv_obj_set_style_bg_opa(m,LV_OPA_COVER,0);glass(m);
        lv_obj_set_style_border_width(m,1,0);lv_obj_set_style_border_color(m,color(line()),0);lv_obj_set_style_radius(m,6,0);
        static const char* headings[]={"Расход","Израсходовано","Отбор","Собрано"};
        for(int i=0;i<4;++i){auto* l=at(m,headings[i],5+i*118,0,&busa_font_small);lv_obj_set_style_text_color(l,color(muted()),0);
            lv_obj_set_width(l,112);lv_label_set_long_mode(l,LV_LABEL_LONG_DOT);
            live_[8+i]=at(m,"—",5+i*118,14,&busa_font);lv_obj_set_width(live_[8+i],112);lv_label_set_long_mode(live_[8+i],LV_LABEL_LONG_DOT);}
        auto* bar=row(body_,RELAYS);
        static const char* relayNames[]={"ТЭН откл.","Кран закрыт","Кран открыт","Авария"};
        for(int i=0;i<4;++i){auto* p=plain(bar,115,RELAYS);lv_obj_set_style_radius(p,6,0);
            lv_obj_set_style_bg_color(p,color(surface()),0);lv_obj_set_style_bg_opa(p,LV_OPA_COVER,0);
            lv_obj_set_style_border_color(p,color(line()),0);lv_obj_set_style_border_width(p,1,0);
            at(p,relayNames[i],5,0,&busa_font_small);live_[i]=at(p,"Нет связи",5,14,&busa_font_small);}
    }
    lv_obj_t* statusLabel(lv_obj_t* parent,int width=LV_PCT(100),int height=26){
        auto* l=smallLabel(parent,"");lv_obj_set_size(l,width,height);lv_label_set_long_mode(l,LV_LABEL_LONG_DOT);return l;
    }
    void heading(const char* text){auto* r=row(body_,18);label(r,text);pageStatus_=statusLabel(r,285,16);}
    void temperatures(){
        const auto& doc=session_.snapshot();heading("ТЕМПЕРАТУРЫ");
        static const int order[]={0,2,1,3};
        for(int lineNo=0;lineNo<2;++lineNo){auto* r=row(body_,70);
            for(int col=0;col<2;++col){int i=order[lineNo*2+col];auto* p=panel(r,235,70);lv_obj_set_style_pad_all(p,2,0);auto* head=row(p,20);
                label(head,i==3?"Авария":sensor(i));live_[i]=label(head,"—");lv_obj_set_style_text_font(live_[i],&busa_font,0);lv_obj_set_style_text_color(live_[i],color(colors_[i]),0);
                auto* fields=row(p,42);fields_[2*i]=field(fields,"ВКЛ, °C",value(doc["display_references"][i]["target"]),true,false,110);
                fields_[2*i+1]=field(fields,"ВЫКЛ, °C",value(doc["display_references"][i]["notice"]),true,false,110);}}
        selected(button(body_,"Сохранить пороги",200),true);
    }
    void water(){
        heading("ВОДА И ОТБОР");auto* r=row(body_,134);
        auto* q=panel(r,235,134);label(q,"Отбор жидкости");live_[3]=label(q,"—");live_[4]=label(q,"—");live_[5]=smallLabel(q,"—");live_[6]=smallLabel(q,"—");live_[7]=smallLabel(q,"—");
        lv_obj_set_style_text_font(live_[3],&busa_font_value,0);lv_obj_set_style_text_color(live_[3],color(colors_[0]),0);
        for(int i=3;i<=7;++i){lv_obj_set_width(live_[i],LV_PCT(100));lv_label_set_long_mode(live_[i],LV_LABEL_LONG_WRAP);}
        auto* p=panel(r,235,134);label(p,"Охлаждение");live_[0]=label(p,"—");live_[1]=label(p,"—");live_[2]=smallLabel(p,"—");
        lv_obj_set_style_text_font(live_[0],&busa_font_value,0);lv_obj_set_style_text_color(live_[0],color(colors_[0]),0);
        for(int i=0;i<=2;++i){lv_obj_set_width(live_[i],LV_PCT(100));lv_label_set_long_mode(live_[i],LV_LABEL_LONG_WRAP);}
        const auto& d=session_.snapshot();fields_[0]=field(p,"Импульсов на 1 литр",value(d["water"]["pulses_per_litre"]),true);
        selected(button(body_,"Сохранить коэффициент",201),true);
    }
    void graphAxes(double high){
        for(int i=0;i<4;++i){
            char b[32];snprintf(b,sizeof(b),"%.3g",high*(3-i)/3.0);lv_label_set_text(graphY_[i],b);
        }
    }
    void graphTimeLabels(){
        const char* clock=session_.snapshot()["clock"]["datetime"]|"";
        int d,m,y,h,min;
        const bool ok=session_.connected()&&sscanf(clock,"%d.%d.%d %d:%d",&d,&m,&y,&h,&min)==5&&h>=0&&h<24&&min>=0&&min<60;
        for(int i=0;i<4;++i){
            char b[16];
            if(ok){int t=(h*60+min-(3-i)*graphMinutes_/3+1440)%1440;snprintf(b,sizeof(b),"%02d:%02d",t/60,t%60);}
            else snprintf(b,sizeof(b),"-%d мин",(3-i)*graphMinutes_/3);
            lv_label_set_text(graphX_[i],b);
        }
    }
    void clearGraph(){
        if(!chart_)return;
        for(unsigned i=0;i<busa_graph::spec(graphKind_).series;++i)lv_chart_set_all_value(chart_,series_[i],LV_CHART_POINT_NONE);
        graphAxes(graphKind_==0?125:10);lv_chart_refresh(chart_);
    }
    void graphs(){
        auto* r=row(body_,26);
        static const char* names[]={"Температура","Отбор","Охлаждение"};
        for(int i=0;i<3;++i)selected(button(r,names[i],300+i,156),graphKind_==i);
        const auto spec=busa_graph::spec(graphKind_);
        auto* head=row(body_,17);smallLabel(head,spec.title);pageStatus_=statusLabel(head,270,17);set(pageStatus_,"Ожидание данных");
        if(graphKind_==0){
            auto* legend=row(body_,17);static const int order[]={0,2,1,3};
            for(int i:order){auto* l=smallLabel(legend,i==3?"Авария":sensor(i));graphLegend_[i]=l;lv_obj_set_style_text_color(l,color(colors_[i]),0);}
        }
        const int plotHeight=graphKind_==0?84:103;
        auto* plot=row(body_,plotHeight);
        auto* y=plain(plot,48,plotHeight);lv_obj_set_flex_flow(y,LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(y,LV_FLEX_ALIGN_SPACE_BETWEEN,LV_FLEX_ALIGN_END,LV_FLEX_ALIGN_CENTER);
        for(int i=0;i<4;++i){graphY_[i]=smallLabel(y,"0");lv_obj_set_style_text_font(graphY_[i],&busa_font_nav,0);}
        chart_=lv_chart_create(plot);lv_obj_set_size(chart_,421,plotHeight);lv_chart_set_type(chart_,LV_CHART_TYPE_LINE);
        lv_obj_set_style_pad_all(chart_,0,0);
        lv_chart_set_point_count(chart_,160);lv_obj_set_style_bg_color(chart_,color(surface()),0);lv_obj_set_style_border_color(chart_,color(line()),0);
        lv_obj_set_style_line_color(chart_,color(line()),LV_PART_MAIN);lv_obj_set_style_line_width(chart_,2,LV_PART_ITEMS);
        lv_obj_set_style_size(chart_,2,LV_PART_INDICATOR);lv_chart_set_div_line_count(chart_,4,4);
        lv_chart_set_range(chart_,LV_CHART_AXIS_PRIMARY_Y,0,1000);
        for(unsigned i=0;i<spec.series;++i)series_[i]=lv_chart_add_series(chart_,color(graphKind_==0?colors_[i]:spec.color),LV_CHART_AXIS_PRIMARY_Y);
        clearGraph();
        auto* times=row(body_,15);plain(times,44,1);
        for(int i=0;i<4;++i){graphX_[i]=smallLabel(times,"—");lv_obj_set_style_text_font(graphX_[i],&busa_font_nav,0);}
        auto* ranges=row(body_,26);
        for(int i=0;i<5;++i){static const char* names[]={"5 мин","10 мин","15 мин","30 мин","1 ч"};static const int mins[]={5,10,15,30,60};
            graphRange_[i]=button(ranges,names[i],310+i,91);selected(graphRange_[i],graphMinutes_==mins[i]);}
        graphDirty_=true;requestGraph();
    }
    void requestGraph(){
        if(waiting_||!session_.connected())return;
        graphDirty_=false;lastGraph_=millis();send(Route::chartHistory,{{"minutes",String(graphMinutes_)}});
    }
    static const char* calcTitle(int mode){static const char* names[]={"Разбавление водой","Коррекция ареометра","Вторая перегонка","Объём куба"};return names[mode];}
    void rememberCalc(){if(page_!=4)return;for(int i=0;i<5;++i)if(fields_[i])calcInput_[calcMode_][i]=text(fields_[i]);}
    bool geometryValid()const{
        const String& a=calcInput_[3][0];const String& b=calcInput_[3][1];char *ae=nullptr,*be=nullptr;
        double d=strtod(a.c_str(),&ae),h=strtod(b.c_str(),&be);
        return a.length()&&b.length()&&ae&&be&&!*ae&&!*be&&BusaCalc::cylinder(d,h).ok;
    }
    String geometrySummary()const{
        if(!geometryValid())return "Введите внутренние размеры куба";
        String d=calcInput_[3][0],h=calcInput_[3][1];d.replace('.',',');h.replace('.',',');
        return "Размеры куба: "+d+" × "+h+" см · изменить";
    }
    void clearCalc(){
        for(int i=0;i<5;++i)if(fields_[i]&&(calcMode_!=3||i==2)){calcInput_[calcMode_][i]="";lv_textarea_set_text(fields_[i],"");}
        if(calcMode_==3)calculate(false);else set(resultLabel_,"—");if(pageStatus_!=resultLabel_)set(pageStatus_,"");
    }
    void calcConditions(){
        if(calcNotes_){lv_obj_del(calcNotes_);calcNotes_=nullptr;return;}
        static const char* notes[]={"Все объёмы и крепость — при 20 °C. Учтено сжатие смеси.","Для водно-спиртового раствора без сахара, 0…40 °C. Стеклянный спиртомер, калибровка 20 °C; номинальное расширение стекла. Оценка, не поверка прибора.","Доли голов и хвостов — от абсолютного спирта (АС). Литры АС не равны фактическим объёмам фракций. Выход — расчётная оценка без дополнительных потерь.","Вертикальный цилиндр с плоским дном. Все размеры внутренние; уровень измеряется от дна. Диаметр и высота сохраняются до их изменения."};
        calcNotes_=plain(lv_scr_act(),WIDTH-8,HEIGHT-TOP-NAV-8);lv_obj_set_pos(calcNotes_,4,TOP+4);
        lv_obj_set_style_bg_color(calcNotes_,color(paper()),0);lv_obj_set_style_bg_opa(calcNotes_,LV_OPA_COVER,0);
        lv_obj_set_style_border_width(calcNotes_,1,0);lv_obj_set_style_border_color(calcNotes_,color(line()),0);
        lv_obj_add_flag(calcNotes_,LV_OBJ_FLAG_CLICKABLE);
        at(calcNotes_,"Условия расчёта",8,8);
        auto* l=at(calcNotes_,notes[calcMode_],8,35);lv_obj_set_width(l,WIDTH-30);lv_label_set_long_mode(l,LV_LABEL_LONG_WRAP);
        auto* close=button(calcNotes_,"Закрыть",334,120,false);lv_obj_align(close,LV_ALIGN_BOTTOM_MID,0,-8);
    }
    void calculator(){
        static const char* labels[4][5]={{"Объём спирта, л","Начальная крепость, %","Требуемая крепость, %","",""},{"Температура спирта, °C","Показание ареометра, %","","",""},{"Объём спирта-сырца, л","Крепость спирта-сырца, %","Крепость на выходе, %","Доля голов от АС, %","Доля хвостов от АС, %"},{"Внутренний диаметр, см","Внутренняя высота, см","Уровень жидкости, см","",""}};
        static const int counts[]={3,2,5,3};
        lv_obj_set_style_pad_row(body_,calcMode_<2?6:2,0);
        auto* tabs=plain(body_,LV_PCT(100),42);lv_obj_set_flex_flow(tabs,LV_FLEX_FLOW_COLUMN);lv_obj_set_style_pad_row(tabs,2,0);
        for(int n=0;n<2;++n){auto* r=row(tabs,20);for(int col=0;col<2;++col){int i=n*2+col;
            auto* b=button(r,calcTitle(i),320+i,234);lv_obj_set_height(b,20);
            lv_obj_set_style_pad_top(b,0,0);lv_obj_set_style_pad_bottom(b,0,0);
            auto* l=lv_obj_get_child(b,0);lv_obj_set_style_text_font(l,&busa_font_small,0);lv_label_set_long_mode(l,LV_LABEL_LONG_WRAP);selected(b,i==calcMode_);}}
        if(calcMode_==3){
            if(geometryExpanded_){
                auto* controls=row(body_,26);button(controls,"Сохранить и свернуть",332,235);button(controls,"Загрузить размеры",331,235);
                auto* inputs=row(body_,42);for(int i=0;i<3;++i){fields_[i]=field(inputs,labels[3][i],calcInput_[3][i],true,false,153);lv_textarea_set_placeholder_text(fields_[i],"0,00");}
            }else{
                button(body_,geometrySummary().c_str(),335);
                fields_[2]=field(body_,labels[3][2],calcInput_[3][2],true);lv_textarea_set_placeholder_text(fields_[2],"0,00");
            }
        }else{
            for(int i=0;i<counts[calcMode_];){auto* inputs=row(body_,42);const int n=calcMode_==2&&i==0?2:counts[calcMode_]-i;for(int col=0;col<n;++col,++i){fields_[i]=field(inputs,labels[calcMode_][i],calcInput_[calcMode_][i],true,false,n==2?235:153);lv_textarea_set_placeholder_text(fields_[i],"0,00");}}
        }
        auto* buttons=row(body_,22);
        selected(button(buttons,"Рассчитать",330,153),true);button(buttons,calcMode_==3?"Очистить уровень":"Очистить",333,153);button(buttons,"Условия расчёта",334,153);
        for(int i=0;i<3;++i){auto* b=lv_obj_get_child(buttons,i);lv_obj_set_height(b,22);
            lv_obj_set_style_pad_top(b,0,0);lv_obj_set_style_pad_bottom(b,0,0);lv_obj_set_style_text_font(lv_obj_get_child(b,0),&busa_font_small,0);}
        auto* p=panel(body_,LV_PCT(100),calcMode_==2?36:50);lv_obj_set_style_pad_all(p,calcMode_==2?0:2,0);resultLabel_=smallLabel(p,"—");lv_obj_set_width(resultLabel_,LV_PCT(100));lv_label_set_long_mode(resultLabel_,LV_LABEL_LONG_WRAP);
        pageStatus_=resultLabel_;
        calculate(false);
    }
    void pageText(){
        if(!resultLabel_||!pageCounter_)return;
        const char* data=pageContent_.c_str();const uint32_t size=pageContent_.length();
        // Use LVGL's own line breaking so the fixed-height page fits its actual font.
        uint32_t starts[256]={0};unsigned pages=1;uint32_t pos=0;unsigned line=0;
        while(pos<size&&pages<256){const uint32_t bytes=_lv_txt_get_next_line(data+pos,&busa_font_small,0,474,nullptr,LV_TEXT_FLAG_NONE);
            if(!bytes)break;pos+=bytes;
            if(++line==pageLines_&&pos<size){starts[pages++]=pos;line=0;}
        }
        if(pageOffset_>=pages)pageOffset_=pages-1;
        const uint32_t end=pageOffset_+1<pages?starts[pageOffset_+1]:size;
        String shown=pageContent_.substring(starts[pageOffset_],end);
        while(shown.endsWith("\n")||shown.endsWith("\r"))shown.remove(shown.length()-1);
        set(resultLabel_,shown);
        set(pageCounter_,String(pageOffset_+1)+" / "+String(pages));
    }
    void pagedResult(int height=80,unsigned lines=5){
        pageLines_=lines;resultLabel_=smallLabel(body_,"—");lv_obj_set_size(resultLabel_,LV_PCT(100),height);
        lv_label_set_long_mode(resultLabel_,LV_LABEL_LONG_WRAP);lv_obj_set_style_text_letter_space(resultLabel_,0,0);lv_obj_set_style_text_line_space(resultLabel_,0,0);
        auto* r=row(body_,26);button(r,"Предыдущая",410,152);pageCounter_=smallLabel(r,"1 / 1");button(r,"Следующая",411,152);
    }
    void settings(){
        if(sub_<0){label(body_,"НАСТРОЙКИ");static const char* names[]={"Температура","Wi-Fi","Экран","Google","Bluetooth · RainMaker","Обновление ПО","Журнал событий","Диагностика","О системе"};
            static const int ids[]={400,401,402,403,407,408,405,406,404};
            for(int i=0;i<9;){auto* r=row(body_,27);for(int col=0;col<2&&i<9;++col,++i)button(r,names[i],ids[i],235);}return;}
        static const char* title[]={"Температура","Wi-Fi","Экран","Google","О системе","Журнал событий","Диагностика","Bluetooth · RainMaker","Обновление ПО"};
        auto* top=row(body_,26);label(top,title[sub_]);button(top,"Все настройки",409,132);
        const auto& d=session_.snapshot();
        if(sub_==0){static const int order[]={0,2,1,3};for(int i=0;i<4;){auto* r=row(body_,42);for(int col=0;col<2;++col,++i){int k=order[i];fields_[k]=field(r,k==3?"Авария":sensor(k),value(d["calibration_offsets"][k]),true,false,235);}}button(body_,"Сохранить поправки",202);}
        else if(sub_==1){auto* r=row(body_,26);live_[0]=smallLabel(r,"Wi-Fi —");lv_obj_set_width(live_[0],280);lv_label_set_long_mode(live_[0],LV_LABEL_LONG_DOT);button(r,"Обновить список",203,180);
            wifiList_=lv_dropdown_create(body_);lv_obj_set_size(wifiList_,LV_PCT(100),26);lv_obj_set_style_pad_ver(wifiList_,3,LV_PART_MAIN);lv_obj_set_style_text_font(wifiList_,&busa_font,LV_PART_MAIN);lv_obj_set_style_bg_color(wifiList_,color(surface()),LV_PART_MAIN);lv_obj_set_style_text_color(wifiList_,color(ink()),LV_PART_MAIN);lv_obj_set_style_border_color(wifiList_,color(line()),LV_PART_MAIN);lv_obj_set_style_text_font(wifiList_,LV_FONT_DEFAULT,LV_PART_INDICATOR);lv_dropdown_set_options(wifiList_,"Нет сетей");registerKey(wifiList_,2,0);
            lv_obj_add_event_cb(wifiList_,dropdownOpened,LV_EVENT_READY,this);fields_[0]=field(body_,"Пароль","",false,true);lv_textarea_set_max_length(fields_[0],63);
            auto* b=row(body_,26);button(b,"Подключить",204,235);button(b,"Отключить",205,235);}
        else if(sub_==2){auto* r=row(body_,26);selected(button(r,"Светлая",206,235),requestedTheme_==0);selected(button(r,"Тёмная",207,235),requestedTheme_==1);selected(button(body_,"Системная · утверждённые рисунки БУСА",208),requestedTheme_==2);selected(button(body_,"Применить и сохранить тему",221),true);button(body_,"Калибровка сенсора",209);}
        else if(sub_==3){live_[0]=smallLabel(body_,"Google —");auto* r=row(body_,42);fields_[0]=field(r,"Адрес Apps Script (/exec)","",false,false,235);fields_[1]=field(r,"Секрет устройства","",false,true,235);
            button(body_,"Сохранить подключение Google",210);auto* b=row(body_,26);button(b,"Проверить отправку диагностики",211,235);button(b,"Отключить передачу в Google",212,235);}
        else if(sub_==4){for(int i=0;i<5;++i){live_[i]=smallLabel(body_,"—");lv_obj_set_size(live_[i],LV_PCT(100),i==0?60:16);lv_label_set_long_mode(live_[i],LV_LABEL_LONG_WRAP);}}
        else if(sub_==5){button(body_,"Обновить журнал",213);pagedResult();}
        else if(sub_==6){for(int i=0;i<10;){auto* r=row(body_,16);for(int col=0;col<2;++col,++i){live_[i]=smallLabel(r,"—");lv_obj_set_width(live_[i],235);lv_label_set_long_mode(live_[i],LV_LABEL_LONG_DOT);}}
            auto* r=row(body_,26);button(r,"Начать обучение 20 клавиш",217,235);button(r,"Считать и далее",218,235);auto* b=row(body_,26);button(b,"Назад на шаг",219,235);button(b,"Отмена",220,235);}
        else if(sub_==7){auto* r=row(body_,26);button(r,"Включить сопряжение",214,235);button(r,"Отключить Bluetooth",215,235);button(body_,"Обновить состояние",216);pagedResult(60,4);}
        else if(sub_==8){auto* l=label(body_,"Обновление через локальное приложение");lv_obj_set_width(l,LV_PCT(100));lv_label_set_long_mode(l,LV_LABEL_LONG_WRAP);}
        pageStatus_=statusLabel(body_,LV_PCT(100),sub_==6||sub_==7?14:24);
        if(sub_==5)send(Route::events,{{"after","0"}});
    }
    static void dropdownOpened(lv_event_t* e){auto* list=lv_dropdown_get_list(lv_event_get_target(e));if(list){lv_obj_set_style_text_font(list,&busa_font,LV_PART_MAIN);lv_obj_set_style_text_font(list,&busa_font,LV_PART_SELECTED);}}
    bool send(Route route,std::initializer_list<Arg> args){
        uint8_t bytes[apiMaxRequest]={};ApiBuilder b(bytes,sizeof(bytes),route);
        for(const auto& a:args)if(!b.add(a.key.c_str(),a.value.c_str())){set(pageStatus_,"Проверьте поля");return false;}
        return submit(route,b,bytes);
    }
    bool submit(Route route,const ApiBuilder& b,const uint8_t* bytes){
        if(waiting_||!session_.submitApi(bytes,b.size())){set(pageStatus_,"Нет связи или выполняется запрос");return false;}
        waiting_=true;pendingRoute_=route;pendingPage_=page_;pendingSub_=sub_;pendingGeneration_=viewGeneration_;set(pageStatus_,"Выполняется…");return true;
    }
    void form(Route route,const char* key,int n){
        uint8_t bytes[apiMaxRequest]={};ApiBuilder b(bytes,sizeof(bytes),route);
        if(route==Route::calibration&&!b.add("confirm","YES"))return;
        for(int i=0;i<n;++i){String k=String(key)+i;if(!b.add(k.c_str(),text(fields_[i]).c_str())){set(pageStatus_,"Проверьте поля");return;}}
        submit(route,b,bytes);
    }
    void alarms(){
        uint8_t bytes[apiMaxRequest]={};ApiBuilder b(bytes,sizeof(bytes),Route::temperatureAlarms);
        for(int i=0;i<4;++i){
            if(!b.add((String("on")+i).c_str(),text(fields_[2*i]).c_str())||!b.add((String("off")+i).c_str(),text(fields_[2*i+1]).c_str())){set(pageStatus_,"Проверьте поля");return;}
        }
        submit(Route::temperatureAlarms,b,bytes);
    }
    void action(int id){
        if(id==340||id==341){touchFinish(id==341);return;}
        if(touchTarget_)return;
        if(waiting_&&page_==4&&calcMode_==3)return;
        if(id>=100&&id<106){rememberCalc();if(page_!=id-100)previousPage_=page_;page_=id-100;sub_=-1;render();return;}
        if(id>=400&&id<409){sub_=id-400;if(sub_==2)requestedTheme_=theme_;render();return;}
        if(id==409){sub_=-1;render();return;}
        if(id==410||id==411){if(id==410){if(pageOffset_)--pageOffset_;}else if(pageOffset_<255)++pageOffset_;pageText();return;}
        if(id>=300&&id<=302){graphKind_=id-300;render();return;}
        if(id>=310&&id<=314){static const int minutes[]={5,10,15,30,60};graphMinutes_=minutes[id-310];render();return;}
        if(id>=320&&id<=323){rememberCalc();calcMode_=id-320;render();return;}
        hideKeyboard();
        if((id==200||id==201||id==202)&&(!formValid_||!session_.connected())){set(pageStatus_,"Нет актуальных настроек. Дождитесь связи.");return;}
        switch(id){
            case 200:alarms();break;
            case 201:send(Route::waterCoefficient,{{"confirm","YES"},{"pulses_per_litre",text(fields_[0])}});break;
            case 202:form(Route::calibration,"offset",4);break;
            case 203:send(Route::wifiScan,{{"start","1"},{"fresh","1"}});break;
            case 204:{unsigned i=lv_dropdown_get_selected(wifiList_);if(i<networkCount_)send(Route::wifiConnect,{{"ssid",networks_[i]},{"pass",lv_textarea_get_text(fields_[0])}});else set(pageStatus_,"Выберите сеть");break;}
            case 205:send(Route::wifiDisconnect,{});break;
            case 206:case 207:case 208:requestedTheme_=id-206;render();break;
            case 221:themeSynced_=false;send(Route::theme,{{"confirm","YES"},{"mode",String(requestedTheme_)}});break;
            case 209:startCalibration();break;
            case 210:send(Route::googleConfig,{{"url",lv_textarea_get_text(fields_[0])},{"token",lv_textarea_get_text(fields_[1])}});break;
            case 211:send(Route::googleTest,{{"confirm","YES"}});break;
            case 212:send(Route::googleOff,{{"confirm","YES"}});break;
            case 213:send(Route::events,{{"after","0"}});break;
            case 214:send(Route::rainmakerEnable,{{"confirm","YES"}});break;
            case 215:send(Route::rainmakerDisable,{{"confirm","YES"}});break;
            case 216:send(Route::rainmakerStatus,{});break;
            case 217:send(Route::keypadStart,{{"confirm","YES"}});break;
            case 218:send(Route::keypadNext,{{"confirm","YES"}});break;
            case 219:send(Route::keypadUndo,{{"confirm","YES"}});break;
            case 220:send(Route::keypadCancel,{{"confirm","YES"}});break;
            case 330:calculate();if(calcMode_==3&&geometryExpanded_&&geometryValid())action(332);break;
            case 333:clearCalc();break;
            case 334:calcConditions();break;
            case 331:if(calcMode_==3){rememberCalc();geometryReadAttempted_=send(Route::geometryRead,{});}break;
            case 332:if(calcMode_==3&&geometryExpanded_){rememberCalc();if(!geometryValid()){set(resultLabel_,"Проверьте размеры");break;}send(Route::geometrySave,{{"confirm","YES"},{"diameter",calcInput_[3][0]},{"height",calcInput_[3][1]}});}break;
            case 335:if(calcMode_==3){rememberCalc();geometryExpanded_=true;render();}break;
        }
    }
    static String fmt(double v){String s(v,2);s.replace('.',',');return s;}
    void calculate(bool explicitRequest=true){
        static const int counts[]={3,2,5,3};double v[5]={};
        for(int i=0;i<counts[calcMode_];++i){
            if(fields_[i])calcInput_[calcMode_][i]=text(fields_[i]);
            String s=calcInput_[calcMode_][i];char* e=nullptr;v[i]=strtod(s.c_str(),&e);
            if(calcMode_==3&&i==2&&s.isEmpty())continue;
            if(s.isEmpty()||*e||!std::isfinite(v[i])){set(resultLabel_,explicitRequest?"Проверьте значения":"—");return;}
        }
        BusaCalc::Result r;
        if(calcMode_==0)r=BusaCalc::dilute(v[0],v[1],v[2]);
        else if(calcMode_==1)r=BusaCalc::correct(v[0],v[1]);
        else if(calcMode_==2)r=BusaCalc::fraction(v[0],v[1],v[2],v[3],v[4]);
        else r=calcInput_[3][2].isEmpty()?BusaCalc::cylinder(v[0],v[1]):BusaCalc::cylinder(v[0],v[1],v[2]);
        if(!r.ok){set(resultLabel_,"Недопустимые значения");return;}
        if(calcMode_==0)set(resultLabel_,"Необходимо долить воды: "+fmt(r.water)+" л\nИтоговый объём: "+fmt(r.total)+" л");
        else if(calcMode_==1)set(resultLabel_,"Крепость при 20 °C: "+fmt(r.strength)+" % об.");
        else if(calcMode_==2)set(resultLabel_,"Абсолютный спирт: "+fmt(r.absolute)+" л АС"+"   "+"Головы: "+fmt(r.heads)+" л АС"+"\n"+"Хвосты: "+fmt(r.tails)+" л АС"+"   "+"Выход продукта: "+fmt(r.product)+" л");
        else set(resultLabel_,"Полный объём: "+fmt(r.capacity)+" л"+(r.levelKnown?String("\nОбъём жидкости: ")+fmt(r.liquid)+" л":""));
    }
    void handleResult(int code,const String& body){
        if(calibrating_||pendingPage_!=page_||pendingSub_!=sub_||pendingGeneration_!=viewGeneration_)return;
        if(code<200||code>=300){if(pendingRoute_==Route::chartHistory)clearGraph();set(pageStatus_,code==0?String("Ответ не получен; проверьте состояние"):String("Ошибка ")+code+": "+body.substring(0,100));return;}
        set(pageStatus_,"Выполнено");
        if(pendingRoute_==Route::geometryRead&&calcMode_==3){JsonDocument d;if(deserializeJson(d,body)){set(resultLabel_,"Ошибка загрузки размеров");return;}
            rememberCalc();for(int i=0;i<2;++i)calcInput_[3][i]=d[i?"height":"diameter"]|"";
            geometryExpanded_=!geometryValid();render();return;}
        if(pendingRoute_==Route::geometrySave&&calcMode_==3){geometryExpanded_=false;render();return;}
        if(pendingRoute_>=Route::keypadStart&&pendingRoute_<=Route::keypadCancel){set(pageStatus_,body);return;}
        if(pendingRoute_==Route::referenceEdit||pendingRoute_==Route::calibrationEdit){set(pageStatus_,body);return;}
        if(pendingRoute_==Route::theme){themeSynced_=true;theme_=requestedTheme_;prefs_.putUChar("theme",theme_);build();return;}
        if(pendingRoute_==Route::wifiConnect||pendingRoute_==Route::googleConfig){if(fields_[0]&&pendingRoute_==Route::wifiConnect)lv_textarea_set_text(fields_[0],"");if(fields_[1]&&pendingRoute_==Route::googleConfig)lv_textarea_set_text(fields_[1],"");}
        if(pendingRoute_==Route::wifiScan){JsonDocument d;if(deserializeJson(d,body)){set(pageStatus_,"Ошибка списка сетей");return;}scanPending_=d["scanning"]|false;
            if(scanPending_){set(pageStatus_,"Поиск сетей…");return;}
            networkCount_=0;String options;
            for(JsonObjectConst n:d["networks"].as<JsonArrayConst>()){if(networkCount_>=32)break;const char* ssid=n["ssid"]|"";if(!*ssid)continue;networks_[networkCount_++]=ssid;String display=ssid;display.replace('\n',' ');display.replace('\r',' ');if(options.length())options+="\n";options+=display+" ("+String(n["rssi"]|0)+")";}
            lv_dropdown_set_options(wifiList_,options.length()?options.c_str():"Нет сетей");set(pageStatus_,d["error"]|"");return;}
        if(pendingRoute_==Route::chartHistory&&chart_){
            JsonDocument d;busa_graph::Scale scale;
            if(deserializeJson(d,body)||!busa_graph::analyze(d.as<JsonObjectConst>(),graphKind_,graphMinutes_,scale)){
                clearGraph();set(pageStatus_,"Ошибка данных графика");return;
            }
            lv_chart_set_point_count(chart_,scale.count);
            for(unsigned k=0;k<busa_graph::spec(graphKind_).series;++k){
                auto arr=busa_graph::samples(d.as<JsonObjectConst>(),graphKind_,k);
                for(unsigned i=0;i<scale.count;++i){
                    JsonVariantConst v=arr[i];
                    lv_chart_set_value_by_id(chart_,series_[k],i,busa_graph::valid(v)?lv_coord_t(busa_graph::coordinate(v.as<double>(),scale.high)):LV_CHART_POINT_NONE);
                }
            }
            graphAxes(scale.high);lv_chart_refresh(chart_);set(pageStatus_,busa_graph::state(d.as<JsonObjectConst>(),graphKind_,scale));return;}
        if(pendingRoute_==Route::events){JsonDocument d;if(deserializeJson(d,body)){set(pageStatus_,"Ошибка журнала");return;}String s;
            for(JsonObjectConst e:d["events"].as<JsonArrayConst>()){s+=String((e["uptime_ms"]|0u)/1000)+" с  "+String(e["text"]|"")+"\n";if(s.length()>7000)break;}pageContent_=s.length()?s:"Нет событий";pageOffset_=0;pageText();}
        else if(pendingRoute_==Route::rainmakerStatus){JsonDocument d;if(!deserializeJson(d,body)){pageContent_=String("BLE: ")+(d["ble"]|"—")+"\n"+(d["feedback"]|"");pageOffset_=0;pageText();}}
    }
    struct KeyControl {lv_obj_t* obj;uint8_t type;int id;};
    void registerKey(lv_obj_t* obj,uint8_t type,int id){
        if(keyCount_>=int(sizeof(keyControls_)/sizeof(keyControls_[0])))return;
        keyControls_[keyCount_++]={obj,type,id};
        lv_obj_set_style_outline_color(obj,color(accent()),LV_STATE_FOCUSED);
        lv_obj_set_style_outline_width(obj,2,LV_STATE_FOCUSED);
        lv_obj_set_style_outline_pad(obj,0,LV_STATE_FOCUSED);
    }
    void orderKeys(){
        busa_focus::Rect bounds[64];unsigned order[64];KeyControl sorted[64];
        for(int i=0;i<keyCount_;++i){lv_area_t a;lv_obj_get_coords(keyControls_[i].obj,&a);bounds[i]={a.x1,a.y1,a.x2-a.x1+1,a.y2-a.y1+1};}
        busa_focus::orderControls(bounds,keyCount_,order);for(int i=0;i<keyCount_;++i)sorted[i]=keyControls_[order[i]];
        for(int i=0;i<keyCount_;++i)keyControls_[i]=sorted[i];
    }
    void focusKey(int index){
        if(!keyCount_)return;
        if(keyFocus_>=0&&keyFocus_<keyCount_)lv_obj_clear_state(keyControls_[keyFocus_].obj,LV_STATE_FOCUSED);
        keyFocus_=(index+keyCount_)%keyCount_;
        auto* obj=keyControls_[keyFocus_].obj;lv_obj_add_state(obj,LV_STATE_FOCUSED);
    }
    void focusObject(lv_obj_t* obj){for(int i=0;i<keyCount_;++i)if(keyControls_[i].obj==obj){focusKey(i);return;}}
    int fieldIndex(lv_obj_t* obj)const{for(int i=0;i<8;++i)if(fields_[i]==obj)return i;return -1;}
    void keyHint(const String& s){set(pageStatus_?pageStatus_:resultLabel_,s);}
    void cancelKeyEdit(bool restore){
        if(keyEdit_&&restore){if(keyDropdown_)lv_dropdown_set_selected(keyEdit_,keyOriginalChoice_);else lv_textarea_set_text(keyEdit_,keyOriginal_.c_str());}
        keyEdit_=nullptr;keyDropdown_=false;keyOriginal_="";keyConfirm_=-1;lastTapKey_=-1;lastTapAt_=0;
    }
    void beginKeyEdit(const KeyControl& c){
        hideKeyboard();keyEdit_=c.obj;keyDropdown_=c.type==2;lastTapKey_=-1;tapMode_=0;
        if(keyDropdown_){keyOriginalChoice_=lv_dropdown_get_selected(keyEdit_);return;}
        keyOriginal_=lv_textarea_get_text(keyEdit_);lv_textarea_set_cursor_pos(keyEdit_,LV_TEXTAREA_CURSOR_LAST);
    }
    static int keyDigit(uint8_t k){static const uint8_t map[]={17,4,5,6,8,9,10,12,13,14};for(int i=0;i<10;++i)if(map[i]==k)return i;return -1;}
    static const char* tapLetters(int digit,int mode){
        static const char* lower[]={"0",".,!?1","abc2","def3","ghi4","jkl5","mno6","pqrs7","tuv8","wxyz9"};
        static const char* upper[]={"0",".,!?1","ABC2","DEF3","GHI4","JKL5","MNO6","PQRS7","TUV8","WXYZ9"};
        static const char* symbols[]={"0"," .,!?:;1","@#_$2","%&*3","()-4","+={}5","[]~6","/\\|7","<>8","^'\"9"};
        return mode==0?lower[digit]:mode==1?upper[digit]:symbols[digit];
    }
    bool keySaveReference(int index,bool clear){
        if(index<0||index>=8||!formValid_||!session_.connected())return false;
        if(page_==1)return send(Route::referenceEdit,{{"confirm","YES"},{"sensor",String(index/2)},{"field",String(index%2)},{"op",clear?"clear":"set"},{"value",clear?String(""):text(fields_[index])}});
        if(page_==5&&sub_==0&&index<4)return send(Route::calibrationEdit,{{"confirm","YES"},{"sensor",String(index)},{"op",clear?"clear":"set"},{"value",clear?String("0"):text(fields_[index])}});
        return false;
    }
    void physicalKey(uint8_t k,uint32_t at){
        if(testOverlay_||displaySleeping_||calibrating_||!session_.connected()||k==2||k==3)return;
        if(waiting_&&page_==4&&calcMode_==3)return;
        if(touchTarget_){if(k==15||k==19)touchFinish(k==19);return;}
        if(calcNotes_){if(k==15||k==19)calcConditions();return;}
        if(keyConfirm_>=0){
            const int i=keyConfirm_;keyConfirm_=-1;
            if(k==19){if(keySaveReference(i,true))lv_textarea_set_text(fields_[i],"");}
            else keyHint("Удаление отменено");
            return;
        }
        if(keyEdit_){
            auto* obj=keyEdit_;const int index=fieldIndex(obj);
            if(k==15){cancelKeyEdit(true);keyHint("Изменение отменено");return;}
            if(keyDropdown_){
                if(k==19){cancelKeyEdit(false);focusKey(keyFocus_+1);return;}
                unsigned n=lv_dropdown_get_option_cnt(obj);if(n&&(k==7||k==11))lv_dropdown_set_selected(obj,(lv_dropdown_get_selected(obj)+n+(k==7?n-1:1))%n);
                return;
            }
            const bool numeric=lv_textarea_get_accepted_chars(obj)!=nullptr;
            if(k==19){
                if(page_==1||(page_==5&&sub_==0)){if(!keySaveReference(index,false)){keyHint("Не удалось отправить значение");return;}}
                cancelKeyEdit(false);
                if(page_==4){calculate();}
                else if(page_==5&&sub_==1)focusKey(keyFocus_+1);
                return;
            }
            if(k==0){if(numeric)lv_textarea_set_text(obj,"");else lv_textarea_del_char(obj);lastTapKey_=-1;return;}
            if(k==16||k==18){uint32_t p=lv_textarea_get_cursor_pos(obj);if(k==16&&p)lv_textarea_set_cursor_pos(obj,p-1);else if(k==18)lv_textarea_set_cursor_pos(obj,p+1);lastTapKey_=-1;return;}
            if(k==7||k==11){
                if(!numeric){tapMode_=(tapMode_+(k==7?3:1))%4;lastTapKey_=-1;static const char* modes[]={"abc","ABC","123","SYM"};keyHint(modes[tapMode_]);}
                else if(page_==5&&sub_==0){String v=text(obj);float x=constrain(roundf((v.toFloat()+(k==7?.1f:-.1f))*10)/10,-10.0f,10.0f);lv_textarea_set_text(obj,String(x,1).c_str());}
                else lv_textarea_set_cursor_pos(obj,k==7?0:LV_TEXTAREA_CURSOR_LAST);
                return;
            }
            if(k==busa_keys::longZero){String v=lv_textarea_get_text(obj);if(!numeric||(v.indexOf(',')<0&&v.indexOf('.')<0))lv_textarea_add_char(obj,',');lastTapKey_=-1;return;}
            int digit=keyDigit(k);if(digit<0)return;
            if(numeric||tapMode_==2||digit==0){lv_textarea_add_char(obj,'0'+digit);lastTapKey_=-1;return;}
            const char* letters=tapLetters(digit,tapMode_);const size_t n=strlen(letters);
            if(lastTapKey_==k&&uint32_t(at-lastTapAt_)<950&&lv_textarea_get_cursor_pos(obj)>0){tapIndex_=(tapIndex_+1)%n;lv_textarea_del_char(obj);}
            else tapIndex_=0;
            lv_textarea_add_char(obj,letters[tapIndex_]);lastTapKey_=k;lastTapAt_=at;return;
        }
        if(k==16||k==18){action(100+(page_+6+(k==16?-1:1))%6);return;}
        if(k==15){if(page_==5&&sub_>=0)action(409);else if(page_)action(100+(previousPage_==page_?0:previousPage_));return;}
        if(k==7||k==11){if(keyCount_)focusKey(keyFocus_<0?(k==7?keyCount_-1:0):keyFocus_+(k==7?-1:1));return;}
        if(k==19){if(!keyCount_){if(page_==0)action(101);return;}if(keyFocus_<0){focusKey(0);return;}
            const KeyControl c=keyControls_[keyFocus_];if(c.type==0)action(c.id);else beginKeyEdit(c);return;}
        if(k==0){
            if(keyFocus_>=0&&keyControls_[keyFocus_].type==1){const int i=fieldIndex(keyControls_[keyFocus_].obj);
                if(page_==1&&i>=0&&formValid_){keyConfirm_=i;keyHint("Удалить порог? ENT — да, ESC — нет");return;}
                if(page_==5&&sub_==0&&i>=0&&keySaveReference(i,true)){lv_textarea_set_text(fields_[i],"0.0");return;}}
            if(page_==4)clearCalc();
        }
    }
    void showDisplayTest(uint32_t now){
        if(testOverlay_)lv_obj_del(testOverlay_);
        auto* layer=lv_layer_top();testOverlay_=plain(layer,WIDTH,HEIGHT);
        lv_obj_set_style_bg_color(testOverlay_,color(0),0);lv_obj_set_style_bg_opa(testOverlay_,LV_OPA_COVER,0);
        lv_obj_add_flag(testOverlay_,LV_OBJ_FLAG_CLICKABLE);auto* l=label(testOverlay_,"BUSA MONITOR");
        lv_obj_set_style_text_color(l,color(0xffffff),0);lv_obj_set_style_text_font(l,&busa_font_value,0);lv_obj_center(l);testUntil_=now+5000;
    }
    void consumeKeys(uint32_t now){
        if(!session_.connected()){keyCursor_.reset();return;}
        auto q=session_.snapshot()["keypad_input"];if(q.isNull()){keyCursor_.reset();return;}
        auto a=q["events"].as<JsonArrayConst>();if(a.isNull()||a.size()>busa_keys::capacity){keyCursor_.reset();return;}
        busa_keys::Event events[busa_keys::capacity]={};size_t n=0;
        for(JsonArrayConst e:a){if(e.size()!=3||!e[0].is<uint32_t>()||!e[1].is<uint8_t>()||!e[2].is<uint32_t>()){keyCursor_.reset();return;}events[n++]={e[0].as<uint32_t>(),e[2].as<uint32_t>(),e[1].as<uint8_t>()};}
        bool sleep=q["sleep"]|false;if(sleep!=displaySleeping_){displaySleeping_=sleep;hw_.backlight(!sleep);if(sleep)cancelKeyEdit(true);}
        const uint32_t test=q["test_seq"]|0u;
        const auto result=keyCursor_.consume(q["boot"]|0u,q["head"]|0u,events,n,[&](const busa_keys::Event& e){physicalKey(e.code,now-e.age);});
        if(result==busa_keys::Result::lost){cancelKeyEdit(true);keyHint("Пропущен ввод клавиатуры; повтори нажатие");}
        if(result!=busa_keys::Result::baseline&&test!=lastTestSeq_&&!sleep)showDisplayTest(now);
        lastTestSeq_=test;
    }
    void refresh(){
        const auto& d=session_.snapshot();const bool ok=session_.connected();
        const bool settingsForm=page_==1||page_==2||(page_==5&&sub_==0);
        if(settingsForm&&!ok)formValid_=false;
        if(settingsForm&&ok&&!formValid_){render();return;}
        set(connection_,ok?"UART":"Нет связи");
        setInk(connection_,ok?accent():0xff6473);
        const bool indicators[]={ok&&(d["station"]["connected"]|false),ok&&(d["network_indicators"]["internet"]|false),ok};
        for(int i=0;i<3;++i)setBg(statusDots_[i],indicators[i]?0x53d486:0x8297a1);
        String clock=ok?String(d["clock"]["datetime"]|"—"):String("—");set(clock_,"Дата и время: "+clock);
        if(page_==3&&chart_){graphTimeLabels();if(!ok){clearGraph();set(pageStatus_,"Нет связи");}if(graphKind_==0)for(int i=0;i<4;++i){String s=i==3?"Авария":sensor(i);s+=" "+measured(d["temperatures"][i]["c"]," °C");set(graphLegend_[i],s);}}
        if(page_==0){
            for(int i=0;i<4;++i){auto v=d["sound_alerts"]["channels"][i]["sound_on"];const bool on=ok&&!v.isNull()&&v.as<bool>();
                set(live_[i],!ok||v.isNull()?"Нет связи":on?"Вкл.":"Выкл.");
                setBg(lv_obj_get_parent(live_[i]),on?(i==3?0x42232d:0x14382e):surface());
                setInk(live_[i],on?(i==3?0xff6473:0x53d486):muted());
                set(live_[4+i],measured(d["temperatures"][i]["c"]," °C",1));
                set(live_[12+i],"ВКЛ "+measured(d["display_references"][i]["target"],"",1)+" · ВЫКЛ "+measured(d["display_references"][i]["notice"],"",1));}
            auto rate=d["water"]["lpm"];metric(live_[8],ok&&!rate.isNull()?String(rate.as<double>()*60,1)+" л/ч":"—");
            metric(live_[9],measured(d["water"]["total_l_estimate"]," л",1));
            metric(live_[10],measured(d["collection"]["ml_per_min"]," мл/мин",1));
            metric(live_[11],measured(d["collection"]["total_ml"]," мл",1));
        }else if(page_==1){for(int i=0;i<4;++i)set(live_[i],measured(d["temperatures"][i]["c"]," °C",1));}
        else if(page_==2){auto rate=d["water"]["lpm"];set(live_[0],ok&&!rate.isNull()?String(rate.as<double>()*60,1)+" л/ч":"—");
            set(live_[1],"Учтено: "+measured(d["water"]["total_l_estimate"]," л",2));set(live_[2],String("Импульсы: ")+measured(d["water"]["pulses"],"",0));
            set(live_[3],measured(d["collection"]["ml_per_min"]," мл/мин",1));
            auto collectionRate=d["collection"]["ml_per_min"];set(live_[6],ok&&collectionRate.is<double>()?fmt(collectionRate.as<double>()*0.06)+" л/ч":"— л/ч");
            auto duration=d["collection"]["duration_ms"];if(duration.isNull())duration=d["collection"]["elapsed_ms"];
            set(live_[7],ok&&duration.is<uint32_t>()?"Время отбора: "+String(duration.as<uint32_t>()/1000)+" с":"Время отбора: —");
            set(live_[4],"Собрано: "+measured(d["collection"]["total_ml"]," мл",1));
            set(live_[5],!ok?"Нет связи":(d["collection"]["pulse_seen"]|(d["collection"]["sensor_present"]|false))?((d["collection"]["calibrated"]|false)?"Импульсы зарегистрированы":"Импульсы · нет калибровки"):"Нет импульсов");}
        else if(page_==5){
            if(sub_==1)set(live_[0],ok?String("Wi-Fi: ")+(d["station"]["ssid"]|"—"):"Нет связи");
            else if(sub_==3)set(live_[0],ok?String("Google: ")+String(d["google"]["configured"].as<bool>()?"настроен":"не настроен")+" · код "+String(d["google"]["state"]|0):"Нет связи");
            else if(sub_==4){set(live_[0],ok?String(d["firmware"]|"—"):"Нет связи");set(live_[1],"Время работы: "+measured(d["uptime_ms"]," мс",0));
                set(live_[2],"Wi-Fi: "+String(ok&&(d["wifi"]["sta_connected"]|false)?"подключён":"—"));set(live_[3],"RAM: "+measured(d["esp32"]["free_heap_bytes"]," байт",0));set(live_[4],"PSRAM: "+measured(d["performance"]["psram_total_bytes"]," байт",0));}
            else if(sub_==6){set(live_[0],"UART: "+String(ok?"связь есть":"нет связи"));set(live_[1],"LCD: "+String(hw_.lcdReady()?"инициализирован":"ошибка"));set(live_[2],"Сенсор: "+String(calibrationValid_?"калиброван":"нет"));set(live_[3],"microSD: "+String(hw_.sdReady()?"доступна":"нет"));
                set(live_[4],"GPIO15: "+String(ok?(d["diagnostics"]["water_flow"]["status"]|"—"):"—"));set(live_[5],"PCF8575: "+String(ok&&(d["modules"]["pcf8575"]|false)?"доступен":"—"));
                set(live_[6],"I2C ошибок: "+measured(d["performance"]["i2c_errors"],"",0));set(live_[7],"Макс. цикл: "+measured(d["performance"]["loop_worst_ms"]," мс",0));
                set(live_[8],"Клавиатура: "+String(ok?(d["keypad"]["message"]|"—"):"—"));set(live_[9],"Следующая: "+String(ok?(d["keypad"]["expected"]|"—"):"—"));}
        }
    }
    void animate(uint32_t now){
        const auto& d=session_.snapshot();bool water=session_.connected()&&!d["water"]["lpm"].isNull()&&d["water"]["lpm"].as<float>()>0;
        bool temp=session_.connected()&&!d["temperatures"][2]["c"].isNull();
        for(int i=0;i<7;++i){if(!motion_[i])continue;bool active=i<4?water:temp;
            if(!active){if(!lv_obj_has_flag(motion_[i],LV_OBJ_FLAG_HIDDEN))lv_obj_add_flag(motion_[i],LV_OBJ_FLAG_HIDDEN);continue;}
            if(lv_obj_has_flag(motion_[i],LV_OBJ_FLAG_HIDDEN))lv_obj_clear_flag(motion_[i],LV_OBJ_FLAG_HIDDEN);
            int t=(now/40+i*23)%100,x=0,y=0;
            if(i<2){x=142;y=1144-t*8;}else if(i==2){x=684+t*2;y=367;}else if(i==3){x=788+t*2;y=913;}
            else{x=300+(i-4)*90;y=1420-t;}
            // LVGL invalidation only when the quantized display position changes.
            const int16_t px=x*BUSA_ASSET_W/1024,py=y*BUSA_ASSET_H/1536;
            if(lv_obj_get_x(motion_[i])!=px||lv_obj_get_y(motion_[i])!=py)lv_obj_set_pos(motion_[i],px,py);}
    }
    Hardware& hw_;DisplaySession& session_;Preferences prefs_;
    lv_disp_drv_t driver_{};lv_disp_draw_buf_t drawBuffer_{};lv_indev_drv_t input_{};
    lv_obj_t *connection_=nullptr,*body_=nullptr,*nav_=nullptr,*keyboard_=nullptr,*pageStatus_=nullptr,*resultLabel_=nullptr,*wifiList_=nullptr,*chart_=nullptr,*calMark_=nullptr;
    busa_keys::Cursor keyCursor_;KeyControl keyControls_[64]={};int keyCount_=0,keyFocus_=-1,keyConfirm_=-1,previousPage_=0;
    lv_obj_t* keyEdit_=nullptr;String keyOriginal_;bool keyDropdown_=false,displaySleeping_=false;uint16_t keyOriginalChoice_=0;
    int lastTapKey_=-1,tapMode_=0;size_t tapIndex_=0;uint32_t lastTapAt_=0,lastTestSeq_=0,testUntil_=0;lv_obj_t* testOverlay_=nullptr;
    lv_obj_t* clock_=nullptr;lv_obj_t* statusDots_[3]={};lv_obj_t* graphRange_[5]={};lv_obj_t* graphLegend_[4]={};lv_obj_t* graphY_[4]={};lv_obj_t* graphX_[4]={};bool graphDirty_=false;
    lv_obj_t* navButtons_[6]={};lv_obj_t* fields_[8]={};lv_obj_t* live_[16]={};lv_obj_t* motion_[7]={};lv_chart_series_t* series_[4]={};
    lv_img_dsc_t art_{},navArt_[6]{};uint8_t* asset_=nullptr;
    lv_obj_t *calcNotes_=nullptr,*touchModal_=nullptr,*touchTitle_=nullptr,*touchEditor_=nullptr,*touchTarget_=nullptr,*pageCounter_=nullptr;String pageContent_;unsigned pageOffset_=0,pageLines_=5;
    busa_touch::Transform transform_{};busa_touch::Point raw_[3]{},screen_[3]{};
    bool calibrationValid_=false,calibrating_=false,held_=false,wasConnected_=false,waiting_=false,scanPending_=false,frameOpen_=false,awaitRelease_=false,formValid_=false;
    int calIndex_=0,calSamples_=0,sumX_=0,sumY_=0,page_=0,sub_=-1,pendingPage_=0,pendingSub_=-1,graphKind_=0,graphMinutes_=5,calcMode_=0;
    uint32_t lastCalibrationRead_=0;
    bool themeSynced_=false;uint8_t theme_=2,requestedTheme_=2;uint32_t lastTick_=0,lastAnimation_=0,revision_=0,lastScan_=0,frameStart_=0,lastGraph_=0,viewGeneration_=0,pendingGeneration_=0;
    bool geometryExpanded_=true,geometryReadAttempted_=false;
    Route pendingRoute_=Route::wifiScan;String networks_[32];unsigned networkCount_=0;String calcInput_[4][5];
};
constexpr uint32_t Interface::colors_[4];
}
