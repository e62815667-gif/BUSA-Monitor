#pragma once
#include <lvgl.h>
#include <Preferences.h>
#include <initializer_list>
#include "BUSA_GuitionHardware.h"
#include "BUSA_DisplaySession.h"
#include "BUSA_TouchCalibration.h"
#include "BUSA_Calculator.h"
#include "BUSA_DisplayAsset.h"
LV_FONT_DECLARE(busa_font);
LV_FONT_DECLARE(busa_font_small);
LV_FONT_DECLARE(busa_font_value);
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
        int code;String body;
        if(session_.takeApiResult(code,body)){waiting_=false;handleResult(code,body);}
        if(!calibrating_){
            if(revision_!=session_.snapshotRevision()||wasConnected_!=session_.connected()){
                revision_=session_.snapshotRevision();wasConnected_=session_.connected();refresh();
            }
            if(page_==0&&now-lastAnimation_>=60){lastAnimation_=now;animate(now);}
            if(page_==3&&!waiting_&&session_.connected()&&now-lastGraph_>=5000)requestGraph();
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
    uint32_t ink()const{return theme_==0?0x19384b:0xe6f2f7;}
    uint32_t paper()const{return theme_==0?0xf0f6f8:0x060e15;}
    uint32_t surface()const{return theme_==0?0xffffff:0x0d2430;}
    uint32_t line()const{return theme_==0?0xb6d2df:0x285064;}
    static constexpr uint32_t accent(){return 0x009fde;}
    uint32_t muted()const{return theme_==0?0x527181:0x9bb6c4;}
    static constexpr int TOP=28, NAV=42, PAD=3, GAP=3, SCENE=138, METRICS=27, RELAYS=25;
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
        if(self->calibrating_||!self->calibrationValid_||!self->hw_.readRawTouch(raw))return;
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
        calibrating_=true;calibrationValid_=false;calIndex_=0;held_=false;awaitRelease_=true;calSamples_=0;sumX_=sumY_=0;
        lastCalibrationRead_=millis()-busa_touch::readPeriodMs;
        auto* root=lv_scr_act();lv_obj_clean(root);rootStyle(root);
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
        self->action(id);
    }
    static void textFocus(lv_event_t* e){
        auto* self=static_cast<Interface*>(lv_event_get_user_data(e));auto* ta=lv_event_get_target(e);
        lv_keyboard_set_textarea(self->keyboard_,ta);lv_keyboard_set_mode(self->keyboard_,lv_textarea_get_accepted_chars(ta)?LV_KEYBOARD_MODE_NUMBER:LV_KEYBOARD_MODE_TEXT_LOWER);
        lv_obj_clear_flag(self->keyboard_,LV_OBJ_FLAG_HIDDEN);lv_obj_move_foreground(self->keyboard_);
        lv_obj_set_height(self->body_,HEIGHT-TOP-128);
        lv_obj_update_layout(self->body_);
        lv_obj_scroll_to_view(ta,LV_ANIM_OFF);
    }
    static void keyboardEvent(lv_event_t* e){
        auto* self=static_cast<Interface*>(lv_event_get_user_data(e));
        if(lv_event_get_code(e)==LV_EVENT_READY||lv_event_get_code(e)==LV_EVENT_CANCEL)self->hideKeyboard();
    }
    void hideKeyboard(){if(!keyboard_)return;lv_keyboard_set_textarea(keyboard_,nullptr);lv_obj_add_flag(keyboard_,LV_OBJ_FLAG_HIDDEN);lv_obj_set_height(body_,HEIGHT-TOP-NAV);}
    lv_obj_t* label(lv_obj_t* parent,const char* text){
        auto* o=lv_label_create(parent);lv_obj_set_style_text_font(o,&busa_font,0);lv_obj_set_style_text_color(o,color(ink()),0);lv_label_set_text(o,text);return o;
    }
    lv_obj_t* smallLabel(lv_obj_t* parent,const char* text){
        auto* o=label(parent,text);lv_obj_set_style_text_font(o,&busa_font_small,0);return o;
    }
    lv_obj_t* plain(lv_obj_t* parent,int w,int h){
        auto* o=lv_obj_create(parent);lv_obj_remove_style_all(o);lv_obj_set_size(o,w,h);
        lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE);return o;
    }
    lv_obj_t* at(lv_obj_t* parent,const char* text,int x,int y,const lv_font_t* font=&busa_font){
        auto* o=label(parent,text);lv_obj_set_style_text_font(o,font,0);lv_obj_set_pos(o,x,y);return o;
    }
    lv_obj_t* button(lv_obj_t* parent,const char* text,int id,int width=LV_PCT(100)){
        auto* o=lv_btn_create(parent);lv_obj_set_size(o,width,28);lv_obj_set_style_radius(o,6,0);
        lv_obj_set_style_bg_color(o,color(surface()),0);lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);
        lv_obj_set_style_shadow_width(o,0,0);lv_obj_set_style_border_width(o,1,0);
        lv_obj_set_style_border_color(o,color(line()),0);lv_obj_set_style_pad_all(o,1,0);
        lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE);
        auto* l=label(o,text);lv_obj_set_style_text_color(l,color(ink()),0);
        lv_obj_set_width(l,LV_PCT(100));lv_label_set_long_mode(l,LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(l,LV_TEXT_ALIGN_CENTER,0);lv_obj_center(l);
        lv_obj_set_user_data(o,reinterpret_cast<void*>(intptr_t(id)));lv_obj_add_event_cb(o,event,LV_EVENT_CLICKED,this);return o;
    }
    void selected(lv_obj_t* o,bool on){
        lv_obj_set_style_bg_color(o,color(on?(theme_==0?0x007bb3:0x11455d):surface()),0);
        lv_obj_set_style_border_color(o,color(on?accent():line()),0);
        lv_obj_set_style_text_color(lv_obj_get_child(o,0),color(on?0xffffff:ink()),0);
    }
    lv_obj_t* panel(lv_obj_t* parent,int width=LV_PCT(100),int height=LV_SIZE_CONTENT){
        auto* o=lv_obj_create(parent);lv_obj_set_size(o,width,height);lv_obj_set_style_bg_color(o,color(surface()),0);lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);
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
        auto* box=plain(parent,width,LV_SIZE_CONTENT);lv_obj_set_flex_flow(box,LV_FLEX_FLOW_COLUMN);lv_obj_set_style_pad_row(box,1,0);
        auto* caption=smallLabel(box,title);lv_obj_set_style_text_color(caption,color(muted()),0);
        lv_obj_set_width(caption,LV_PCT(100));lv_label_set_long_mode(caption,LV_LABEL_LONG_WRAP);
        auto* ta=lv_textarea_create(box);lv_obj_set_width(ta,LV_PCT(100));lv_obj_set_height(ta,26);
        lv_obj_set_style_text_font(ta,&busa_font,0);lv_obj_set_style_text_color(ta,color(ink()),0);lv_obj_set_style_bg_color(ta,color(surface()),0);
        lv_obj_set_style_border_color(ta,color(line()),0);lv_obj_set_style_border_width(ta,1,0);lv_obj_set_style_radius(ta,5,0);lv_obj_set_style_pad_all(ta,3,0);lv_textarea_set_one_line(ta,true);
        lv_textarea_set_max_length(ta,numeric?14:220);
        if(numeric)lv_textarea_set_accepted_chars(ta,"0123456789.-,");
        lv_textarea_set_password_mode(ta,secret);lv_textarea_set_text(ta,value.c_str());
        lv_obj_add_event_cb(ta,textFocus,LV_EVENT_CLICKED,this);return ta;
    }
    void navIcon(lv_obj_t* parent,int i){
        static const lv_point_t points[6][12]={
            {{1,8},{10,1},{19,8},{16,8},{16,19},{11,19},{11,12},{7,12},{7,19},{3,19},{3,8},{1,8}},
            {{7,13},{7,3},{9,1},{11,1},{13,3},{13,13},{16,16},{15,19},{12,21},{8,21},{5,18},{7,13}},
            {{10,1},{4,9},{2,14},{4,18},{8,20},{12,20},{16,18},{18,14},{16,9},{10,1},{10,1},{10,1}},
            {{2,2},{2,19},{19,19},{2,19},{2,15},{7,10},{11,13},{18,4},{18,4},{18,4},{18,4},{18,4}},
            {{4,1},{16,1},{16,20},{4,20},{4,1},{4,7},{16,7},{4,7},{4,13},{16,13},{10,13},{10,20}},
            {{7,1},{13,1},{14,5},{18,6},{20,11},{17,14},{17,18},{12,20},{8,18},{4,18},{1,12},{7,1}}
        };
        auto* icon=lv_line_create(parent);lv_line_set_points(icon,points[i],12);
        lv_obj_set_style_line_width(icon,2,0);lv_obj_set_style_line_rounded(icon,true,0);
        lv_obj_set_style_line_color(icon,color(theme_==0?0x007bb3:0xa5e8f5),0);lv_obj_align(icon,LV_ALIGN_TOP_MID,0,0);
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
    void build(){
        auto* root=lv_scr_act();lv_obj_clean(root);rootStyle(root);lv_obj_clear_flag(root,LV_OBJ_FLAG_SCROLLABLE);
        auto* top=plain(root,WIDTH,TOP);lv_obj_set_pos(top,0,0);
        at(top,"BUSA MONITOR",6,0);clock_=at(top,"Дата и время: —",6,15,&busa_font_small);
        lv_obj_set_style_text_color(clock_,color(muted()),0);
        for(int i=0;i<3;++i){static const char* names[]={"Wi-Fi","Google","ESP32"};auto* p=plain(top,65,18);lv_obj_set_pos(p,205+i*68,4);
            lv_obj_set_style_radius(p,9,0);lv_obj_set_style_border_width(p,1,0);lv_obj_set_style_border_color(p,color(line()),0);
            statusDots_[i]=plain(p,5,5);lv_obj_set_style_radius(statusDots_[i],LV_RADIUS_CIRCLE,0);lv_obj_set_style_bg_opa(statusDots_[i],LV_OPA_COVER,0);lv_obj_set_pos(statusDots_[i],5,6);
            at(p,names[i],15,2,&busa_font_small);}
        connection_=at(top,"UART",430,7,&busa_font_small);
        body_=lv_obj_create(root);lv_obj_set_pos(body_,0,TOP);lv_obj_set_size(body_,WIDTH,HEIGHT-TOP-NAV);
        lv_obj_set_style_bg_color(body_,color(paper()),0);lv_obj_set_style_border_width(body_,0,0);lv_obj_set_style_radius(body_,0,0);
        lv_obj_set_style_pad_all(body_,PAD,0);lv_obj_set_style_pad_row(body_,GAP,0);lv_obj_set_flex_flow(body_,LV_FLEX_FLOW_COLUMN);
        lv_obj_set_scroll_dir(body_,LV_DIR_VER);
        nav_=plain(root,WIDTH-6,NAV-2);lv_obj_set_pos(nav_,3,HEIGHT-NAV);
        lv_obj_set_style_radius(nav_,9,0);lv_obj_set_style_border_width(nav_,1,0);lv_obj_set_style_border_color(nav_,color(line()),0);
        lv_obj_set_style_bg_color(nav_,color(surface()),0);lv_obj_set_style_bg_opa(nav_,LV_OPA_COVER,0);
        lv_obj_set_style_pad_all(nav_,2,0);lv_obj_set_style_pad_column(nav_,2,0);lv_obj_set_flex_flow(nav_,LV_FLEX_FLOW_ROW);
        static const char* titles[]={"Главная","Температуры","Вода и отбор","Графики","Калькулятор","Настройки"};
        for(int i=0;i<6;++i){auto* b=button(nav_,titles[i],100+i,76);lv_obj_set_height(b,34);
            auto* l=lv_obj_get_child(b,0);lv_obj_set_style_text_font(l,&busa_font_small,0);lv_obj_align(l,LV_ALIGN_BOTTOM_MID,0,0);
            navIcon(b,i);navButtons_[i]=b;}
        keyboard_=lv_keyboard_create(root);lv_obj_set_size(keyboard_,WIDTH,128);lv_obj_align(keyboard_,LV_ALIGN_BOTTOM_MID,0,0);
        lv_obj_add_flag(keyboard_,LV_OBJ_FLAG_HIDDEN);lv_obj_add_event_cb(keyboard_,keyboardEvent,LV_EVENT_ALL,this);
        render();
    }
    void render(){
        hideKeyboard();lv_obj_clean(body_);++viewGeneration_;formValid_=session_.connected();for(auto& p:fields_)p=nullptr;for(auto& p:live_)p=nullptr;for(auto& p:motion_)p=nullptr;
        wifiList_=nullptr;chart_=nullptr;resultLabel_=nullptr;pageStatus_=nullptr;scanPending_=false;
        for(int i=0;i<6;++i)selected(navButtons_[i],i==page_);
        if(page_==0){lv_obj_clear_flag(body_,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_scrollbar_mode(body_,LV_SCROLLBAR_MODE_OFF);}
        else{lv_obj_add_flag(body_,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_scrollbar_mode(body_,LV_SCROLLBAR_MODE_AUTO);}
        switch(page_){case 0:home();break;case 1:temperatures();break;case 2:water();break;case 3:graphs();break;case 4:calculator();break;case 5:settings();break;}
        refresh();
    }
    void home(){
        auto* scene=plain(body_,LV_PCT(100),SCENE);
        auto* center=plain(scene,BUSA_ASSET_W,BUSA_ASSET_H);lv_obj_align(center,LV_ALIGN_CENTER,0,0);
        auto* im=lv_img_create(center);lv_img_set_src(im,&art_);lv_img_set_antialias(im,true);lv_obj_center(im);
        for(unsigned i=0;i<7;++i){auto* dot=plain(center,3,3);
            lv_obj_set_style_bg_color(dot,color(i<4?0x38bdf8:0xc5fffa),0);lv_obj_set_style_bg_opa(dot,LV_OPA_COVER,0);lv_obj_set_style_radius(dot,LV_RADIUS_CIRCLE,0);motion_[i]=dot;}
        static const int xs[]={10,10,318,318},ys[]={83,12,83,12};
        for(int i=0;i<4;++i){at(scene,i==3?"Авария":sensor(i),xs[i],ys[i]);
            live_[4+i]=at(scene,"—",xs[i],ys[i]+17,&busa_font_value);lv_obj_set_style_text_color(live_[4+i],color(colors_[i]),0);}
        live_[11]=at(scene,"Собрано — мл",318,116,&busa_font_small);
        auto* m=plain(body_,LV_PCT(100),METRICS);
        lv_obj_set_style_bg_color(m,color(surface()),0);lv_obj_set_style_bg_opa(m,LV_OPA_COVER,0);
        lv_obj_set_style_border_width(m,1,0);lv_obj_set_style_border_color(m,color(line()),0);lv_obj_set_style_radius(m,6,0);
        static const char* headings[]={"Расход","Израсходовано","Отбор"};
        for(int i=0;i<3;++i){auto* l=at(m,headings[i],7+i*156,0,&busa_font_small);lv_obj_set_style_text_color(l,color(muted()),0);
            live_[8+i]=at(m,"—",7+i*156,11,&busa_font);}
        auto* bar=row(body_,RELAYS);
        static const char* relayNames[]={"ТЭН откл.","Кран закрыт","Кран открыт","Авария"};
        for(int i=0;i<4;++i){auto* p=plain(bar,115,RELAYS);lv_obj_set_style_radius(p,6,0);
            lv_obj_set_style_bg_color(p,color(surface()),0);lv_obj_set_style_bg_opa(p,LV_OPA_COVER,0);
            lv_obj_set_style_border_color(p,color(line()),0);lv_obj_set_style_border_width(p,1,0);
            at(p,relayNames[i],5,0,&busa_font_small);live_[i]=at(p,"Нет связи",5,12,&busa_font_small);}
    }
    void temperatures(){
        const auto& doc=session_.snapshot();label(body_,"ТЕМПЕРАТУРЫ");
        static const int order[]={0,2,1,3};
        for(int lineNo=0;lineNo<2;++lineNo){auto* r=row(body_,68);
            for(int col=0;col<2;++col){int i=order[lineNo*2+col];auto* p=panel(r,235,68);auto* head=row(p,21);
                label(head,i==3?"Авария":sensor(i));live_[i]=label(head,"—");lv_obj_set_style_text_font(live_[i],&busa_font_value,0);lv_obj_set_style_text_color(live_[i],color(colors_[i]),0);
                auto* fields=row(p,39);fields_[2*i]=field(fields,"ВКЛ, °C",value(doc["display_references"][i]["target"]),true,false,110);
                fields_[2*i+1]=field(fields,"ВЫКЛ, °C",value(doc["display_references"][i]["notice"]),true,false,110);}}
        selected(button(body_,"Сохранить пороги",200),true);pageStatus_=smallLabel(body_,"");lv_obj_set_width(pageStatus_,LV_PCT(100));
    }
    void water(){
        label(body_,"ВОДА И ОТБОР");auto* r=row(body_,142);
        auto* q=panel(r,235,142);label(q,"Отбор жидкости");live_[3]=label(q,"—");live_[4]=label(q,"—");live_[5]=label(q,"—");
        lv_obj_set_style_text_font(live_[3],&busa_font_value,0);lv_obj_set_style_text_color(live_[3],color(colors_[0]),0);
        auto* p=panel(r,235,142);label(p,"Охлаждение");live_[0]=label(p,"—");live_[1]=label(p,"—");live_[2]=smallLabel(p,"—");
        lv_obj_set_style_text_font(live_[0],&busa_font_value,0);lv_obj_set_style_text_color(live_[0],color(colors_[0]),0);
        const auto& d=session_.snapshot();fields_[0]=field(p,"Коэффициент, имп/л",value(d["water"]["pulses_per_litre"]),true);
        selected(button(body_,"Сохранить коэффициент",201),true);pageStatus_=smallLabel(body_,"");lv_obj_set_width(pageStatus_,LV_PCT(100));
    }
    void graphs(){
        auto* r=row(body_,25);
        selected(button(r,"Температура",300,156),graphKind_==0);
        selected(button(r,"Отбор",302,156),graphKind_==2);selected(button(r,"Охлаждение",301,156),graphKind_==1);
        auto* legend=row(body_,13);
        for(int i=0;i<4;++i){auto* l=smallLabel(legend,i==3?"Авария":sensor(i));lv_obj_set_style_text_color(l,color(colors_[i]),0);}
        chart_=lv_chart_create(body_);lv_obj_set_size(chart_,LV_PCT(100),115);lv_chart_set_type(chart_,LV_CHART_TYPE_LINE);
        lv_chart_set_point_count(chart_,160);lv_obj_set_style_bg_color(chart_,color(surface()),0);lv_obj_set_style_border_color(chart_,color(line()),0);
        lv_obj_set_style_line_color(chart_,color(line()),LV_PART_MAIN);lv_obj_set_style_line_width(chart_,2,LV_PART_ITEMS);
        lv_obj_set_style_size(chart_,0,LV_PART_INDICATOR);lv_chart_set_div_line_count(chart_,4,5);
        lv_chart_set_range(chart_,LV_CHART_AXIS_PRIMARY_Y,0,1250);
        for(int i=0;i<4;++i)series_[i]=lv_chart_add_series(chart_,color(colors_[i]),LV_CHART_AXIS_PRIMARY_Y);
        auto* ranges=row(body_,28);
        for(int i=0;i<5;++i){static const char* names[]={"5 мин","10 мин","15 мин","30 мин","1 ч"};static const int mins[]={5,10,15,30,60};
            graphRange_[i]=button(ranges,names[i],310+i,91);selected(graphRange_[i],graphMinutes_==mins[i]);}
        pageStatus_=smallLabel(body_,"Выберите период");lv_obj_set_width(pageStatus_,LV_PCT(100));requestGraph();
    }
    void requestGraph(){lastGraph_=millis();send(Route::chartHistory,{{"minutes",String(graphMinutes_)}});}
    void calculator(){
        static const char* names[]={"Разбавление","Ареометр","Расчёт выхода","Объём куба"};
        static const char* labels[4][5]={{"Объём, л","Исходная, %","Нужная, %","",""},{"Температура, °C","Показание, %","","",""},{"Объём, л","Исходная, %","На выходе, %","Доля 1, %","Доля 2, %"},{"Диаметр, см","Высота, см","Уровень, см","",""}};
        static const int counts[]={3,2,5,3};
        auto* r=row(body_,28);for(int i=0;i<4;++i){auto* b=button(r,names[i],320+i,115);lv_obj_set_style_text_font(lv_obj_get_child(b,0),&busa_font_small,0);selected(b,i==calcMode_);}
        label(body_,names[calcMode_]);
        for(int i=0;i<counts[calcMode_];){auto* inputs=row(body_);for(int col=0;col<3&&i<counts[calcMode_];++col,++i)fields_[i]=field(inputs,labels[calcMode_][i],calcInput_[calcMode_][i],true,false,153);}
        if(calcMode_==3){auto* buttons=row(body_,28);selected(button(buttons,"Рассчитать",330,153),true);button(buttons,"Загрузить размеры",331,153);button(buttons,"Сохранить размеры",332,153);}
        else selected(button(body_,"Рассчитать",330),true);
        auto* p=panel(body_);resultLabel_=label(p,"—");lv_obj_set_width(resultLabel_,LV_PCT(100));lv_label_set_long_mode(resultLabel_,LV_LABEL_LONG_WRAP);
    }
    void settings(){
        label(body_,"НАСТРОЙКИ");
        if(sub_<0){static const char* names[]={"Температура","Wi-Fi","Экран","Google","Bluetooth · RainMaker","Обновление ПО","Журнал событий","Диагностика","О системе"};
            static const int ids[]={400,401,402,403,407,408,405,406,404};
            for(int i=0;i<9;){auto* r=row(body_,29);for(int col=0;col<2&&i<9;++col,++i)button(r,names[i],ids[i],235);}return;}
        button(body_,"Назад",409);
        const auto& d=session_.snapshot();
        if(sub_==0){for(int i=0;i<4;++i)fields_[i]=field(body_,sensor(i),value(d["calibration_offsets"][i]),true);button(body_,"Сохранить поправки",202);}
        else if(sub_==1){live_[0]=label(body_,"Wi-Fi —");button(body_,"Обновить список",203);wifiList_=lv_dropdown_create(body_);lv_obj_set_width(wifiList_,LV_PCT(100));lv_dropdown_set_options(wifiList_,"Нет сетей");fields_[0]=field(body_,"Пароль","",false,true);lv_textarea_set_max_length(fields_[0],63);button(body_,"Подключить",204);button(body_,"Отключить",205);}
        else if(sub_==2){button(body_,"Светлая тема",206);button(body_,"Тёмная тема",207);button(body_,"Системная тема",208);button(body_,"Калибровка сенсора",209);}
        else if(sub_==3){live_[0]=label(body_,"Google —");fields_[0]=field(body_,"URL Apps Script","",false);fields_[1]=field(body_,"Токен","",false,true);button(body_,"Сохранить подключение",210);button(body_,"Проверить отправку",211);button(body_,"Отключить передачу",212);}
        else if(sub_==4){for(int i=0;i<5;++i){live_[i]=label(body_,"—");lv_obj_set_width(live_[i],LV_PCT(100));lv_label_set_long_mode(live_[i],LV_LABEL_LONG_WRAP);}}
        else if(sub_==5){button(body_,"Обновить журнал",213);resultLabel_=label(body_,"");lv_obj_set_width(resultLabel_,LV_PCT(100));lv_label_set_long_mode(resultLabel_,LV_LABEL_LONG_WRAP);send(Route::events,{{"after","0"}});}
        else if(sub_==6){for(int i=0;i<10;++i){live_[i]=label(body_,"—");lv_obj_set_width(live_[i],LV_PCT(100));lv_label_set_long_mode(live_[i],LV_LABEL_LONG_WRAP);}
            button(body_,"Обучение клавиатуры",217);button(body_,"Считать и далее",218);button(body_,"Предыдущий шаг",219);button(body_,"Отмена обучения",220);}
        else if(sub_==7){button(body_,"Включить сопряжение",214);button(body_,"Отключить Bluetooth",215);button(body_,"Обновить состояние",216);resultLabel_=label(body_,"");lv_obj_set_width(resultLabel_,LV_PCT(100));lv_label_set_long_mode(resultLabel_,LV_LABEL_LONG_WRAP);}
        else if(sub_==8){auto* l=label(body_,"Обновление через локальное приложение");lv_obj_set_width(l,LV_PCT(100));lv_label_set_long_mode(l,LV_LABEL_LONG_WRAP);}
        pageStatus_=label(body_,"");lv_obj_set_width(pageStatus_,LV_PCT(100));lv_label_set_long_mode(pageStatus_,LV_LABEL_LONG_WRAP);
    }
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
        if(id>=100&&id<106){page_=id-100;sub_=-1;render();return;}
        if(id>=400&&id<409){sub_=id-400;render();return;}
        if(id==409){sub_=-1;render();return;}
        if(id>=300&&id<=302){graphKind_=id-300;render();return;}
        if(id>=310&&id<=314){static const int minutes[]={5,10,15,30,60};graphMinutes_=minutes[id-310];for(int i=0;i<5;++i)selected(graphRange_[i],i==id-310);requestGraph();return;}
        if(id>=320&&id<=323){calcMode_=id-320;render();return;}
        hideKeyboard();
        if((id==200||id==201||id==202)&&(!formValid_||!session_.connected())){set(pageStatus_,"Нет актуальных настроек. Дождитесь связи.");return;}
        switch(id){
            case 200:alarms();break;
            case 201:send(Route::waterCoefficient,{{"confirm","YES"},{"pulses_per_litre",text(fields_[0])}});break;
            case 202:form(Route::calibration,"offset",4);break;
            case 203:send(Route::wifiScan,{{"start","1"},{"fresh","1"}});break;
            case 204:{unsigned i=lv_dropdown_get_selected(wifiList_);if(i<networkCount_)send(Route::wifiConnect,{{"ssid",networks_[i]},{"pass",lv_textarea_get_text(fields_[0])}});else set(pageStatus_,"Выберите сеть");break;}
            case 205:send(Route::wifiDisconnect,{});break;
            case 206:case 207:case 208:requestedTheme_=id-206;send(Route::theme,{{"confirm","YES"},{"mode",String(requestedTheme_)}});break;
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
            case 330:calculate();break;
            case 331:send(Route::geometryRead,{});break;
            case 332:send(Route::geometrySave,{{"confirm","YES"},{"diameter",text(fields_[0])},{"height",text(fields_[1])}});break;
        }
    }
    void calculate(){
        static const int counts[]={3,2,5,3};double v[5]={};
        for(int i=0;i<counts[calcMode_];++i){
            String s=text(fields_[i]);calcInput_[calcMode_][i]=s;char* e=nullptr;v[i]=strtod(s.c_str(),&e);
            if(calcMode_==3&&i==2&&s.isEmpty())continue;
            if(s.isEmpty()||*e||!std::isfinite(v[i])){set(resultLabel_,"Проверьте значения");return;}
        }
        BusaCalc::Result r;
        if(calcMode_==0)r=BusaCalc::dilute(v[0],v[1],v[2]);
        else if(calcMode_==1)r=BusaCalc::correct(v[0],v[1]);
        else if(calcMode_==2)r=BusaCalc::fraction(v[0],v[1],v[2],v[3],v[4]);
        else r=calcInput_[3][2].isEmpty()?BusaCalc::cylinder(v[0],v[1]):BusaCalc::cylinder(v[0],v[1],v[2]);
        if(!r.ok){set(resultLabel_,"Недопустимые значения");return;}
        if(calcMode_==0)set(resultLabel_,"Вода: "+String(r.water,3)+" л\nИтого: "+String(r.total,3)+" л");
        else if(calcMode_==1)set(resultLabel_,"Результат: "+String(r.strength,2)+" %");
        else if(calcMode_==2)set(resultLabel_,"Доля 1: "+String(r.heads,3)+" л\nДоля 2: "+String(r.tails,3)+" л\nВыход: "+String(r.product,3)+" л");
        else set(resultLabel_,"Вместимость: "+String(r.capacity,3)+" л"+(r.levelKnown?String("\nОбъём: ")+String(r.liquid,3)+" л":""));
    }
    void handleResult(int code,const String& body){
        if(calibrating_||pendingPage_!=page_||pendingSub_!=sub_||pendingGeneration_!=viewGeneration_)return;
        if(code<200||code>=300){set(pageStatus_,code==0?String("Ответ не получен; проверьте состояние"):String("Ошибка ")+code+": "+body.substring(0,100));return;}
        set(pageStatus_,"Выполнено");
        if(pendingRoute_==Route::geometryRead&&calcMode_==3){JsonDocument d;if(!deserializeJson(d,body)){for(int i=0;i<2;++i){calcInput_[3][i]=d[i?"height":"diameter"]|"";lv_textarea_set_text(fields_[i],calcInput_[3][i].c_str());}set(resultLabel_,"Размеры загружены");}return;}
        if(pendingRoute_==Route::geometrySave){set(resultLabel_,"Размеры сохранены");return;}
        if(pendingRoute_>=Route::keypadStart&&pendingRoute_<=Route::keypadCancel){set(pageStatus_,body);return;}
        if(pendingRoute_==Route::theme){theme_=requestedTheme_;prefs_.putUChar("theme",theme_);build();return;}
        if(pendingRoute_==Route::wifiConnect||pendingRoute_==Route::googleConfig){if(fields_[0]&&pendingRoute_==Route::wifiConnect)lv_textarea_set_text(fields_[0],"");if(fields_[1]&&pendingRoute_==Route::googleConfig)lv_textarea_set_text(fields_[1],"");}
        if(pendingRoute_==Route::wifiScan){JsonDocument d;if(deserializeJson(d,body)){set(pageStatus_,"Ошибка списка сетей");return;}scanPending_=d["scanning"]|false;
            if(scanPending_){set(pageStatus_,"Поиск сетей…");return;}
            networkCount_=0;String options;
            for(JsonObjectConst n:d["networks"].as<JsonArrayConst>()){if(networkCount_>=32)break;const char* ssid=n["ssid"]|"";if(!*ssid)continue;networks_[networkCount_++]=ssid;String display=ssid;display.replace('\n',' ');display.replace('\r',' ');if(options.length())options+="\n";options+=display+" ("+String(n["rssi"]|0)+")";}
            lv_dropdown_set_options(wifiList_,options.length()?options.c_str():"Нет сетей");set(pageStatus_,d["error"]|"");return;}
        if(pendingRoute_==Route::chartHistory&&chart_){
            JsonDocument d;if(deserializeJson(d,body)){set(pageStatus_,"Ошибка графика");return;}
            const char* flowKey=graphKind_==2?"collection_ml_min":"cooling_lph";
            int count=graphKind_==0?d["temperature"][0].size():d[flowKey].size();if(count<=0)return;
            lv_chart_set_point_count(chart_,count);double lo=0,hi=graphKind_==0?125:1;
            for(int k=0;k<4;++k){JsonArrayConst arr=graphKind_==0?d["temperature"][k].as<JsonArrayConst>():d[flowKey].as<JsonArrayConst>();
                for(int i=0;i<count;++i){JsonVariantConst v=arr[i];bool valid=!v.isNull()&&(graphKind_==0||k==0);
                    double x=valid?v.as<double>():0;if(valid){lo=min(lo,x);hi=max(hi,x);}
                    lv_chart_set_value_by_id(chart_,series_[k],i,valid?lv_coord_t(constrain(x*10,-30000.0,30000.0)):LV_CHART_POINT_NONE);}}
            lv_chart_set_range(chart_,LV_CHART_AXIS_PRIMARY_Y,lv_coord_t(max(-30000.0,lo*10)),lv_coord_t(min(30000.0,hi*10)));lv_chart_refresh(chart_);return;}
        if(pendingRoute_==Route::events){JsonDocument d;if(deserializeJson(d,body)){set(pageStatus_,"Ошибка журнала");return;}String s;
            for(JsonObjectConst e:d["events"].as<JsonArrayConst>()){s+=String((e["uptime_ms"]|0u)/1000)+" с  "+String(e["text"]|"")+"\n";if(s.length()>7000)break;}set(resultLabel_,s.length()?s:"Нет событий");}
        else if(pendingRoute_==Route::rainmakerStatus){JsonDocument d;if(!deserializeJson(d,body))set(resultLabel_,String("BLE: ")+(d["ble"]|"—")+"\n"+(d["feedback"]|""));}
    }
    void refresh(){
        const auto& d=session_.snapshot();const bool ok=session_.connected();
        const bool settingsForm=page_==1||page_==2||(page_==5&&sub_==0);
        if(settingsForm&&!ok)formValid_=false;
        if(settingsForm&&ok&&!formValid_){render();return;}
        set(connection_,ok?"UART":"Нет связи");
        lv_obj_set_style_text_color(connection_,color(ok?accent():0xff6473),0);
        const bool indicators[]={ok&&(d["station"]["connected"]|false),ok&&(d["network_indicators"]["internet"]|false),ok};
        for(int i=0;i<3;++i)lv_obj_set_style_bg_color(statusDots_[i],color(indicators[i]?0x53d486:0x8297a1),0);
        String clock=ok?String(d["clock"]["datetime"]|"—"):String("—");set(clock_,"Дата и время: "+clock);
        if(page_==0){
            for(int i=0;i<4;++i){auto v=d["sound_alerts"]["channels"][i]["sound_on"];const bool on=ok&&!v.isNull()&&v.as<bool>();
                set(live_[i],!ok||v.isNull()?"Нет связи":on?"Вкл.":"Выкл.");
                lv_obj_set_style_bg_color(lv_obj_get_parent(live_[i]),color(on?(i==3?0x42232d:0x14382e):surface()),0);
                lv_obj_set_style_text_color(live_[i],color(on?(i==3?0xff6473:0x53d486):muted()),0);
                set(live_[4+i],measured(d["temperatures"][i]["c"]," °C",1));}
            auto rate=d["water"]["lpm"];set(live_[8],ok&&!rate.isNull()?String(rate.as<double>()*60,1)+" л/ч":"—");
            set(live_[9],measured(d["water"]["total_l_estimate"]," л",1));
            set(live_[10],measured(d["collection"]["ml_per_min"]," мл/мин",1));
            set(live_[11],"Собрано "+measured(d["collection"]["total_ml"]," мл",1));
        }else if(page_==1){for(int i=0;i<4;++i)set(live_[i],measured(d["temperatures"][i]["c"]," °C",1));}
        else if(page_==2){auto rate=d["water"]["lpm"];set(live_[0],ok&&!rate.isNull()?String(rate.as<double>()*60,1)+" л/ч":"—");
            set(live_[1],"Учтено: "+measured(d["water"]["total_l_estimate"]," л",2));set(live_[2],String("Импульсы: ")+measured(d["water"]["pulses"],"",0));
            set(live_[3],measured(d["collection"]["ml_per_min"]," мл/мин",1));
            set(live_[4],"Собрано: "+measured(d["collection"]["total_ml"]," мл",1));
            set(live_[5],ok&&(d["collection"]["sensor_present"]|false)?"Датчик подключён":"Нет датчика");}
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
    lv_obj_t* clock_=nullptr;lv_obj_t* statusDots_[3]={};lv_obj_t* graphRange_[5]={};
    lv_obj_t* navButtons_[6]={};lv_obj_t* fields_[8]={};lv_obj_t* live_[12]={};lv_obj_t* motion_[7]={};lv_chart_series_t* series_[4]={};
    lv_img_dsc_t art_{};uint8_t* asset_=nullptr;
    busa_touch::Transform transform_{};busa_touch::Point raw_[3]{},screen_[3]{};
    bool calibrationValid_=false,calibrating_=false,held_=false,wasConnected_=false,waiting_=false,scanPending_=false,frameOpen_=false,awaitRelease_=false,formValid_=false;
    int calIndex_=0,calSamples_=0,sumX_=0,sumY_=0,page_=0,sub_=-1,pendingPage_=0,pendingSub_=-1,graphKind_=0,graphMinutes_=5,calcMode_=0;
    uint32_t lastCalibrationRead_=0;
    uint8_t theme_=2,requestedTheme_=2;uint32_t lastTick_=0,lastAnimation_=0,revision_=0,lastScan_=0,frameStart_=0,lastGraph_=0,viewGeneration_=0,pendingGeneration_=0;
    Route pendingRoute_=Route::wifiScan;String networks_[32];unsigned networkCount_=0;String calcInput_[4][5];
};
constexpr uint32_t Interface::colors_[4];
}
