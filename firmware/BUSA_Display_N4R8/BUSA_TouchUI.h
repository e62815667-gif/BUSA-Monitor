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
namespace busa_guition {
class Interface {
public:
    Interface(Hardware& hw,DisplaySession& session):hw_(hw),session_(session){}
    bool begin(){
        if(!hw_.lcdReady())return false;
        lv_init();
        static_assert(LV_COLOR_DEPTH==16,"RGB565 required");
        auto* pixels=static_cast<lv_color_t*>(heap_caps_malloc(WIDTH*24*sizeof(lv_color_t),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
        if(!pixels)return false;
        lv_disp_draw_buf_init(&drawBuffer_,pixels,nullptr,WIDTH*24);
        lv_disp_drv_init(&driver_);driver_.hor_res=WIDTH;driver_.ver_res=HEIGHT;driver_.draw_buf=&drawBuffer_;
        driver_.flush_cb=flush;driver_.user_data=this;
        if(!lv_disp_drv_register(&driver_))return false;
        lv_indev_drv_init(&input_);input_.type=LV_INDEV_TYPE_POINTER;input_.read_cb=readTouch;input_.user_data=this;
        if(!lv_indev_drv_register(&input_))return false;
        prefs_.begin("busa-touch",false);
        calibrationValid_=prefs_.getBytesLength("affine")==sizeof(transform_)&&prefs_.getBytes("affine",&transform_,sizeof(transform_))==sizeof(transform_)&&validTransform();
        theme_=prefs_.getUChar("theme",2);if(theme_>2)theme_=2;
        asset_=static_cast<uint8_t*>(heap_caps_malloc(BUSA_ASSET_SIZE,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
        if(asset_){
            if(!hw_.readAsset("/busa/rig.rgb565a",asset_,BUSA_ASSET_SIZE))memcpy(asset_,BUSA_ASSET,BUSA_ASSET_SIZE);
            else if(memcmp(asset_,BUSA_ASSET,BUSA_ASSET_SIZE)!=0){hw_.invalidAsset();memcpy(asset_,BUSA_ASSET,BUSA_ASSET_SIZE);}
        }
        art_.header.always_zero=0;art_.header.cf=LV_IMG_CF_TRUE_COLOR_ALPHA;
        art_.header.w=BUSA_ASSET_W;art_.header.h=BUSA_ASSET_H;art_.data_size=BUSA_ASSET_SIZE;art_.data=asset_?asset_:BUSA_ASSET;
        lastTick_=millis();
        if(calibrationValid_)build();else startCalibration();
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
            if(page_==0&&now-lastAnimation_>=80){lastAnimation_=now;animate(now);}
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
    uint32_t paper()const{return theme_==0?0xf3f7fa:0x061521;}
    uint32_t surface()const{return theme_==0?0xffffff:0x0c2132;}
    uint32_t line()const{return theme_==0?0xc9dce6:0x244253;}
    static void flush(lv_disp_drv_t* d,const lv_area_t* a,lv_color_t* p){
        auto* self=static_cast<Interface*>(d->user_data);
        if(!self->frameOpen_){self->frameStart_=millis();self->frameOpen_=true;}
        self->hw_.panel().draw16bitRGBBitmap(a->x1,a->y1,reinterpret_cast<uint16_t*>(p),a->x2-a->x1+1,a->y2-a->y1+1);
        if(lv_disp_flush_is_last(d)){self->hw_.frameComplete(self->frameStart_);self->frameOpen_=false;}
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
    void rootStyle(lv_obj_t* o){lv_obj_set_style_bg_color(o,color(paper()),0);lv_obj_set_style_text_color(o,color(ink()),0);lv_obj_set_style_text_font(o,&busa_font,0);}
    void startCalibration(){
        calibrating_=true;calibrationValid_=false;calIndex_=0;held_=false;awaitRelease_=true;calSamples_=0;sumX_=sumY_=0;
        auto* root=lv_scr_act();lv_obj_clean(root);rootStyle(root);
        auto* msg=lv_label_create(root);lv_label_set_text(msg,"Калибровка: нажмите и отпустите метку");lv_obj_align(msg,LV_ALIGN_CENTER,0,0);
        calMark_=lv_obj_create(root);lv_obj_remove_style_all(calMark_);lv_obj_set_size(calMark_,18,18);
        lv_obj_set_style_radius(calMark_,LV_RADIUS_CIRCLE,0);lv_obj_set_style_bg_color(calMark_,color(0x2087ed),0);lv_obj_set_style_bg_opa(calMark_,LV_OPA_COVER,0);placeMark();
    }
    busa_touch::Point target()const{static const busa_touch::Point p[]={{20,20},{460,20},{20,252},{460,252}};return p[calIndex_];}
    void placeMark(){auto p=target();lv_obj_set_pos(calMark_,p.x-9,p.y-9);}
    void calibrationPoll(){
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
        calibrationValid_=prefs_.putBytes("affine",&transform_,sizeof(transform_))==sizeof(transform_);
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
        lv_obj_set_height(self->body_,HEIGHT-28-43-112);
        lv_obj_scroll_to_view(ta,LV_ANIM_OFF);
    }
    static void keyboardEvent(lv_event_t* e){
        auto* self=static_cast<Interface*>(lv_event_get_user_data(e));
        if(lv_event_get_code(e)==LV_EVENT_READY||lv_event_get_code(e)==LV_EVENT_CANCEL)self->hideKeyboard();
    }
    void hideKeyboard(){if(!keyboard_)return;lv_keyboard_set_textarea(keyboard_,nullptr);lv_obj_add_flag(keyboard_,LV_OBJ_FLAG_HIDDEN);lv_obj_set_height(body_,HEIGHT-28-43);}
    lv_obj_t* label(lv_obj_t* parent,const char* text){
        auto* o=lv_label_create(parent);lv_obj_set_style_text_font(o,&busa_font,0);lv_label_set_text(o,text);return o;
    }
    lv_obj_t* button(lv_obj_t* parent,const char* text,int id,int width=LV_PCT(100)){
        auto* o=lv_btn_create(parent);lv_obj_set_size(o,width,34);lv_obj_set_style_radius(o,8,0);
        lv_obj_set_style_bg_color(o,color(0x2087ed),0);lv_obj_set_style_shadow_width(o,0,0);
        auto* l=label(o,text);lv_obj_set_style_text_color(l,color(0xffffff),0);lv_obj_center(l);
        lv_obj_set_user_data(o,reinterpret_cast<void*>(intptr_t(id)));lv_obj_add_event_cb(o,event,LV_EVENT_CLICKED,this);return o;
    }
    lv_obj_t* panel(lv_obj_t* parent,int width=LV_PCT(100),int height=LV_SIZE_CONTENT){
        auto* o=lv_obj_create(parent);lv_obj_set_size(o,width,height);lv_obj_set_style_bg_color(o,color(surface()),0);
        lv_obj_set_style_border_color(o,color(line()),0);lv_obj_set_style_border_width(o,1,0);
        lv_obj_set_style_radius(o,9,0);lv_obj_set_style_pad_all(o,6,0);lv_obj_set_style_pad_row(o,5,0);
        lv_obj_set_flex_flow(o,LV_FLEX_FLOW_COLUMN);lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE);return o;
    }
    lv_obj_t* row(lv_obj_t* parent,int height=LV_SIZE_CONTENT){
        auto* o=lv_obj_create(parent);lv_obj_remove_style_all(o);lv_obj_set_size(o,LV_PCT(100),height);
        lv_obj_set_flex_flow(o,LV_FLEX_FLOW_ROW);lv_obj_set_flex_align(o,LV_FLEX_ALIGN_SPACE_BETWEEN,LV_FLEX_ALIGN_CENTER,LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_column(o,4,0);return o;
    }
    lv_obj_t* field(lv_obj_t* parent,const char* title,const String& value,bool numeric,bool secret=false,int width=LV_PCT(100)){
        auto* box=panel(parent,width);label(box,title);
        auto* ta=lv_textarea_create(box);lv_obj_set_width(ta,LV_PCT(100));lv_obj_set_height(ta,34);
        lv_obj_set_style_text_font(ta,&busa_font,0);lv_obj_set_style_pad_all(ta,5,0);lv_textarea_set_one_line(ta,true);
        lv_textarea_set_max_length(ta,numeric?14:220);
        if(numeric)lv_textarea_set_accepted_chars(ta,"0123456789.-,");
        lv_textarea_set_password_mode(ta,secret);lv_textarea_set_text(ta,value.c_str());
        lv_obj_add_event_cb(ta,textFocus,LV_EVENT_CLICKED,this);return ta;
    }
    static String text(lv_obj_t* ta){String s=lv_textarea_get_text(ta);s.replace(',','.');return s;}
    static String value(JsonVariantConst v,int digits=1){
        if(v.isNull()||!v.is<double>())return "";
        return String(v.as<double>(),digits);
    }
    String measured(JsonVariantConst v,const char* unit,int digits=1)const{
        String s=session_.connected()?value(v,digits):String("");
        return s.isEmpty()?String("—"):s+unit;
    }
    void set(lv_obj_t* o,const String& s){if(o&&strcmp(lv_label_get_text(o),s.c_str()))lv_label_set_text(o,s.c_str());}
    void build(){
        auto* root=lv_scr_act();lv_obj_clean(root);rootStyle(root);lv_obj_clear_flag(root,LV_OBJ_FLAG_SCROLLABLE);
        auto* top=lv_obj_create(root);lv_obj_remove_style_all(top);lv_obj_set_size(top,WIDTH,28);lv_obj_set_pos(top,0,0);
        auto* title=label(top,"BUSA MONITOR");lv_obj_set_pos(title,8,5);
        connection_=label(top,"UART —");lv_obj_align(connection_,LV_ALIGN_RIGHT_MID,-8,0);
        body_=lv_obj_create(root);lv_obj_set_pos(body_,0,28);lv_obj_set_size(body_,WIDTH,HEIGHT-28-43);
        lv_obj_set_style_bg_color(body_,color(paper()),0);lv_obj_set_style_border_width(body_,0,0);lv_obj_set_style_radius(body_,0,0);
        lv_obj_set_style_pad_all(body_,5,0);lv_obj_set_style_pad_row(body_,5,0);lv_obj_set_flex_flow(body_,LV_FLEX_FLOW_COLUMN);
        lv_obj_set_scroll_dir(body_,LV_DIR_VER);
        nav_=lv_obj_create(root);lv_obj_remove_style_all(nav_);lv_obj_set_size(nav_,WIDTH,43);lv_obj_set_pos(nav_,0,HEIGHT-43);
        lv_obj_set_style_bg_color(nav_,color(paper()),0);lv_obj_set_style_bg_opa(nav_,LV_OPA_COVER,0);
        lv_obj_set_style_pad_all(nav_,3,0);lv_obj_set_style_pad_column(nav_,2,0);lv_obj_set_flex_flow(nav_,LV_FLEX_FLOW_ROW);
        static const char* titles[]={"Главная","Темп.","Вода","Графики","Расчёт","Настройки"};
        for(int i=0;i<6;++i){auto* b=button(nav_,titles[i],100+i,76);lv_obj_set_height(b,36);navButtons_[i]=b;}
        keyboard_=lv_keyboard_create(root);lv_obj_set_size(keyboard_,WIDTH,155);lv_obj_align(keyboard_,LV_ALIGN_BOTTOM_MID,0,0);
        lv_obj_add_flag(keyboard_,LV_OBJ_FLAG_HIDDEN);lv_obj_add_event_cb(keyboard_,keyboardEvent,LV_EVENT_ALL,this);
        render();
    }
    void render(){
        hideKeyboard();lv_obj_clean(body_);++viewGeneration_;formValid_=session_.connected();for(auto& p:fields_)p=nullptr;for(auto& p:live_)p=nullptr;for(auto& p:motion_)p=nullptr;
        wifiList_=nullptr;chart_=nullptr;resultLabel_=nullptr;pageStatus_=nullptr;scanPending_=false;
        for(int i=0;i<6;++i){lv_obj_set_style_bg_color(navButtons_[i],color(i==page_?0x2087ed:surface()),0);lv_obj_set_style_text_color(lv_obj_get_child(navButtons_[i],0),color(i==page_?0xffffff:ink()),0);}
        switch(page_){case 0:home();break;case 1:temperatures();break;case 2:water();break;case 3:graphs();break;case 4:calculator();break;case 5:settings();break;}
        refresh();
    }
    void home(){
        auto* bar=row(body_,32);
        for(int i=0;i<4;++i){auto* p=panel(bar,112,30);live_[i]=label(p,"К —");}
        auto* middle=row(body_,144);auto* left=panel(middle,151,144);auto* center=lv_obj_create(middle);
        lv_obj_remove_style_all(center);lv_obj_set_size(center,BUSA_ASSET_W,BUSA_ASSET_H);lv_obj_clear_flag(center,LV_OBJ_FLAG_SCROLLABLE);
        auto* im=lv_img_create(center);lv_img_set_src(im,&art_);lv_obj_center(im);
        for(unsigned i=0;i<7;++i){auto* dot=lv_obj_create(center);lv_obj_remove_style_all(dot);lv_obj_set_size(dot,3,3);
            lv_obj_set_style_bg_color(dot,color(i<4?0x38bdf8:0xc5fffa),0);lv_obj_set_style_bg_opa(dot,LV_OPA_COVER,0);lv_obj_set_style_radius(dot,LV_RADIUS_CIRCLE,0);motion_[i]=dot;}
        auto* right=panel(middle,151,144);
        for(int i=0;i<4;++i){auto* p=i%2?right:left;auto* name=label(p,sensor(i));lv_obj_set_style_text_color(name,color(colors_[i]),0);live_[4+i]=label(p,"—");}
        auto* r=row(body_);live_[8]=label(r,"Расход —");live_[9]=label(r,"Вода —");
    }
    void temperatures(){
        const auto& doc=session_.snapshot();label(body_,"ТЕМПЕРАТУРЫ · пороги звука");
        for(int i=0;i<4;++i){auto* r=row(body_);auto* p=panel(r,130);label(p,sensor(i));live_[i]=label(p,"—");
            fields_[2*i]=field(r,"ВКЛ, °C",value(doc["display_references"][i]["target"]),true,false,155);
            fields_[2*i+1]=field(r,"ВЫКЛ, °C",value(doc["display_references"][i]["notice"]),true,false,155);}
        button(body_,"Сохранить пороги",200);pageStatus_=label(body_,"");
    }
    void water(){
        label(body_,"ВОДА И ОТБОР");
        auto* p=panel(body_);label(p,"Охлаждение");live_[0]=label(p,"—");live_[1]=label(p,"—");live_[2]=label(p,"—");
        const auto& d=session_.snapshot();
        fields_[0]=field(body_,"Коэффициент, имп/л",value(d["water"]["pulses_per_litre"]),true);
        button(body_,"Сохранить коэффициент",201);
        auto* q=panel(body_);label(q,"Отбор жидкости");live_[3]=label(q,"Нет датчика");pageStatus_=label(body_,"");
    }
    void graphs(){
        label(body_,"ГРАФИКИ");auto* r=row(body_);
        button(r,"Температуры",300,146);button(r,"Вода",301,146);button(r,"Отбор",302,146);
        auto* ranges=row(body_);
        for(int i=0;i<5;++i){static const char* names[]={"5 мин","10 мин","15 мин","30 мин","60 мин"};button(ranges,names[i],310+i,88);}
        chart_=lv_chart_create(body_);lv_obj_set_size(chart_,LV_PCT(100),105);lv_chart_set_type(chart_,LV_CHART_TYPE_LINE);
        lv_chart_set_point_count(chart_,160);lv_obj_set_style_bg_color(chart_,color(surface()),0);lv_chart_set_range(chart_,LV_CHART_AXIS_PRIMARY_Y,0,1250);
        for(int i=0;i<4;++i)series_[i]=lv_chart_add_series(chart_,color(colors_[i]),LV_CHART_AXIS_PRIMARY_Y);
        pageStatus_=label(body_,"Выберите период");requestGraph();
    }
    void requestGraph(){lastGraph_=millis();send(Route::chartHistory,{{"minutes",String(graphMinutes_)}});}
    void calculator(){
        static const char* names[]={"Разбавление","Ареометр","Расчёт выхода","Объём куба"};
        static const char* labels[4][5]={{"Объём, л","Исходная, %","Нужная, %","",""},{"Температура, °C","Показание, %","","",""},{"Объём, л","Исходная, %","На выходе, %","Доля 1, %","Доля 2, %"},{"Диаметр, см","Высота, см","Уровень, см","",""}};
        static const int counts[]={3,2,5,3};
        auto* r=row(body_);for(int i=0;i<4;++i)button(r,names[i],320+i,112);
        label(body_,names[calcMode_]);for(int i=0;i<counts[calcMode_];++i)fields_[i]=field(body_,labels[calcMode_][i],calcInput_[calcMode_][i],true);
        button(body_,"Рассчитать",330);if(calcMode_==3){button(body_,"Загрузить размеры",331);button(body_,"Сохранить размеры",332);}resultLabel_=label(body_,"");lv_obj_set_width(resultLabel_,LV_PCT(100));lv_label_set_long_mode(resultLabel_,LV_LABEL_LONG_WRAP);
    }
    void settings(){
        label(body_,"НАСТРОЙКИ");
        if(sub_<0){static const char* names[]={"Калибровка температуры","Wi-Fi","Экран","Google","О системе","Журнал событий","Диагностика","Bluetooth"};
            for(int i=0;i<8;++i)button(body_,names[i],400+i);return;}
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
        if(id>=400&&id<408){sub_=id-400;render();return;}
        if(id==409){sub_=-1;render();return;}
        if(id>=300&&id<=302){graphKind_=id-300;render();return;}
        if(id>=310&&id<=314){static const int minutes[]={5,10,15,30,60};graphMinutes_=minutes[id-310];requestGraph();return;}
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
        set(connection_,ok?String("UART · ") + ((d["wifi"]["sta_connected"]|false)?"Wi-Fi":"Локально"):"UART · нет связи");
        if(page_==0){
            for(int i=0;i<4;++i){auto v=d["sound_alerts"]["channels"][i]["sound_on"];set(live_[i],String("К")+String(i+1)+"  "+(!ok||v.isNull()?"—":v.as<bool>()?"Звук":"Тихо"));
                set(live_[4+i],measured(d["temperatures"][i]["c"]," °C",2));}
            auto rate=d["water"]["lpm"];set(live_[8],String("Расход ")+(ok&&!rate.isNull()?String(rate.as<double>()*60,1)+" л/ч":"—"));
            set(live_[9],"Вода "+measured(d["water"]["total_l_estimate"]," л",2));
        }else if(page_==1){for(int i=0;i<4;++i)set(live_[i],measured(d["temperatures"][i]["c"]," °C",2));}
        else if(page_==2){auto rate=d["water"]["lpm"];set(live_[0],String("Расход: ")+(ok&&!rate.isNull()?String(rate.as<double>()*60,1)+" л/ч":"—"));
            set(live_[1],"Израсходовано: "+measured(d["water"]["total_l_estimate"]," л",2));set(live_[2],String("Импульсы: ")+measured(d["water"]["pulses"],"",0));
            set(live_[3],String("Отбор: ")+measured(d["collection"]["ml_per_min"]," мл/мин",1)+" · "+measured(d["collection"]["total_ml"]," мл",1));}
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
            if(!active){lv_obj_add_flag(motion_[i],LV_OBJ_FLAG_HIDDEN);continue;}lv_obj_clear_flag(motion_[i],LV_OBJ_FLAG_HIDDEN);
            int t=(now/40+i*23)%100,x=0,y=0;
            if(i<2){x=142;y=1144-t*8;}else if(i==2){x=684+t*2;y=367;}else if(i==3){x=788+t*2;y=913;}
            else{x=300+(i-4)*90;y=1420-t;}
            lv_obj_set_pos(motion_[i],x*BUSA_ASSET_W/1024,y*BUSA_ASSET_H/1536);}
    }
    Hardware& hw_;DisplaySession& session_;Preferences prefs_;
    lv_disp_drv_t driver_{};lv_disp_draw_buf_t drawBuffer_{};lv_indev_drv_t input_{};
    lv_obj_t *connection_=nullptr,*body_=nullptr,*nav_=nullptr,*keyboard_=nullptr,*pageStatus_=nullptr,*resultLabel_=nullptr,*wifiList_=nullptr,*chart_=nullptr,*calMark_=nullptr;
    lv_obj_t* navButtons_[6]={};lv_obj_t* fields_[8]={};lv_obj_t* live_[12]={};lv_obj_t* motion_[7]={};lv_chart_series_t* series_[4]={};
    lv_img_dsc_t art_{};uint8_t* asset_=nullptr;
    busa_touch::Transform transform_{};busa_touch::Point raw_[3]{},screen_[3]{};
    bool calibrationValid_=false,calibrating_=false,held_=false,wasConnected_=false,waiting_=false,scanPending_=false,frameOpen_=false,awaitRelease_=false,formValid_=false;
    int calIndex_=0,calSamples_=0,sumX_=0,sumY_=0,page_=0,sub_=-1,pendingPage_=0,pendingSub_=-1,graphKind_=0,graphMinutes_=5,calcMode_=0;
    uint8_t theme_=2,requestedTheme_=2;uint32_t lastTick_=0,lastAnimation_=0,revision_=0,lastScan_=0,frameStart_=0,lastGraph_=0,viewGeneration_=0,pendingGeneration_=0;
    Route pendingRoute_=Route::wifiScan;String networks_[32];unsigned networkCount_=0;String calcInput_[4][5];
};
constexpr uint32_t Interface::colors_[4];
}
