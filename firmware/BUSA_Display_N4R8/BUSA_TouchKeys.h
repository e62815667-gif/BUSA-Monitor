#pragma once
#include <lvgl.h>
#include <cstring>
namespace busa_touch_keys {
static const char* number[]={"1","2","3","Удалить","\n","4","5","6","Очистить","\n","7","8","9","←","\n","+/-","0",",","→",""};
static const char* lower[]={"q","w","e","r","t","y","u","i","o","p","Удалить","\n","a","s","d","f","g","h","j","k","l","Очистить","\n","ABC","z","x","c","v","b","n","m",".",",","\n","123","←","Пробел","→","-","_",""};
static const char* upper[]={"Q","W","E","R","T","Y","U","I","O","P","Удалить","\n","A","S","D","F","G","H","J","K","L","Очистить","\n","abc","Z","X","C","V","B","N","M",".",",","\n","123","←","Пробел","→","-","_",""};
static const char* special[]={"1","2","3","4","5","6","7","8","9","0","Удалить","\n","@","#","$","%","&","*","(",")","-","_","Очистить","\n","+","=","/","\\",":",";","!","?","'","\"","\n","[","]","{","}","<",">","|","~", "\x60","^","\n","abc","←","Пробел","→",".",",",""};
inline bool same(const char* a,const char* b){return strcmp(a,b)==0;}
inline void map(lv_obj_t* kb,lv_keyboard_mode_t mode,const char** keys,bool numeric){
    static lv_btnmatrix_ctrl_t controls[4][64]={};
    auto* c=controls[unsigned(mode)];
    unsigned n=0;
    for(unsigned i=0;keys[i][0];++i){
        if(same(keys[i],"\n"))continue;
        unsigned width=1;
        if(numeric&&(same(keys[i],"Удалить")||same(keys[i],"Очистить")||same(keys[i],"←")||same(keys[i],"→")))width=2;
        if(!numeric&&(same(keys[i],"Удалить")||same(keys[i],"Очистить")))width=3;
        if(!numeric&&same(keys[i],"Пробел"))width=5;
        c[n++]=lv_btnmatrix_ctrl_t(width|LV_BTNMATRIX_CTRL_NO_REPEAT|LV_BTNMATRIX_CTRL_CLICK_TRIG);
    }
    lv_keyboard_set_map(kb,mode,keys,c);
}
inline void configure(lv_obj_t* kb){
    lv_obj_remove_event_cb(kb,lv_keyboard_def_event_cb);
    lv_keyboard_set_popovers(kb,false);
    map(kb,LV_KEYBOARD_MODE_TEXT_LOWER,lower,false);
    map(kb,LV_KEYBOARD_MODE_TEXT_UPPER,upper,false);
    map(kb,LV_KEYBOARD_MODE_SPECIAL,special,false);
    map(kb,LV_KEYBOARD_MODE_NUMBER,number,true);
}
inline void apply(lv_obj_t* kb,lv_obj_t* editor,const char* key){
    if(!key||!editor)return;
    if(same(key,"ABC"))lv_keyboard_set_mode(kb,LV_KEYBOARD_MODE_TEXT_UPPER);
    else if(same(key,"abc"))lv_keyboard_set_mode(kb,LV_KEYBOARD_MODE_TEXT_LOWER);
    else if(same(key,"123"))lv_keyboard_set_mode(kb,LV_KEYBOARD_MODE_SPECIAL);
    else if(same(key,"Удалить"))lv_textarea_del_char(editor);
    else if(same(key,"Очистить"))lv_textarea_set_text(editor,"");
    else if(same(key,"←"))lv_textarea_cursor_left(editor);
    else if(same(key,"→"))lv_textarea_cursor_right(editor);
    else if(same(key,"Пробел"))lv_textarea_add_char(editor,' ');
    else if(same(key,"+/-")){
        const bool negative=lv_textarea_get_text(editor)[0]=='-';
        lv_textarea_set_cursor_pos(editor,negative?1:0);
        if(negative)lv_textarea_del_char(editor);else lv_textarea_add_char(editor,'-');
        lv_textarea_set_cursor_pos(editor,LV_TEXTAREA_CURSOR_LAST);
    }else if(same(key,",")&&lv_keyboard_get_mode(kb)==LV_KEYBOARD_MODE_NUMBER){
        const char* s=lv_textarea_get_text(editor);
        if(!strchr(s,',')&&!strchr(s,'.'))lv_textarea_add_char(editor,',');
    }else lv_textarea_add_text(editor,key);
}
}
