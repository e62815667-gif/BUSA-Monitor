#pragma once
// БУСА-1М №001 — V14.7: flicker-free TFT, monochrome dark theme, Internet / local Wi-Fi LED.
// THIS CODE ONLY DRAWS DATA. NO RELAY/POWER/PWM CONTROL.
// No placeholder readings: water flow raw pulses only until calibrated.
// Google transport is implemented later in BUSA_Google.h; TFT displays status only.
extern volatile bool googleEnabled;
extern volatile int googleState;
extern volatile int googleInternet;
static const uint8_t v14Order[6]={0,1,2,5,3,4};
static const char* const v14Tabs[6]={"Глав","Т","Отбор","Настр","Граф","Событ"};
// V14 palette is software-defined. DO NOT use ST7789 invertDisplay(): it
// inverts every accent and destroys the colored illustration.
static uint16_t V14_TEXT=0x2104,V14_MUTED=0x632c,V14_WARN=0xa800,
 V14_GREEN=0x2644,V14_BLUE=0x0338,V14_ORANGE=0xfc60,V14_SEA=0x0578,
 V14_CARD=0xffff,V14_PALE=0xffdf,V14_RULE=0xd69a;
static uint16_t v14SeriesColors[4]={0x037b,0xd980,0xb500,0x3685};
static uint16_t V14_INFO=0xff16,V14_GRID=0xe71c,V14_BADGE=0xe7f8;
static uint32_t v14FlowLast=UINT32_MAX;
static String v14ClockLast="";
static int8_t v14NetLast=-1,v14LocalLast=-1;
struct V14MetricCache{int16_t x=-1,y=-1,width=-1;String value;uint16_t fg=0,bg=0;};
static V14MetricCache v14Metrics[10];
static uint8_t v14MetricCount=0;
static void v14InvalidateLive(){v14MetricCount=0;v14FlowLast=UINT32_MAX;v14ClockLast="";v14NetLast=-1;v14LocalLast=-1;}
static void v14ApplyPalette(bool dark){
 HQ_MONO_DARK=dark;
 if(dark){
   // Pure BLACK background, WHITE text: matches "BUSA 1M AI" boot splash.
   // Only gray contours and monochrome illustrations (no software inversion).
   HQ_NAVY=0x0000;HQ_DEEP=0x0000;HQ_GLOW=0xffff;
   HQ_AZURE=0xbdf7;HQ_SKY=0xffff;HQ_CYAN=0xe71c;HQ_LIGHT=0xffff;
   HQ_LIME=0xd69a;HQ_AQUA=0xbdf7;HQ_GLASS=0x4208;HQ_STEEL=0x8410;
   HQ_SILVER=0xce59;HQ_METAL=0x9cf3;HQ_DARKMETAL=0x4208;
   V14_TEXT=0xffff;V14_MUTED=0xbdf7;V14_WARN=0xffff;
   V14_GREEN=0xffff;V14_BLUE=0xffff;V14_ORANGE=0xffff;
   V14_SEA=0xbdf7;V14_CARD=0x0000;V14_PALE=0x0000;V14_RULE=0x8410;
   V14_INFO=0x0000;V14_GRID=0x4208;V14_BADGE=0x4208;
   v14SeriesColors[0]=0xffff;v14SeriesColors[1]=0xd69a;
   v14SeriesColors[2]=0xbdf7;v14SeriesColors[3]=0x9cf3;
 }else{
   HQ_NAVY=0xffdf;HQ_DEEP=0xffff;HQ_GLOW=0xfca0;
   HQ_AZURE=0xd4a5;HQ_SKY=0x234c;HQ_CYAN=0x0519;HQ_LIGHT=0x2104;
   HQ_LIME=0x2664;HQ_AQUA=0x767c;HQ_GLASS=0xf638;HQ_STEEL=0xad55;
   HQ_SILVER=0xe73c;HQ_METAL=0xb5b6;HQ_DARKMETAL=0x6b4d;
   V14_TEXT=0x2104;V14_MUTED=0x632c;V14_WARN=0xa800;
   V14_GREEN=0x2644;V14_BLUE=0x0338;V14_ORANGE=0xfc60;
   V14_SEA=0x0578;V14_CARD=0xffff;V14_PALE=0xffdf;V14_RULE=0xd69a;
   V14_INFO=0xff16;V14_GRID=0xe71c;V14_BADGE=0xe7f8;
   v14SeriesColors[0]=0x037b;v14SeriesColors[1]=0xd980;
   v14SeriesColors[2]=0xb500;v14SeriesColors[3]=0x3685;
 }
}
static void v14Print(const String& txt,int x,int y,uint16_t col=V14_TEXT){
  ru.setFont(u8g2_font_6x13_t_cyrillic);
  ruText(txt,x,y,col);
}
static void v14Tiny(const String& txt,int x,int y,uint16_t col=V14_TEXT){
  ru.setFont(u8g2_font_5x7_t_cyrillic);
  ruText(txt,x,y,col);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
}
static void v14Big(const String& txt,int x,int y,uint16_t col=V14_TEXT){
  ru.setFont(u8g2_font_7x13_t_cyrillic);
  ruText(txt,x,y,col);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
}
static void v14Metric(const String& txt,int x,int y,int width,bool valid){
  // Update only CHANGED glyphs. Never erase entire frame for live telemetry.
  uint16_t color=valid?V14_GREEN:V14_WARN;
  for(uint8_t i=0;i<v14MetricCount;i++){
    V14MetricCache& m=v14Metrics[i];
    if(m.x==x && m.y==y && m.width==width){
      if(m.value==txt && m.fg==color && m.bg==V14_CARD)return;
      m.value=txt;m.fg=color;m.bg=V14_CARD;
      screen->fillRect(x,y-13,width,16,V14_CARD);
      v14Big(txt,x,y,color);return;
    }
  }
  if(v14MetricCount<10){
    V14MetricCache& m=v14Metrics[v14MetricCount++];
    m.x=x;m.y=y;m.width=width;m.value=txt;m.fg=color;m.bg=V14_CARD;
  }
  screen->fillRect(x,y-13,width,16,V14_CARD);
  v14Big(txt,x,y,color);
}
static void v14FootArrow(int x,int direction,uint16_t color){
  // All four icons have identical centre and stroke proportions.
  const int cx=x+8,cy=225,r=5;
  if(direction==0){screen->fillTriangle(cx-r,cy,cx+3,cy-r,cx+3,cy+r,color);}
  if(direction==1){screen->fillTriangle(cx,cy-r,cx-r,cy+3,cx+r,cy+3,color);}
  if(direction==2){screen->fillTriangle(cx,cy+r,cx-r,cy-3,cx+r,cy-3,color);}
  if(direction==3){screen->fillTriangle(cx+r,cy,cx-3,cy-r,cx-3,cy+r,color);}
}
// Independent status dots: Internet external TCP/443 probe, local blue AP client.
// Red means not confirmed. TFT settings show STA name and AP address in detail.
static void v14NetIndicators(){
  const int8_t net=(WiFi.status()==WL_CONNECTED && googleInternet==1)?1:0;
  const int8_t local=(WiFi.softAPgetStationNum()>0)?1:0;
  if(v14NetLast==net && v14LocalLast==local)return;
  v14NetLast=net;v14LocalLast=local;
  screen->fillRect(121,2,98,17,V14_CARD);
  screen->fillCircle(130,11,4,net?0x07e0:0xf800);
  v14Tiny("ИНТ",137,14,V14_TEXT);
  screen->fillCircle(179,11,4,local?0x001f:0xf800);
  v14Tiny("ЛОК",186,14,V14_TEXT);
}
static void v14Frame(uint32_t now){
  hqBackground();
  screen->fillRect(0,0,320,21,V14_CARD);
  screen->drawFastHLine(0,21,320,V14_ORANGE);
  v14Big("БУСА-1М №001",5,15,V14_TEXT);
  v14NetIndicators();
  // DD.MM.YY HH:MM. Exact pixel width is measured; date cannot clip at 320.
  String dt=deviceClockValid()?clockStamp(true):String("--.--.-- --:--");
  if(dt.length()==16)dt=dt.substring(0,6)+dt.substring(8);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
  int tw=ru.getUTF8Width(dt.c_str());
  v14Print(dt,max(220,317-tw),15,V14_TEXT);
  // Six tabs, preserving internal IDs with the explicit approved visual order.
  for(int i=0;i<6;i++){
    int x=1+i*53;bool chosen=(v14Order[i]==currentPage);
    screen->fillRoundRect(x,25,51,22,4,chosen?V14_ORANGE:V14_CARD);
    screen->drawRoundRect(x,25,51,22,4,chosen?V14_ORANGE:V14_RULE);
    if(chosen)screen->drawFastHLine(x+6,45,39,V14_WARN);
    String title=v14Tabs[i];
    int w=ru.getUTF8Width(title.c_str());
    v14Print(title,x+max(2,(51-w)/2),41,V14_TEXT);
  }
  // Footer always visible, no truncation, equal arrow centres.
  screen->fillRect(0,211,320,29,V14_CARD);
  screen->drawFastHLine(0,211,320,V14_ORANGE);
  static const char* labels[]={"Влево","Вверх","Вниз","Вправо","OK","Назад"};
  for(int i=0;i<6;i++){
    const int x=1+i*53;
    screen->fillRoundRect(x,215,51,22,4,i==4?V14_BADGE:V14_CARD);
    screen->drawRoundRect(x,215,51,22,4,V14_RULE);
    if(i<4)v14FootArrow(x,i,V14_ORANGE);
    else if(i==4){screen->drawCircle(x+10,225,5,V14_GREEN);screen->fillCircle(x+10,225,2,V14_GREEN);}
    else{screen->drawFastHLine(x+4,225,11,V14_BLUE);screen->fillTriangle(x+4,225,x+9,221,x+9,229,V14_BLUE);}
    v14Print(labels[i],x+14,229,V14_TEXT);
  }
}
static void v14MiniRig(){
  // Approved stylized installation; YF-S401 is on the cooling-water line.
  hqRig(123,52,113,142);
  v14Tiny("ВОДА YF",176,188,V14_BLUE);
}

static void v14DrawFlowBar(){
  const uint32_t pulses=flowPulses();
  if(v14FlowLast==pulses)return;
  v14FlowLast=pulses;
  screen->fillRoundRect(4,198,313,11,3,V14_INFO);
  v14Tiny(String("YF-S401: ")+String(pulses)+
     " имп. (не калибр.)    K1-K4: БЛОК",8,206,V14_TEXT);
}
static void v14Home(uint32_t now,bool layout){
  static const char* names[]={"Вода","Колонна","Куб","Верх колонны"};
  v14MiniRig();
  for(int i=0;i<4;i++){
    int y=52+i*36;
    hqPanel(3,y,116,34);
    screen->fillCircle(12,y+12,4,v14SeriesColors[i]);
    v14Print(names[i],22,y+16,V14_TEXT);
    v14Metric(v13Temp(i,now),21,y+30,94,hasTemp(i,now));
  }
  hqPanel(240,52,77,65);
  v14Tiny("ОТБОР",247,65,V14_TEXT);
  v14Tiny("мл/мин",247,106,V14_MUTED);
  v14Metric("Н/Д",247,91,65,false);
  hqPanel(240,123,77,71);
  v14Tiny("СОБРАНО",247,136,V14_TEXT);
  v14Tiny("мл",247,182,V14_MUTED);
  v14Metric("Н/Д",247,163,64,false);
  v14FlowLast=UINT32_MAX; // always draw once when page is rebuilt
  v14DrawFlowBar();
}
static void v14Temperature(uint32_t now,bool layout){
  static const char* names[]={"Вода","Колонна","Куб","Верх колонны"};
  for(int i=0;i<4;i++){
    int x=4+(i%2)*158,y=52+(i/2)*77;
    hqPanel(x,y,153,73);
    if(i==selectedTemp)screen->fillRoundRect(x+3,y+3,4,65,2,V14_ORANGE);
    screen->fillCircle(x+14,y+15,4,v14SeriesColors[i]);
    v14Print(names[i],x+25,y+18,V14_TEXT);
    v14Metric(v13Temp(i,now),x+11,y+40,133,hasTemp(i,now));
    v14Tiny(String("Зад.: ")+v13Ref(refTemp[i]),x+12,y+54,V14_MUTED);
    v14Tiny(String("Пред.: ")+v13Ref(noticeTemp[i]),x+12,y+66,V14_MUTED);
  }
  if(editingTemp){
    hqPanel(43,73,234,114);
    v14Big(String("Редактирование T")+String(selectedTemp+1),53,97,V14_TEXT);
    v14Metric(editTempBuffer.isEmpty()?"--":editTempBuffer,55,128,209,true);
    v14Print("ENT сохранить",53,150,V14_GREEN);
    v14Print("ESC отменить",53,171,V14_MUTED);
  }
}
static void v14Collection(bool layout){
  hqPanel(5,52,151,62);
  v14Print("Расход отбора",13,70,V14_TEXT);
  v14Metric("НЕТ ДАТЧИКА",13,98,137,false);
  hqPanel(162,52,153,62);
  v14Print("Собрано, мл",170,70,V14_TEXT);
  v14Metric("Н/Д",170,98,138,false);
  hqPanel(5,121,310,86);
  v14Print("ГРАФИК ОТБОРА",15,140,V14_TEXT);
  screen->drawFastHLine(27,191,271,V14_RULE);
  screen->drawFastVLine(27,150,42,V14_RULE);
  v14Print("Нет датчика отбора",69,176,V14_WARN);
  v14Tiny("YF-S401 относится только к охлаждающей воде",13,199,V14_MUTED);
}
static void v14GraphTabs(){
  static const char* names[]={"ТЕМПЕРАТУРЫ","ВОДА","ОТБОР"};
  for(int j=0;j<3;j++){
    int x=5+j*104;
    screen->fillRoundRect(x,52,100,22,4,graphSubView==j?V14_ORANGE:V14_CARD);
    screen->drawRoundRect(x,52,100,22,4,V14_RULE);
    const String name=names[j];
    ru.setFont(u8g2_font_6x13_t_cyrillic);
    int w=ru.getUTF8Width(name.c_str());
    v14Print(name,x+max(2,(100-w)/2),67,V14_TEXT);
  }
}
static void v14Graphs(bool layout){
  v14GraphTabs();
  hqPanel(5,79,310,128);
  if(graphSubView==2){
    v14Print("График отбора",16,97,V14_TEXT);
    screen->drawFastHLine(29,189,270,V14_RULE);
    screen->drawFastVLine(29,109,80,V14_RULE);
    v14Print("НЕТ ДАТЧИКА ОТБОРА",61,155,V14_WARN);
    return;
  }
  const int gx=31, gy=107,gw=265,gh=77;
  screen->drawFastVLine(gx,gy,gh,V14_RULE);
  screen->drawFastHLine(gx,gy+gh,gw,V14_RULE);
  for(int j=1;j<4;j++)screen->drawFastHLine(gx,gy+j*gh/4,gw,0xe71c);
  if(graphSubView==0){
    v14Tiny("T1",16,96,v14SeriesColors[0]);
    v14Tiny("T2",57,96,v14SeriesColors[1]);
    v14Tiny("T3",98,96,v14SeriesColors[2]);
    v14Tiny("T4",139,96,v14SeriesColors[3]);
    v14Tiny("°C",288,96,V14_MUTED);
    v14Tiny("125",8,111,V14_MUTED);
    v14Tiny("0",17,185,V14_MUTED);
    bool any=false;
    for(int k=0;k<4;k++){
      for(int i=1;i<v13HistoryCount;i++){
        int a=(v13HistoryHead+48-v13HistoryCount+i-1)%48;
        int b=(v13HistoryHead+48-v13HistoryCount+i)%48;
        float va=v13History[k][a],vb=v13History[k][b];
        if(!isfinite(va)||!isfinite(vb))continue;
        any=true;
        int x1=gx+(i-1)*(gw-2)/47,x2=gx+i*(gw-2)/47;
        int y1=gy+gh-(int)(fminf(fmaxf(va,0.0f),125.0f)*gh/125.0f);
        int y2=gy+gh-(int)(fminf(fmaxf(vb,0.0f),125.0f)*gh/125.0f);
        screen->drawLine(x1,y1,x2,y2,v14SeriesColors[k]);
      }
    }
    if(!any)v14Print("Нет измерений T1–T4",69,151,V14_WARN);
  }else{
    v14Tiny("YF-S401, импульсы за интервал ~2 с",14,96,V14_TEXT);
    uint32_t maxv=1;bool has=false;
    for(int i=0;i<40;i++){
      if(waterPulseHistory[i]>maxv)maxv=waterPulseHistory[i];
      if(waterPulseHistory[i])has=true;
    }
    for(int i=0;i<40;i++){
      int p=(flowTraceHead+i)%40;
      int x=gx+2+i*6;
      int h=(int)((uint64_t)waterPulseHistory[p]*(gh-3)/maxv);
      if(h>0)screen->fillRect(x,gy+gh-h,4,h,V14_BLUE);
    }
    v14Tiny(String("0–")+String(maxv)+" имп.",8,198,V14_MUTED);
    v14Tiny(String("Последний интервал: ")+String(waterPulseDelta)+" имп.",151,198,V14_TEXT);
    if(!has)v14Print("Импульсы не зарегистрированы",38,149,V14_WARN);
  }
  v14Tiny("История текущего включения, без калибровки",33,205,V14_MUTED);
}
static void v14Events(bool layout){
  hqPanel(5,52,310,155);
  v14Print("ЖУРНАЛ СОБЫТИЙ",14,70,V14_TEXT);
  screen->drawFastHLine(14,76,290,V14_RULE);
  int count=min((int)v13EventCount,5);
  for(int i=0;i<count;i++){
    int y=96+i*21;
    screen->fillCircle(19,y-4,3,V14_ORANGE);
    // Never cut UTF-8 at arbitrary bytes; small font clips long labels at panel width.
    v14Tiny(v13Events[i],29,y,V14_TEXT);
    if(i<4)screen->drawFastHLine(14,y+5,289,0xe71c);
  }
}
static void v14Settings(bool layout){
  static const char* names[]={"Температуры","Отбор","Wi-Fi","Экран","Система","Google","К1-К4"};
  for(int i=0;i<7;i++){
    int x=4+(i%2)*158,y=52+(i/2)*38;
    hqPanel(x,y,153,35);
    hqSymbol(i==0?1:i==1?2:i==2?0:i==3?4:i==4?3:5,
             x+19,y+18,8,V14_BLUE);
    if(i==approvedSettingsItem)screen->fillRoundRect(x+3,y+4,4,27,2,V14_ORANGE);
    v14Print(names[i],x+35,y+22,V14_TEXT);
    v14Print(">",x+138,y+22,V14_MUTED);
  }
  if(tftView){
    hqPanel(37,72,247,126);
    v14Big(String(names[approvedSettingsItem]),49,95,V14_TEXT);
    screen->drawFastHLine(47,103,226,V14_RULE);
    switch(approvedSettingsItem){
      case 0:
        v14Print("T1–T4: справочные поля",48,124,V14_TEXT);
        v14Tiny("Заданное / Предупреждение",48,144,V14_MUTED);
        v14Tiny("ENT: открыть экран Т",48,174,V14_BLUE);break;
      case 1:
        v14Print("Датчика отбора нет",48,126,V14_WARN);
        v14Tiny("YF-S401 — только вода",48,151,V14_MUTED);break;
      case 2:
        v14Print(WiFi.status()==WL_CONNECTED?"Интернет Wi-Fi: ДА":"Интернет Wi-Fi: НЕТ",48,120,V14_TEXT);
        v14Tiny(String("Сеть: ")+(WiFi.status()==WL_CONNECTED?stationSsid.substring(0,19):String("--")),48,139,V14_BLUE);
        v14Tiny("AP: "+WiFi.softAPIP().toString(),48,158,V14_BLUE);
        v14Tiny(String("Клиентов локально: ")+WiFi.softAPgetStationNum(),48,176,V14_TEXT);break;
      case 3:
        v14Print("ST7789 / 320x240",48,129,V14_TEXT);
        v14Tiny("Ориентация: горизонтальная",48,151,V14_MUTED);break;
      case 4:
        v14Print(keypadLearning?"Обучение 5x4":"Диагностика 5x4",48,124,V14_TEXT);
        v14Tiny(keypadLearning?(String("Клавиша: ")+String(KEY_CAPS[learnStep])):
                  "Обучение через iPhone",48,145,V14_BLUE);
        v14Tiny(String("PCF: ")+(pcfDetected?"есть":"нет")+
                  "  scan="+String(pcfScanError),48,167,V14_WARN);break;
      case 5:
        v14Print(googleEnabled?"Google настроен":"Google: НЕТ НАСТРОЕК",48,124,V14_TEXT);
        v14Tiny(String("Канал: ")+(googleState==5?"ПОДТВЕРЖДЕНО":
                    googleState==4?"ОТПРАВЛЕНО":
                    googleState==3?"ОТПРАВКА":
                    googleState==1?"НЕТ ИНТЕРНЕТА":"ПРОВЕРЬ СТАТУС"),
                   48,146,V14_BLUE);
        v14Tiny("Настройка: iPhone / Google",48,167,V14_MUTED);
        break;
      default:
        v14Print("К1–К4: БЛОК",48,126,V14_WARN);
        for(int i=0;i<4;i++){
          int x=49+i*53;
          screen->fillRoundRect(x,140,46,29,4,V14_INFO);
          v14Tiny("К"+String(i+1),x+5,151,V14_TEXT);
          v14Tiny("ВЫКЛ",x+5,164,V14_WARN);
        }
    }
    v14Tiny("ESC: назад",190,190,V14_MUTED);
  }
}
// Called on TFT startup, page switch, theme change or keypad action.
void renderLandscape(uint32_t now){
  v14ApplyPalette(tftThemeDark);
  v13Collect(now);
  v14InvalidateLive();
  tftFieldCount=0;
  v14Frame(now);
  switch(currentPage){
    case 0:v14Home(now,true);break;
    case 1:v14Temperature(now,true);break;
    case 2:v14Collection(true);break;
    case 3:v14Graphs(true);break;
    case 4:v14Events(true);break;
    case 5:v14Settings(true);break;
  }
  tftDirty=false;
}

// No full-screen erase in the live-update path. Only changed numerical fields
// and the narrow clock/flow strips are repainted.
void v14RefreshLive(uint32_t now){
  v13Collect(now);
  v14NetIndicators();
  if(currentPage==0){
    for(int i=0;i<4;i++)
      v14Metric(v13Temp(i,now),21,52+i*36+30,94,hasTemp(i,now));
    v14DrawFlowBar();
  } else if(currentPage==1 && !editingTemp){
    for(int i=0;i<4;i++){
      int x=4+(i%2)*158,y=52+(i/2)*77;
      v14Metric(v13Temp(i,now),x+11,y+40,133,hasTemp(i,now));
    }
  }
  String dt=deviceClockValid()?clockStamp(true):String("--.--.-- --:--");
  if(dt.length()==16)dt=dt.substring(0,6)+dt.substring(8);
  if(dt!=v14ClockLast){
    v14ClockLast=dt;
    screen->fillRect(220,1,100,19,V14_CARD);
    ru.setFont(u8g2_font_6x13_t_cyrillic);
    int tw=ru.getUTF8Width(dt.c_str());
    v14Print(dt,max(220,317-tw),15,V14_TEXT);
  }
}
