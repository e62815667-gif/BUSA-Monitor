#pragma once
// БУСА-1М №001 — V14.7: flicker-free TFT, monochrome dark theme, Internet / local Wi-Fi LED.
// THIS CODE ONLY DRAWS DATA. NO RELAY/POWER/PWM CONTROL.
// No placeholder readings: water flow raw pulses only until calibrated.
// Google transport is implemented later in BUSA_Google.h; TFT displays status only.
extern volatile bool googleEnabled;
extern volatile int googleState;
extern volatile int googleInternet;
static const uint8_t v14Order[7]={0,1,2,5,6,3,4};
static const char* const v14Tabs[7]={"Глав","Т","Отбор","Настр","Диаг","Граф","Событ"};
// V14 palette is software-defined. DO NOT use ST7789 invertDisplay(): it
// inverts every accent and destroys the colored illustration.
static uint16_t V14_TEXT=0x2104,V14_MUTED=0x632c,V14_WARN=0xa800,
 V14_GREEN=0x2644,V14_BLUE=0x0338,V14_ORANGE=0xfc60,V14_SEA=0x0578,
 V14_CARD=0xffff,V14_PALE=0xffdf,V14_RULE=0xd69a;
static uint16_t v14SeriesColors[4]={0x037b,0xd980,0xb500,0x3685};
static uint16_t V14_INFO=0xff16,V14_GRID=0xe71c,V14_BADGE=0xe7f8;
static uint32_t v14FlowLast=UINT32_MAX;
static String v14ClockLast="";
static int8_t v14NetLast=-1,v14GoogleLast=-1,v14LocalLast=-1;
struct V14MetricCache{int16_t x=-1,y=-1,width=-1;String value;uint16_t fg=0,bg=0;};
static V14MetricCache v14Metrics[10];
static uint8_t v14MetricCount=0;
static void v14InvalidateLive(){v14MetricCount=0;v14FlowLast=UINT32_MAX;v14ClockLast="";v14NetLast=-1;v14GoogleLast=-1;v14LocalLast=-1;}
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
   if(tftThemeMode==0){
     HQ_NAVY=0xffff;HQ_DEEP=0xffff;HQ_GLOW=0xe73c;HQ_GLASS=0xef7d;
     V14_CARD=0xffff;V14_PALE=0xffff;V14_BADGE=0xef7d;
   }
   if(tftThemeMode==2){ // SYSTEM: exact design colors of public BUSA online monitor
     HQ_MONO_DARK=false; // preserve color-coded temperature sensors and detailed artwork
     HQ_NAVY=0x08c5;
     HQ_DEEP=0x1147;
     HQ_GLOW=0x2cdb;
     HQ_AZURE=0x19a9;
     HQ_SKY=0x869f;
     HQ_CYAN=0x2cdb;
     HQ_LIGHT=0xf7bf;
     HQ_LIME=0x7736;
     HQ_AQUA=0x3c15;
     HQ_GLASS=0x1a0a;
     HQ_STEEL=0x32ad;
     HQ_SILVER=0xbebc;
     HQ_METAL=0x7453;
     HQ_DARKMETAL=0x2a8d;
     V14_TEXT=0xf7bf;
     V14_MUTED=0xae3a;
     V14_WARN=0xda4a;
     V14_GREEN=0x7736;
     V14_BLUE=0x2cdb;
     V14_ORANGE=0x0cc9;
     V14_SEA=0x7e9f;
     V14_CARD=0x1147;
     V14_PALE=0x08c5;
     V14_RULE=0x32ad;
     V14_INFO=0x19a9;
     V14_GRID=0x2a8d;
     V14_BADGE=0x1247;
     v14SeriesColors[0]=0x14b8;
     v14SeriesColors[1]=0xd407;
     v14SeriesColors[2]=0xc287;
     v14SeriesColors[3]=0x3cee;
   }
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
// Three independent Wi-Fi/Google/ESP32 indicators in the 320px display header.
static void v14NetIndicators(){
  const int8_t net=(WiFi.status()==WL_CONNECTED)?1:0;
  const int8_t google=(googleEnabled && googleInternet==1 && (googleHttp==200 || googleHttp==302))?1:0;
  const int8_t local=(WiFi.softAPgetStationNum()>0)?1:0;
  if(v14NetLast==net && v14GoogleLast==google && v14LocalLast==local)return;
  v14NetLast=net;v14GoogleLast=google;v14LocalLast=local;
  screen->fillRect(78,2,146,17,V14_CARD);
  screen->fillCircle(85,11,3,net?0x07e0:0xf800);
  v14Tiny("WiFi",91,14,V14_TEXT);
  screen->fillCircle(131,11,3,google?0x07e0:0xf800);
  v14Tiny("Google",137,14,V14_TEXT);
  screen->fillCircle(183,11,3,local?0x07e0:0xf800);
  v14Tiny("ESP32",189,14,V14_TEXT);
}
static void v14Frame(uint32_t now){
  hqBackground();
  screen->fillRect(0,0,320,21,V14_CARD);
  screen->drawFastHLine(0,21,320,V14_ORANGE);
  v14Big("БУСА-1М",5,15,V14_TEXT);
  v14NetIndicators();
  // DD.MM.YY HH:MM. Exact pixel width is measured; date cannot clip at 320.
  String dt=deviceClockValid()?clockStamp(true):String("--.--.-- --:--");
  if(dt.length()==16)dt=dt.substring(0,6)+dt.substring(8);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
  int tw=ru.getUTF8Width(dt.c_str());
  v14Print(dt,max(220,317-tw),15,V14_TEXT);
  // Six tabs, preserving internal IDs with the explicit approved visual order.
  for(int i=0;i<7;i++){
    int x=1+i*45;bool chosen=(v14Order[i]==currentPage);
    screen->fillRoundRect(x,25,43,22,4,chosen?V14_ORANGE:V14_CARD);
    screen->drawRoundRect(x,25,43,22,4,chosen?V14_ORANGE:V14_RULE);
    if(chosen)screen->drawFastHLine(x+5,45,33,V14_WARN);
    String title=v14Tabs[i];
    int w=ru.getUTF8Width(title.c_str());
    v14Tiny(title,x+max(1,(43-w)/2),40,V14_TEXT);
  }
  // Only approved input pictograms; the * key is the decimal comma in editors.
  screen->fillRect(0,211,320,29,V14_CARD);
  screen->drawFastHLine(0,211,320,V14_ORANGE);
  for(int i=0;i<7;i++){
    const int x=3+i*45;
    const int cx=x+21,cy=226;
    screen->fillRoundRect(x,215,42,22,4,i==4?V14_BADGE:V14_CARD);
    screen->drawRoundRect(x,215,42,22,4,V14_RULE);
    if(i<4){v14FootArrow(x+13,i,V14_ORANGE);}
    else if(i==4){ // *
      screen->drawFastVLine(cx,219,14,V14_GREEN);
      screen->drawLine(cx-6,223,cx+6,230,V14_GREEN);
      screen->drawLine(cx-6,230,cx+6,223,V14_GREEN);
    }else if(i==5){ // ENT/OK: checkmark only
      screen->drawCircle(cx,cy,8,V14_GREEN);
      screen->drawLine(cx-5,cy,cx-1,cy+4,V14_GREEN);
      screen->drawLine(cx-1,cy+4,cx+6,cy-5,V14_GREEN);
    }else{ // ESC/back: hooked left arrow
      screen->drawFastHLine(cx-6,cy+3,13,V14_BLUE);
      screen->drawFastVLine(cx+7,cy-5,9,V14_BLUE);
      screen->drawLine(cx-6,cy+3,cx-1,cy-2,V14_BLUE);
      screen->drawLine(cx-6,cy+3,cx-1,cy+8,V14_BLUE);
    }
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
     " имп. (не калибр.)    K1-K4: ЗВУК 12В",8,206,V14_TEXT);
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
    v14Tiny(String(i==selectedTemp && selectedField==0?">ВКЛ: ":"ВКЛ: ")+v13Ref(refTemp[i]),x+12,y+54,(i==selectedTemp && selectedField==0)?V14_ORANGE:V14_MUTED);
    v14Tiny(String(i==selectedTemp && selectedField==1?">ВЫКЛ: ":"ВЫКЛ: ")+v13Ref(noticeTemp[i]),x+12,y+66,(i==selectedTemp && selectedField==1)?V14_ORANGE:V14_MUTED);
  }
  if(deleteConfirmPending){
    hqPanel(35,70,250,126);
    v14Big("УДАЛИТЬ ПОРОГ?",43,94,V14_WARN);
    v14Print(String("T")+String(selectedTemp+1)+
      (selectedField?" ВЫКЛ":" ВКЛ"),43,120,V14_TEXT);
    v14Print("ENT: удалить",43,150,V14_WARN);
    v14Print("ESC: отмена",43,172,V14_GREEN);
  }else if(editingTemp){
    hqPanel(43,73,234,114);
    v14Big(String("T")+String(selectedTemp+1)+(selectedField?" ВЫКЛ":" ВКЛ"),53,97,V14_TEXT);
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
// Approved diagnostic-style graph layout: two views, independent vertical lanes.
// All samples come from the same 5s DS18B20 circular history as /api/graph-history.
static void v14GraphTabs(){
  static const char* names[]={"ТЕМПЕРАТУРЫ","ВОДА И ОТБОР"};
  for(int j=0;j<2;j++){
    int x=5+j*156;
    screen->fillRoundRect(x,52,151,22,4,graphSubView==j?V14_ORANGE:V14_CARD);
    screen->drawRoundRect(x,52,151,22,4,V14_RULE);
    String label=names[j];
    ru.setFont(u8g2_font_5x7_t_cyrillic);
    int w=ru.getUTF8Width(label.c_str());
    v14Tiny(label,x+max(3,(151-w)/2),67,V14_TEXT);
  }
}
static void v14Graphs(bool layout){
  if(layout){
    v14GraphTabs();
    hqPanel(5,79,310,128);
    screen->fillRoundRect(10,84,300,119,5,0x0000);
    screen->drawRoundRect(10,84,300,119,5,V14_BLUE);
  }else{
    // Repaint only pixels inside the already drawn chart border.
    // Header, tabs, footer and fixed frame never disappear on a 5s sample.
    screen->fillRect(12,86,296,115,0x0000);
  }
  if(graphSubView==0){
    const int x0=83,xw=215,bandY=92,bandH=26;
    for(int k=0;k<4;k++){
      const int rowY=bandY+k*bandH;
      if(k)screen->drawFastHLine(14,rowY-2,290,V14_RULE);
      static const char* names[]={"T1 Вода","T2 Кол.","T3 Куб","T4 Верх"};
      v14Tiny(names[k],15,rowY+8,v14SeriesColors[k]);
      // Show the current measured value alongside each independent trace.
      const int lastIndex=(v13HistoryHead+47)%48;
      const float latest=v13HistoryCount?v13History[k][lastIndex]:NAN;
      v14Tiny(isfinite(latest)?String(latest,1):String("Н/Д"),26,rowY+20,V14_TEXT);
      screen->drawFastVLine(x0-3,rowY+1,21,V14_RULE);
      float lo=INFINITY,hi=-INFINITY;
      for(int i=0;i<v13HistoryCount;i++){
        const int idx=(v13HistoryHead+48-v13HistoryCount+i)%48;
        const float t=v13History[k][idx];
        if(isfinite(t)){lo=fminf(lo,t);hi=fmaxf(hi,t);}
      }
      if(!isfinite(lo)||!isfinite(hi))continue;
      const float padding=fmaxf(0.2f,(hi-lo)*0.22f);
      lo-=padding;hi+=padding;
      for(int i=0;i<v13HistoryCount;i++){
        const int idx=(v13HistoryHead+48-v13HistoryCount+i)%48;
        const float t=v13History[k][idx];
        if(!isfinite(t))continue;
        const int x=x0+(v13HistoryCount<2?xw-1:(i*(xw-1))/47);
        const int y=rowY+20-(int)(fminf(1.0f,fmaxf(0.0f,(t-lo)/(hi-lo)))*18.0f);
        if(i==0 || !isfinite(v13History[k][(idx+47)%48])){
          screen->drawPixel(x,y,v14SeriesColors[k]);
        }else{
          const int pidx=(idx+47)%48;
          const float prev=v13History[k][pidx];
          const int yp=rowY+20-(int)(fminf(1.0f,fmaxf(0.0f,(prev-lo)/(hi-lo)))*18.0f);
          const int xp=x0+((i-1)*(xw-1))/47;
          screen->drawLine(xp,yp,x,y,v14SeriesColors[k]);
        }
      }
    }

  }else{
    v14Tiny("ВОДА · YF-S401 · импульсы/2c",16,98,V14_BLUE);
    screen->drawFastHLine(14,146,291,V14_RULE);
    v14Tiny("ОТБОР · НЕТ ДАТЧИКА",16,163,V14_WARN);
    v14Tiny("Линия отсутствует · данных нет",16,181,V14_MUTED);
    const int px=77,py=109,pw=216,ph=30;
    screen->drawFastVLine(px,py,ph,V14_RULE);
    uint32_t vmax=1;
    for(int i=0;i<waterSamplesRecorded;i++){
      int idx=(flowTraceHead+40-waterSamplesRecorded+i)%40;
      if(waterPulseHistory[idx]>vmax)vmax=waterPulseHistory[idx];
    }
    for(int i=0;i<waterSamplesRecorded;i++){
      int idx=(flowTraceHead+40-waterSamplesRecorded+i)%40;
      int x=px+(i*pw)/39;
      int y=py+ph-1-(int)((uint64_t)waterPulseHistory[idx]*(ph-3)/vmax);
      if(i==0)screen->drawPixel(x,y,V14_SEA);
      else{
        int prev=(flowTraceHead+40-waterSamplesRecorded+i-1)%40;
        int yprev=py+ph-1-(int)((uint64_t)waterPulseHistory[prev]*(ph-3)/vmax);
        screen->drawLine(px+((i-1)*pw)/39,yprev,x,y,V14_SEA);
      }
    }
    if(!waterSamplesRecorded)v14Tiny("Нет импульсов",88,125,V14_MUTED);

  }
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
    if(i<4)screen->drawFastHLine(14,y+5,289,V14_GRID);
  }
}
static void v14Settings(bool layout){
 static const char* names[]={"Температура","Wi-Fi","Экран","Google","О системе"};
 if(!tftView){
  for(int i=0;i<5;i++){
   int x=5+(i%2)*157,y=53+(i/2)*45;
   hqPanel(x,y,152,41);
   if(i==approvedSettingsItem)screen->fillRoundRect(x+3,y+4,4,32,2,V14_ORANGE);
   v14Print(names[i],x+15,y+24,V14_TEXT);
   v14Tiny("ENT",x+127,y+24,V14_MUTED);
  }
  return;
 }
 hqPanel(25,72,270,131);
 v14Big(String(names[approvedSettingsItem]),35,92,V14_TEXT);
 screen->drawFastHLine(35,100,250,V14_RULE);
 switch(approvedSettingsItem){
  case 0:
   v14Print("T"+String(calSensor+1)+" · поправка: "+String(tempOffset[calSensor],1)+" C",35,123,V14_TEXT);
   if(calEditing){
    v14Big("ВВОД: "+String(calDraft,1)+" C",35,147,V14_ORANGE);
    v14Tiny("UP/DN +-0.1 · ENT сохранить",35,170,V14_BLUE);
    v14Tiny("ESC отменить",35,188,V14_MUTED);
   }else{
    v14Tiny("UP/DN выбор датчика · ENT поправка",35,149,V14_BLUE);
    v14Tiny("Пороги звука не меняются",35,173,V14_MUTED);
   }break;
  case 1: { // Physical TFT uses discovery; SSID is NEVER typed manually.
   switch(wifiUiStage){
    case WIFIUI_SCANNING:
     v14Print("Ищу сети Wi-Fi...",35,127,V14_BLUE);
     v14Tiny("Поиск без блокировки датчиков",35,150,V14_MUTED);
     break;
    case WIFIUI_NETWORKS:
     v14Tiny("Доступные сети: "+String(wifiUiCount),35,116,V14_BLUE);
     for(int j=0;j<3;j++){
       int n=(int)wifiUiIndex+j-1;
       if(n<0||n>=wifiUiCount)continue;
       String name=wifiUiNetworks[n];
       if(name.length()>23)name=name.substring(0,22)+"...";
       v14Tiny(String(n==wifiUiIndex?"> ":"  ")+name,35,139+j*19,
               n==wifiUiIndex?V14_ORANGE:V14_TEXT);
     }
     v14Tiny("UP/DN выбор · ENT пароль",35,195,V14_MUTED);
     break;
    case WIFIUI_PASSWORD:{
     String ssid=wifiUiSelectedSsid;
     if(ssid.length()>22)ssid=ssid.substring(0,21)+"...";
     v14Tiny("Сеть: "+ssid,35,115,V14_TEXT);
     v14Print(String("Режим ")+wifiUiModeCaption(),35,137,V14_ORANGE);
     String masked=wifiUiPasswordMask();
     if(masked.length()>28)masked="..."+masked.substring(masked.length()-25);
     v14Tiny("Пароль: "+masked,35,156,V14_BLUE);
     v14Tiny("UP/DN режим · F1 удалить",35,176,V14_MUTED);
     v14Tiny("0 долго: запятая · ENT ОК",35,195,V14_MUTED);
     break;
    }
    case WIFIUI_CONNECTING:
     v14Print("Подключаю Wi-Fi...",35,125,V14_BLUE);
     v14Tiny("Сеть: "+wifiUiSelectedSsid,35,147,V14_TEXT);
     v14Tiny(wifiUiFeedback,35,172,V14_MUTED);
     break;
    default:
     v14Print(WiFi.status()==WL_CONNECTED?"Wi-Fi подключён":"Wi-Fi нет связи",35,122,V14_TEXT);
     v14Tiny("ENT: искать доступные сети",35,147,V14_BLUE);
     v14Tiny(wifiUiFeedback,35,172,V14_MUTED);
   }
   break;
  }
  case 2:
   v14Print(themeDraft==0?"Светлая":themeDraft==1?"Тёмная":"Системная",35,126,V14_TEXT);
   v14Tiny("UP/DN выбор · ENT сохранить",35,152,V14_BLUE);
   v14Tiny("Системная: рисунки БУСА",35,177,V14_MUTED);break;
  case 3:
   v14Print(googleEnabled?"Google: настроен":"Google: не настроен",35,125,V14_TEXT);
   v14Tiny("Передача: "+String(googleState==5?"подтверждена":googleState==6?"ошибка":"проверяется"),35,149,V14_BLUE);
   v14Tiny("Параметры подключения: iPhone",35,175,V14_MUTED);break;
  default:
   v14Tiny(String("Wi-Fi: ")+(WiFi.status()==WL_CONNECTED?"да":"нет"),35,118,V14_TEXT);
   v14Tiny(String("Google: ")+(googleEnabled?"да":"нет"),35,138,V14_TEXT);
   v14Tiny("Прошивка: V14.8 KEYBOARD",35,158,V14_BLUE);
   v14Tiny(String("Работа: ")+(millis()/60000)+" мин",35,180,V14_TEXT);
 }
}
static void v14Diagnostics(bool layout){
 static const char* modules[]={"Экран TFT","Клавиатура","Реле K1-K4","Датчики T1-T4",
 "Wi-Fi / Google","ESP32","Расходомер"};
 if(!tftView){
  for(int i=0;i<7;i++){
   int x=4+(i%2)*158,y=53+(i/2)*38;
   hqPanel(x,y,153,34);
   if(i==diagModule)screen->fillRoundRect(x+3,y+3,4,28,2,V14_ORANGE);
   v14Tiny(modules[i],x+14,y+20,V14_TEXT);
  }
  return;
 }
 hqPanel(19,69,282,135);
 v14Print(String(modules[diagModule]),28,89,V14_TEXT);
 screen->drawFastHLine(28,98,263,V14_RULE);
 switch(diagModule){
  case 0:
   v14Tiny("ST7789 320x240 · SPI",28,121,V14_TEXT);
   v14Tiny("Изображение: визуальный тест",28,148,V14_MUTED);
   v14Tiny(tftStarted?"Запуск выполнен":"Не запущен",28,173,V14_BLUE);break;
  case 1:
   v14Tiny(pcfDetected?"PCF8575 0x20: отвечает":"PCF8575: нет связи",28,121,V14_TEXT);
   v14Tiny(String("Карта 20: ")+(keypadReady?"сохранена":"нет"),28,145,V14_BLUE);
   v14Tiny(String("Ошибка: ")+pcfScanError,28,170,V14_TEXT);break;
  case 2:
   for(int i=0;i<4;i++)
    v14Tiny(String("K")+String(i+1)+": "+(busaSoundActive(i)?"ЗВУК":"ОЖИДАНИЕ"),
     28+(i%2)*132,121+(i/2)*27,V14_TEXT);
   v14Tiny("Физическое срабатывание: отдельно",28,181,V14_MUTED);break;
  case 3:
   for(int i=0;i<4;i++)
    v14Tiny(String("T")+String(i+1)+": "+
     (hasTemp(i,millis())?String(temperature[i],1)+" C":"Н/Д"),
     28+(i%2)*132,121+(i/2)*27,V14_TEXT);
   v14Tiny("Поправки: Настройки",28,181,V14_MUTED);break;
  case 4:
   v14Tiny(WiFi.status()==WL_CONNECTED?"Wi-Fi STA: да":"Wi-Fi STA: нет",28,122,V14_TEXT);
   v14Tiny(googleEnabled?"Google: настроен":"Google: нет",28,146,V14_BLUE);
   v14Tiny("AP: "+WiFi.softAPIP().toString(),28,171,V14_TEXT);break;
  case 5:
   v14Tiny("ESP32-S3 · 240 МГц",28,121,V14_TEXT);
   v14Tiny("RAM: "+String(ESP.getFreeHeap()/1024)+" КБ",28,144,V14_BLUE);
   v14Tiny("Uptime: "+String(millis()/60000)+" мин",28,170,V14_TEXT);break;
  case 6:
   v14Tiny("GPIO15 · YF-S401",28,121,V14_TEXT);
   v14Tiny("Импульсы: "+String(flowPulses()),28,146,V14_BLUE);
   v14Tiny("Калибровка: НЕТ",28,171,V14_MUTED);break;
 }
}
// Called on TFT startup, page switch, theme change or keypad action.
void renderLandscape(uint32_t now){
  v14ApplyPalette(tftThemeMode==1);
  v13Collect(now);
  v14InvalidateLive();
  tftFieldCount=0;
  v14Frame(now);
  switch(currentPage){
    case 0:v14Home(now,true);break;
    case 1:v14Temperature(now,true);break;
    case 2:v14Collection(true);break;
    case 3:v14Graphs(true);busaGraphLastDrawnCapture=v13LastCapture;break;
    case 4:v14Events(true);break;
    case 5:v14Settings(true);break;
    case 6:v14Diagnostics(true);break;
  }
  tftDirty=false;
}

// No full-screen erase in the live-update path. Only changed numerical fields
// and the narrow clock/flow strips are repainted.
void v14RefreshLive(uint32_t now){
  v13Collect(now);
  v14NetIndicators();
  if(currentPage==3 && v13HistoryCount && 
     v13LastCapture!=busaGraphLastDrawnCapture){
    v14Graphs(false); // chart pixels only; prevents full-screen flashing
    busaGraphLastDrawnCapture=v13LastCapture;
  }
  if(currentPage==0){
    for(int i=0;i<4;i++)
      v14Metric(v13Temp(i,now),21,52+i*36+30,94,hasTemp(i,now));
    v14DrawFlowBar();
  } else if(currentPage==1 && !editingTemp && !deleteConfirmPending){
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
