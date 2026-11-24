#pragma once
// БУСА-1М №001 — V14.9: unified nested TFT Settings, 4x calibration, Wi-Fi picker, 3 themes, About.
// THIS CODE ONLY DRAWS DATA. NO RELAY/POWER/PWM CONTROL.
// No placeholder readings: water flow raw pulses only until calibrated.
// Google transport is implemented later in BUSA_Google.h; TFT displays status only.
extern volatile bool googleEnabled;
extern volatile int googleState;
extern volatile int googleInternet;
static const char* const v14FooterSectionNames[8]={
 "Главная","Температуры","Отбор","Графики","События","Настройки","Диагностика","Калькулятор"
};
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
static void v14Metric(const String& txt,int x,int y,int width,bool valid,uint16_t bg=V14_CARD){
  // Update only CHANGED glyphs. Never erase entire frame for live telemetry.
  uint16_t color=valid?V14_GREEN:V14_WARN;
  for(uint8_t i=0;i<v14MetricCount;i++){
    V14MetricCache& m=v14Metrics[i];
    if(m.x==x && m.y==y && m.width==width){
      if(m.value==txt && m.fg==color && m.bg==bg)return;
      m.value=txt;m.fg=color;m.bg=bg;
      screen->fillRect(x,y-13,width,16,bg);
      v14Big(txt,x,y,color);return;
    }
  }
  if(v14MetricCount<10){
    V14MetricCache& m=v14Metrics[v14MetricCount++];
    m.x=x;m.y=y;m.width=width;m.value=txt;m.fg=color;m.bg=bg;
  }
  screen->fillRect(x,y-13,width,16,bg);
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
  v14Tiny("BUSA MONITOR",5,15,V14_TEXT);
  v14NetIndicators();
  // DD.MM.YY HH:MM. Exact pixel width is measured; date cannot clip at 320.
  String stamp=clockStamp(true);
  String dt=deviceClockValid()?stamp.substring(stamp.length()-5):String("--:--");

  ru.setFont(u8g2_font_6x13_t_cyrillic);
  int tw=ru.getUTF8Width(dt.c_str());
  v14Print(dt,max(220,317-tw),15,V14_TEXT);
  // Top-level tab labels intentionally removed. Sections are changed by LEFT/RIGHT.
  // Current section is shown only in the bottom navigation bar.
  // Simplified bottom bar: four navigation arrows, current section, ENTER, ESC/back.
  // Visual change only: physical key codes, NVS mappings and key actions are unchanged.
  screen->fillRect(0,211,320,29,V14_CARD);
  screen->drawFastHLine(0,211,320,V14_ORANGE);
  for(int i=0;i<4;i++){
    const int x=3+i*34;
    screen->fillRoundRect(x,215,31,22,4,V14_CARD);
    screen->drawRoundRect(x,215,31,22,4,V14_RULE);
    v14FootArrow(x+7,i,V14_ORANGE);
  }
  // Section caption replaces the misleading '0,' button.
  const int sectionX=139,sectionW=92;
  screen->fillRoundRect(sectionX,215,sectionW,22,4,V14_BADGE);
  screen->drawRoundRect(sectionX,215,sectionW,22,4,V14_ORANGE);
  const char* section=v14FooterSectionNames[(currentPage==4||currentPage==6)?5:currentPage<8?currentPage:0];
  ru.setFont(u8g2_font_5x7_t_cyrillic);
  const int sectionTextW=ru.getUTF8Width(section);
  v14Tiny(section,sectionX+max(2,(sectionW-sectionTextW)/2),229,V14_TEXT);
  // Explicit text instead of unlabeled checkmark/back icons.
  screen->fillRoundRect(234,215,39,22,4,V14_CARD);
  screen->drawRoundRect(234,215,39,22,4,V14_RULE);
  v14Tiny("ENTER",241,229,V14_GREEN);
  screen->fillRoundRect(276,215,41,22,4,V14_CARD);
  screen->drawRoundRect(276,215,41,22,4,V14_RULE);
  v14Tiny("ESC",288,229,V14_BLUE);
}
static void v14MiniRig(){
  // Enlarged approved illustration; only drawing coordinates have changed.
  hqRig(117,25,130,148,false);
}
static void v14Home(uint32_t now,bool layout){
  static const char* names[]={"Вода","Колонна","Куб","Верх колонны"};
  // Clear the now-redundant top-tabs separator only on the home screen.
  screen->fillRect(0,22,320,29,HQ_NAVY);
  v14MiniRig();
  // Larger frameless T1-T4 readings, retaining all four sensor names and data.
  for(int i=0;i<4;i++){
    const int y=27+i*36;
    screen->fillCircle(11,y+11,4,v14SeriesColors[i]);
    v14Print(names[i],21,y+16,V14_TEXT);
    v14Metric(v13Temp(i,now),21,y+31,94,hasTemp(i,now),HQ_NAVY);
  }
  hqPanel(248,27,69,66);
  v14Tiny("ОТБОР",253,41,V14_TEXT);
  v14Metric(v13Flow(),253,73,59,isfinite(collectionMeter.rate()));
  v14Tiny("мл/мин",253,86,V14_MUTED);
  hqPanel(248,101,69,70);
  v14Tiny("СОБРАНО",252,115,V14_TEXT);
  v14Metric(v13Total(),253,142,59,isfinite(collectionMeter.total()));
  v14Tiny("мл",253,164,V14_MUTED);
  busaDrawRelayStrip(180);
}
static void v14Temperature(uint32_t now,bool layout){
  // Sensor positions only; keep original T1/T2/T3/T4 pins and saved alarm values.
  static const char* names[]={"Вода","Колонна","Куб","Верх колонны"};
  if(!tftView){
    // Same vertical list model as Settings; no diagonal focus jumps.
    for(int sensor=0;sensor<4;sensor++){
      const int x=6,y=25+sensor*45;
      hqPanel(x,y,308,42);
      if(sensor==selectedTemp)screen->fillRoundRect(x+3,y+4,4,34,2,V14_ORANGE);
      screen->fillCircle(x+16,y+14,3,v14SeriesColors[sensor]);
      v14Print(names[sensor],x+26,y+18,V14_TEXT);
      v14Metric(v13Temp(sensor,now),220,y+18,83,hasTemp(sensor,now));
      v14Tiny(String("ВКЛ: ")+v13Ref(refTemp[sensor])+" °C   ВЫКЛ: "+
        v13Ref(noticeTemp[sensor])+" °C",x+26,y+34,V14_MUTED);
    }
    return;
  }
  // Full-page settings replace the previous floating temperature edit popup.
  hqPanel(6,25,308,181);
  const int sensor=selectedTemp;
  v14Big(String("T")+String(sensor+1)+" "+String(names[sensor]),17,47,V14_TEXT);
  v14Metric(v13Temp(sensor,now),230,46,69,hasTemp(sensor,now));
  screen->drawFastHLine(16,55,288,V14_RULE);
  for(int field=0;field<2;field++){
    const int x=15,y=66+field*58;
    const bool chosen=selectedField==field;
    hqPanel(x,y,290,53);
    if(chosen)screen->fillRoundRect(x+4,y+4,5,45,2,V14_ORANGE);
    const String label=String("T")+String(sensor+1)+(field?" ВЫКЛ":" ВКЛ");
    v14Print(label,x+18,y+19,chosen?V14_ORANGE:V14_TEXT);
    const float stored=field?noticeTemp[sensor]:refTemp[sensor];
    String value=v13Ref(stored);
    if(editingTemp && chosen)value=editTempBuffer.isEmpty()?"--":editTempBuffer;
    v14Big(value,x+19,y+43,editingTemp&&chosen?V14_GREEN:V14_TEXT);
  }
  if(deleteConfirmPending){
    v14Tiny("ENT удалить / ESC отмена",18,197,V14_WARN);
  }else if(editingTemp){
    v14Tiny("ENT сохранить / ESC отмена",18,197,V14_GREEN);
  }else{
    v14Tiny("ENT изменить / ESC назад",18,197,V14_MUTED);
  }
}
static void v14Collection(bool layout){
  // The flow graph is shown only under "Графики"; no duplicate here.
  hqPanel(5,25,151,181);
  v14Print("Расход отбора",13,47,V14_TEXT);
  v14Metric(v13Flow(),14,113,137,isfinite(collectionMeter.rate()));
  v14Print("мл/мин",17,146,V14_MUTED);
  hqPanel(162,25,153,181);
  v14Print("Собрано",170,47,V14_TEXT);
  v14Metric(v13Total(),174,113,128,isfinite(collectionMeter.total()));
  v14Print("мл",174,146,V14_MUTED);
}
// Approved diagnostic-style graph layout: two views, independent vertical lanes.
// All samples come from the same 5s DS18B20 circular history as /api/graph-history.
static void v14Graphs(bool layout){busaDrawChart(millis());}
static void v14Events(bool layout){
  hqPanel(5,25,310,181);
  v14Print("ЖУРНАЛ СОБЫТИЙ",14,46,V14_TEXT);
  screen->drawFastHLine(14,54,290,V14_RULE);
  const int count=min((int)v13EventCount,5);
  for(int i=0;i<count;i++){
    const int y=80+i*24;
    screen->fillCircle(19,y-4,3,V14_ORANGE);
    v14Tiny(v13Events[i],29,y,V14_TEXT);
    if(i<4)screen->drawFastHLine(14,y+6,289,V14_GRID);
  }
}
// Consistent nested settings presentation. Each ENT opens exactly one child
// page; ESC returns to the five-item ordered list. No extra modal windows.
static void v14SystemInfoRows(){
  const bool staOk=(WiFi.status()==WL_CONNECTED);
  const bool googleOk=googleEnabled && googleInternet==1 &&
    googleState==5 && (googleHttp==200 || googleHttp==302);
  const bool espApOk=(WiFi.getMode()==WIFI_AP || WiFi.getMode()==WIFI_AP_STA);
  const uint32_t uptimeSeconds=millis()/1000;
  v14Print(String("Wi-Fi подключён: ")+(staOk?"да":"нет"),17,91,V14_TEXT);
  v14Print(String("Google: ")+(googleOk?"да":"нет"),17,113,V14_TEXT);
  v14Print(String("ESP32 Wi-Fi: ")+(espApOk?"да":"нет"),17,135,V14_TEXT);
  v14Print("Прошивка: V14.9",17,157,V14_TEXT);
  v14Print("Работа: "+String(uptimeSeconds/60)+" мин "+
           String(uptimeSeconds%60)+" сек",17,179,V14_TEXT);
}
static void v14Settings(bool layout){
  static const char* names[]={"Температура","Wi-Fi","Экран","Google","О системе","Журнал событий","Диагностика"};
  if(!tftView){
    for(int i=0;i<7;i++){
      const int x=8,y=26+i*25;
      hqPanel(x,y,304,23);
      if(i==approvedSettingsItem)screen->fillRoundRect(x+4,y+3,4,17,2,V14_ORANGE);
      v14Print(names[i],x+21,y+16,V14_TEXT);
    }
    return;
  }
  hqPanel(7,25,306,181);
  switch(approvedSettingsItem){
    case 0:{
      v14Big("Температура",17,48,V14_TEXT);
      screen->drawFastHLine(17,55,280,V14_RULE);
      for(int i=0;i<4;i++){
        const int y=68+i*33;
        hqPanel(14,y,291,30);
        if(i==calSensor)screen->fillRoundRect(18,y+4,4,22,2,V14_ORANGE);
        const float offset=(calEditing&&i==calSensor)?calDraft:tempOffset[i];
        const String correction=String(offset>=0?"+":"")+String(offset,1)+" °C";
        v14Print("T"+String(i+1),29,y+21,V14_TEXT);
        v14Print(correction,181,y+21,i==calSensor?V14_ORANGE:V14_TEXT);
      }
      break;
    }
    case 1:{
      // Ordered Wi-Fi settings only. Never disconnect the always-on BUSA AP.
      v14Big("Wi-Fi",17,48,V14_TEXT);
      screen->drawFastHLine(17,55,280,V14_RULE);
      if(wifiUiStage==WIFIUI_SCANNING){
        v14Print("Поиск сетей...",24,103,V14_TEXT);
        v14Tiny("Поиск выполняется автоматически",24,149,V14_MUTED);
      }else if(wifiUiStage==WIFIUI_NETWORKS){
        v14Tiny("Выбери сеть: UP/DOWN, ENTER",21,67,V14_MUTED);
        const int first=(wifiUiIndex/5)*5;
        for(int row=0;row<5;row++){
          const int index=first+row;
          if(index>=wifiUiCount)break;
          const int y=72+row*25;
          hqPanel(17,y,285,23);
          if(index==wifiUiIndex)screen->fillRoundRect(20,y+4,4,15,2,V14_ORANGE);
          String ssid=wifiUiNetworks[index];
          if(ssid.length()>33)ssid=ssid.substring(0,30)+"...";
          v14Print(ssid,29,y+17,index==wifiUiIndex?V14_ORANGE:V14_TEXT);
        }
      }else if(wifiUiStage==WIFIUI_PASSWORD){
        String ssid=wifiUiSelectedSsid;
        if(ssid.length()>32)ssid=ssid.substring(0,29)+"...";
        v14Print("Сеть: "+ssid,23,81,V14_TEXT);
        v14Print("Режим: "+String(wifiUiModeCaption()),23,108,V14_TEXT);
        String masked=wifiUiPasswordMask();
        if(masked.length()>35)masked="..."+masked.substring(masked.length()-32);
        v14Print("Пароль: "+masked,23,138,V14_TEXT);
        v14Tiny("UP/DOWN: abc/ABC/123/SYM",23,164,V14_MUTED);
        v14Tiny("F1: стереть. ENT: к сохранению",23,187,V14_GREEN);
      }else{
        static const char* menu[]={
          "Выбрать сеть","Ввести пароль","Подключиться"
        };
        for(int i=0;i<3;i++){
          const int x=16,y=65+i*38;
          hqPanel(x,y,288,34);
          const bool selected=(wifiUiMenuIndex==i);
          if(selected)screen->fillRoundRect(x+3,y+5,5,24,2,V14_ORANGE);
          v14Print(menu[i],x+15,y+23,selected?V14_ORANGE:V14_TEXT);
        }
        String message=wifiUiFeedback;
        if(message.isEmpty())
          message=WiFi.status()==WL_CONNECTED?"Подключено: "+WiFi.SSID():"Нет подключения";
        if(message.length()>42)message=message.substring(0,39)+"...";
        v14Tiny(message,19,200,V14_TEXT);
      }
      break;
    }
    case 2:{
      static const char* themeNames[]={"Светлая","Тёмная","Рисунок БУСА"};
      v14Big("Выбор темы",17,48,V14_TEXT);
      screen->drawFastHLine(17,55,280,V14_RULE);
      v14Print(String("Выбрана: ")+themeNames[tftThemeMode],20,77,V14_TEXT);
      for(int i=0;i<3;i++){
        const int y=89+i*35;
        hqPanel(19,y,278,31);
        if(i==themeDraft)screen->fillRoundRect(23,y+5,4,21,2,V14_ORANGE);
        v14Print(themeNames[i],36,y+21,V14_TEXT);
      }
      break;
    }
    case 3:
      v14Big("Google",17,48,V14_TEXT);
      screen->drawFastHLine(17,55,280,V14_RULE);
      v14Print(googleEnabled?"Google: настроен":"Google: не настроен",23,103,V14_TEXT);
      v14Print(String("Передача: ")+(googleState==5?"да":"нет"),23,141,V14_TEXT);
      break;
    default:
      v14Big("О системе",17,48,V14_TEXT);
      screen->drawFastHLine(17,55,280,V14_RULE);
      v14SystemInfoRows();
  }
}
static void v14Diagnostics(bool layout){
  static const char* modules[]={"Экран TFT","Клавиатура","Блок реле","Датчики T1-T4",
    "Wi-Fi / Google","ESP32","Расходомер"};
  if(!tftView){
    for(int i=0;i<7;i++){
      const int x=8,y=26+i*25;
      hqPanel(x,y,304,23);
      if(i==diagModule)screen->fillRoundRect(x+4,y+3,4,17,2,V14_ORANGE);
      v14Print(modules[i],x+21,y+16,V14_TEXT);
    }
    return;
  }
  hqPanel(19,27,282,179);
  v14Print(String(modules[diagModule]),28,50,V14_TEXT);
  screen->drawFastHLine(28,57,263,V14_RULE);
  switch(diagModule){
    case 0:
      v14Tiny("ST7789 320x240 · SPI",28,87,V14_TEXT);
      v14Tiny("Изображение: визуальный тест",28,125,V14_MUTED);
      v14Tiny(tftStarted?"Запуск выполнен":"Не запущен",28,168,V14_BLUE);break;
    case 1:
      v14Tiny(pcfDetected?"PCF8575 0x20: отвечает":"PCF8575: нет связи",28,87,V14_TEXT);
      v14Tiny(String("Карта 20: ")+(keypadReady?"сохранена":"нет"),28,125,V14_BLUE);
      v14Tiny(String("Ошибка: ")+pcfScanError,28,168,V14_TEXT);break;
    case 2:
      for(int i=0;i<4;i++)
        v14Tiny(String(BUSA_RELAY_LABELS[i])+": "+(busaSoundActive(i)?"ВКЛ":"ВЫКЛ"),
          28+(i%2)*132,100+(i/2)*40,V14_TEXT);
      v14Tiny("Физическое срабатывание: отдельно",28,185,V14_MUTED);break;
    case 3:
      for(int i=0;i<4;i++)
        v14Tiny(String("T")+String(i+1)+": "+
          (hasTemp(i,millis())?String(temperature[i],1)+" C":"Н/Д"),
          28+(i%2)*132,100+(i/2)*40,V14_TEXT);
      v14Tiny("Поправки: Настройки",28,185,V14_MUTED);break;
    case 4:
      v14Tiny(WiFi.status()==WL_CONNECTED?"Wi-Fi STA: да":"Wi-Fi STA: нет",28,90,V14_TEXT);
      v14Tiny(googleEnabled?"Google: настроен":"Google: нет",28,133,V14_BLUE);
      v14Tiny("AP: "+WiFi.softAPIP().toString(),28,174,V14_TEXT);break;
    case 5:
      v14Tiny("ESP32-S3 · 240 МГц",28,90,V14_TEXT);
      v14Tiny("RAM: "+String(ESP.getFreeHeap()/1024)+" КБ",28,133,V14_BLUE);
      v14Tiny("Работа: "+String(millis()/60000)+" мин",28,174,V14_TEXT);break;
    case 6:
      v14Tiny("GPIO15 · YF-S401",28,90,V14_TEXT);
      v14Tiny("Импульсы: "+String(flowPulses()),28,133,V14_BLUE);
      v14Tiny("Калибровка: НЕТ",28,174,V14_MUTED);break;
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
    case 7:busaCalcDraw();break;
  }
  tftDirty=false;
}

// No full-screen erase in the live-update path. Only changed numerical fields
// and the narrow clock/flow strips are repainted.
void v14RefreshLive(uint32_t now){
  v13Collect(now);
  v14NetIndicators();
  if(currentPage==5 && tftView && approvedSettingsItem==4){
    // Update uptime and connection flags each second without clearing TFT.
    static uint32_t lastInfoRefresh=0;
    if(!lastInfoRefresh || (uint32_t)(now-lastInfoRefresh)>=1000){
      lastInfoRefresh=now;
      screen->fillRect(15,82,283,109,HQ_DEEP);
      v14SystemInfoRows();
    }
  }
  if(currentPage==3 && v13HistoryCount && 
     v13LastCapture!=busaGraphLastDrawnCapture){
    v14Graphs(false); // chart pixels only; prevents full-screen flashing
    busaGraphLastDrawnCapture=v13LastCapture;
  }
  if(currentPage==0){
    for(int i=0;i<4;i++)
      v14Metric(v13Temp(i,now),21,27+i*36+31,94,hasTemp(i,now),HQ_NAVY);
    v14Metric(v13Flow(),253,73,59,isfinite(collectionMeter.rate()));
    v14Metric(v13Total(),253,142,59,isfinite(collectionMeter.total()));
    busaDrawRelayStrip(180);
  } else if(currentPage==2){
    v14Metric(v13Flow(),14,113,137,isfinite(collectionMeter.rate()));
    v14Metric(v13Total(),174,113,128,isfinite(collectionMeter.total()));
  } else if(currentPage==1 && !editingTemp && !deleteConfirmPending){
    if(!tftView){
      for(int sensor=0;sensor<4;sensor++){
        const int y=25+sensor*45;
        v14Metric(v13Temp(sensor,now),220,y+18,83,hasTemp(sensor,now));
      }
    }else{
      v14Metric(v13Temp(selectedTemp,now),230,46,69,hasTemp(selectedTemp,now));
    }
  }
  String stamp=clockStamp(true);
  String dt=deviceClockValid()?stamp.substring(stamp.length()-5):String("--:--");

  if(dt!=v14ClockLast){
    v14ClockLast=dt;
    screen->fillRect(220,1,100,19,V14_CARD);
    ru.setFont(u8g2_font_6x13_t_cyrillic);
    int tw=ru.getUTF8Width(dt.c_str());
    v14Print(dt,max(220,317-tw),15,V14_TEXT);
  }
}
