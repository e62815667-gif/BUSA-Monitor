#pragma once
// V14 APPROVED LANDSCAPE: 320 x 240 TFT, read-only, six approved tabs.
// This header is included after V13 shared UI helpers in BUSA_Main.h.
// No GPIO, relay, LED or physical output functions are used here.
static const uint8_t v14Order[6]={0,1,2,5,3,4};
// Tab identity maps to original internal page IDs without altering persistence.
static const char* const v14Tabs[6]={"Глав","Т","Отбор","Настр","График","Событ"};
static void v14Tiny(const String &s,int x,int y,uint16_t color=WHITE){
  ru.setFont(u8g2_font_5x7_t_cyrillic);
  ruText(s,x,y,color);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
}
static void v14FootArrow(int x,int dir,uint16_t color){
  int cx=x+10,cy=223;
  if(dir==0)screen->fillTriangle(cx-6,cy,cx+4,cy-6,cx+4,cy+6,color);
  if(dir==1)screen->fillTriangle(cx,cy-6,cx-6,cy+5,cx+6,cy+5,color);
  if(dir==2)screen->fillTriangle(cx,cy+6,cx-6,cy-5,cx+6,cy-5,color);
  if(dir==3)screen->fillTriangle(cx+6,cy,cx-4,cy-6,cx-4,cy+6,color);
}
static void v14Frame(uint32_t now){
  // Rendering system is styled after six approved PNG reference screens.
  hqBackground();
  screen->fillRect(0,0,320,22,HQ_NAVY);
  screen->drawFastHLine(0,21,320,HQ_GLOW);
  v14Tiny("БУСА-1М №001",5,14,HQ_LIGHT);
  v14Tiny(deviceClockValid()?clockStamp(true):"--:--",271,14,HQ_LIGHT);
  hqBulb(255,10,3,WiFi.getMode()!=WIFI_OFF?HQ_LIME:V13_RED);
  for(int i=0;i<6;i++){
    int x=i*53+1;
    bool selected=(v14Order[i]==currentPage);
    screen->fillRoundRect(x,25,51,22,4,selected?HQ_AZURE:HQ_NAVY);
    screen->drawRoundRect(x,25,51,22,4,selected?HQ_SKY:HQ_GLASS);
    if(selected){
      screen->fillRoundRect(x+3,28,45,3,2,HQ_GLOW);
      screen->drawFastHLine(x+7,45,37,HQ_CYAN);
    }
    const String title=v14Tabs[i];
    // Use font metrics for actual Unicode glyph coverage; no UTF-8 slicing.
    ru.setFont(u8g2_font_6x13_t_cyrillic);
    int width=ru.getUTF8Width(title.c_str());
    v13Text(title,x+max(2,(51-width)/2),41,selected?WHITE:HQ_SKY);
  }
  screen->fillRect(0,211,320,29,HQ_NAVY);
  screen->drawFastHLine(0,210,320,HQ_GLOW);
  static const char* const labels[]={"Влево","Вверх","Вниз","Вправо","OK","Назад"};
  for(int i=0;i<6;i++){
    int x=i*53+1;
    screen->fillRoundRect(x+1,214,49,23,4,HQ_DEEP);
    screen->drawRoundRect(x+1,214,49,23,4,i==4?HQ_CYAN:HQ_AZURE);
    if(i<4)v14FootArrow(x,i,HQ_CYAN);
    else if(i==4)hqBulb(x+11,225,3,HQ_LIME);
    else screen->drawLine(x+5,225,x+16,225,HQ_SKY);
    v14Tiny(labels[i],x+18,227,HQ_LIGHT);
  }
}
static void v14MiniRig(){
  hqRig(123,52,113,141); // detailed approved-style illustration
  v14Tiny("МОНИТОР",147,188,HQ_CYAN);
}
static void v14Home(uint32_t now,bool layout){
  static const char* const names[]={"Вода","Колонна","Куб","Верх колонны"};
  static const uint16_t palette[]={V13_CYAN,GREEN,YELLOW,V13_ORANGE};
  if(layout){
    v14MiniRig();
    for(int i=0;i<4;i++){
      int y=52+i*36;
      hqPanel(3,y,116,34);
      hqSymbol(i==0?2:(i==1?1:(i==2?0:1)),11,y+12,7,HQ_CYAN);
      screen->drawFastVLine(4,y+5,23,palette[i]);
      v14Tiny(names[i],20,y+12,GRAY);
    }
    hqPanel(240,52,77,66);
    v14Tiny("ОТБОР",246,64,V13_CYAN);
    v14Tiny("Расход",246,86,GRAY);
    v14Tiny("мл/мин",246,108,GRAY);
    hqPanel(240,124,77,69);
    v14Tiny("СОБРАНО",246,138,V13_CYAN);
    v14Tiny("мл",246,172,GRAY);
    screen->drawRoundRect(3,198,314,11,4,HQ_GLASS);
  }
  for(int i=0;i<4;i++){
    int y=52+i*36;
    screenField(v13Temp(i,now),16,y+30,100,hasTemp(i,now)?GREEN:YELLOW);
  }
  screenField(v13Flow(),248,102,58,YELLOW);
  screenField(v13Total(),248,160,58,YELLOW);
  v14Tiny(String("Wi-Fi: ")+(WiFi.getMode()!=WIFI_OFF?"AP":"—")+
          "  |  K1-K4 заблокированы",9,206,GRAY);
}
static void v14Temperature(uint32_t now,bool layout){
  static const char* names[]={"Вода","Колонна","Куб","Верх колонны"};
  for(int i=0;i<4;i++){
    int x=4+(i%2)*158,y=52+(i/2)*77;
    if(layout){
      hqPanel(x,y,154,73);
      hqSymbol(i%2?1:2,x+134,y+15,8,HQ_CYAN);
      v14Tiny(names[i],x+11,y+13,GRAY);
    }
    // Active temperature is drawn every refresh so highlighting follows 5x4 keys.
    screen->fillRect(x+3,y+3,4,67,i==selectedTemp?HQ_GLOW:HQ_DEEP);
    screenField(v13Temp(i,now),x+12,y+34,133,hasTemp(i,now)?GREEN:YELLOW);
    screenField("Зад:"+v13Ref(refTemp[i]),x+10,y+48,135,GRAY);
    screenField("Пред:"+v13Ref(noticeTemp[i]),x+10,y+66,135,GRAY);
  }
  if(editingTemp){
    screen->fillRoundRect(38,80,244,91,6,PANEL);
    screen->drawRoundRect(38,80,244,91,6,YELLOW);
    v13Text("ВВОД T"+String(selectedTemp+1),49,101,WHITE);
    screenField(editTempBuffer.length()?editTempBuffer:"(очистить)",50,128,206,YELLOW);
    v14Tiny("ENT сохранить | ESC отмена",49,154,GRAY);
  }
}
static void v14Collection(bool layout){
  if(layout){
    hqPanel(4,52,150,64);
    hqSymbol(3,129,68,8,HQ_CYAN);
    hqPanel(161,52,155,64);
    hqSymbol(2,289,68,8,HQ_CYAN);
    v14Tiny("Расход отбора, мл/мин",10,68,V13_CYAN);
    v14Tiny("Объём отбора, мл",167,68,V13_CYAN);
    hqPanel(4,121,312,86);
    v14Tiny("ГРАФИК ОТБОРА",13,135,WHITE);
    v14Tiny("Датчик отбора не подключён",13,170,YELLOW);
    v14Tiny("YF-S401 измеряет только охлаждающую воду",13,191,GRAY);
  }
  screenField("НЕТ ДАННЫХ",12,97,137,YELLOW);
  screenField("НЕТ ДАННЫХ",169,97,138,YELLOW);
}
static void v14Graphs(bool layout){
  if(layout){
    hqPanel(4,52,312,154);
    v14Tiny("ГРАФИКИ T1–T4 · реальные измерения",12,65,V13_CYAN);
  }
  bool has=false;
  for(int i=0;i<4;i++)for(int j=0;j<v13HistoryCount;j++)
    if(isfinite(v13History[i][j]))has=true;
  if(!has){
    screen->fillRect(12,75,295,123,PANEL);
    v13Text("Нет измерений для графика",19,144,YELLOW);
  }else{
    v13TemperatureGraph(11,76,297,117);
  }
}
static void v14Events(bool layout){
  if(layout){
    hqPanel(4,52,312,155);
    v14Tiny("ЖУРНАЛ СОБЫТИЙ",14,66,V13_CYAN);
  }
  screen->fillRect(11,73,298,128,PANEL);
  int count=min((int)v13EventCount,5);
  for(int i=0;i<count;i++){
    int y=89+i*24;
    screen->fillCircle(19,y-3,3,i?GREEN:V13_CYAN);
    String txt=v13Events[i];
    if(txt.length()>55)txt=txt.substring(0,55);
    v14Tiny(txt,28,y,WHITE);
    if(i<4)screen->drawFastHLine(16,y+5,285,BLUE);
  }
}
static void v14Settings(bool layout){
  static const char* const names[]={"Температуры","Отбор жидкости","Wi-Fi","Экран и звук","Система","О системе"};
  for(int i=0;i<6;i++){
    int x=4+(i%2)*158,y=52+(i/2)*53;
    if(layout){hqPanel(x,y,154,49);hqSymbol(i==0?1:i==1?2:i==2?0:i==3?4:i==4?3:5,x+19,y+22,10,HQ_CYAN);}
    screen->fillRect(x+4,y+4,4,40,i==approvedSettingsItem?HQ_CYAN:HQ_DEEP);
    v14Tiny(names[i],x+34,y+27,HQ_LIGHT);
    v14Tiny(">",x+139,y+27,HQ_SKY);
  }
  if(tftView){
    screen->fillRoundRect(54,77,215,99,6,PANEL);
    screen->drawRoundRect(54,77,215,99,6,V13_CYAN);
    v13Text(String(names[approvedSettingsItem]),66,101,WHITE);
    if(approvedSettingsItem==2){
      v14Tiny("AP: "+WiFi.softAPIP().toString(),65,121,GREEN);
      v14Tiny(WiFi.status()==WL_CONNECTED?"STA подключена":"STA нет",65,144,GRAY);
    }else if(approvedSettingsItem==3){
      v14Tiny("TFT: "+String(tftProfileName()),65,121,GRAY);
      v14Tiny("Поворот: "+String(tftRotation),65,144,GRAY);
    }else if(approvedSettingsItem==4){
      v14Tiny("ДИАГНОСТИКА через iPhone",65,121,GRAY);
      v14Tiny("K1-K4 выключены",65,144,YELLOW);
    }else{
      v14Tiny("Только мониторинг",65,121,GRAY);
    }
    v14Tiny("НАЗАД / ESC",65,163,V13_CYAN);
  }
}
void renderLandscape(uint32_t now){
  static uint8_t oldPage=255,oldView=255,oldTab=255,oldSel=255;
  v13Collect(now);
  bool layout=tftDirty||oldPage!=currentPage||oldView!=tftView||
              oldSel!=selectedTemp||oldTab!=approvedSettingsItem;
  if(layout){
    tftFieldCount=0;
    v14Frame(now);
    oldPage=currentPage;oldView=tftView;oldTab=approvedSettingsItem;oldSel=selectedTemp;
    tftDirty=false;
  }
  switch(currentPage){
    case 0:v14Home(now,layout);break;
    case 1:v14Temperature(now,layout);break;
    case 2:v14Collection(layout);break;
    case 3:v14Graphs(layout);break;
    case 4:v14Events(layout);break;
    case 5:v14Settings(layout);break;
    default:break;
  }
  screen->fillRect(270,3,46,15,HQ_NAVY);
  v14Tiny(deviceClockValid()?clockStamp(true):"--:--",272,14,HQ_LIGHT);
}
