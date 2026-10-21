#pragma once
// BUSA-1M-001 V13: approved six-page portrait interface on a passive 240x320 TFT.
// Visual layout based on the approved 20.10.2026 mockup.
// Values are real measurements or explicitly unavailable/uncalibrated.
// This file is included from BUSA_Main.h AFTER shared helpers and globals.
static const uint16_t V13_RED=0xf800, V13_CYAN=0x07ff, V13_ORANGE=0xfd20, V13_PURPLE=0x9a1f;
static float v13History[4][48];
static uint8_t v13HistoryCount=0,v13HistoryHead=0;
static uint32_t v13LastCapture=0;
static String v13Events[7];
static uint8_t v13EventCount=0;
static bool v13EventBoot=false,v13OldWifi=false,v13OldPcf=false,v13OldPca=false,v13OldTemps[4]={false,false,false,false},v13OldFlow=false;
static void v13Log(const String &msg){
  if(v13EventCount<7)v13EventCount++;
  for(int i=(int)v13EventCount-1;i>0;i--)v13Events[i]=v13Events[i-1];
  v13Events[0]=String(millis()/1000)+"с · "+msg;
}
static void v13Collect(uint32_t now){
  if(!v13EventBoot){
    v13EventBoot=true;
    v13Log("БУСА запущена (V13)");
    v13OldWifi=WiFi.status()==WL_CONNECTED;
    v13OldPcf=pcfDetected;v13OldPca=pcaDetected;
    for(int i=0;i<4;i++)v13OldTemps[i]=tempPresent[i];
    v13OldFlow=flowSeen;
  }
  const bool wifi=WiFi.status()==WL_CONNECTED;
  if(wifi!=v13OldWifi){v13Log(wifi?"Wi-Fi STA подключен":"Wi-Fi STA отключен");v13OldWifi=wifi;}
  if(pcfDetected!=v13OldPcf){v13Log(pcfDetected?"PCF8575 найден":"PCF8575 недоступен");v13OldPcf=pcfDetected;}
  if(pcaDetected!=v13OldPca){v13Log(pcaDetected?"PCA9685 найден":"PCA9685 недоступен");v13OldPca=pcaDetected;}
  for(int i=0;i<4;i++){
    if(tempPresent[i]!=v13OldTemps[i]){
      v13Log(String("T")+String(i+1)+(tempPresent[i]?" обнаружен":" не отвечает"));
      v13OldTemps[i]=tempPresent[i];
    }
  }
  if(flowSeen!=v13OldFlow){v13Log("Импульсы расхода зарегистрированы");v13OldFlow=flowSeen;}
  if(v13HistoryCount==0 || elapsed(now,v13LastCapture,5000)){
    v13LastCapture=now;
    for(int i=0;i<4;i++)v13History[i][v13HistoryHead]=hasTemp(i,now)?temperature[i]:NAN;
    v13HistoryHead=(v13HistoryHead+1)%48;
    if(v13HistoryCount<48)v13HistoryCount++;
  }
}
static void v13Text(const String &v,int x,int y,uint16_t c=WHITE){ruText(v,x,y,c);}
static void v13Line(int x,int y,int w,uint16_t c=BLUE){screen->drawFastHLine(x,y,w,c);}
static void v13Panel(int x,int y,int w,int h){
  screen->fillRoundRect(x,y,w,h,5,PANEL);
  screen->drawRoundRect(x,y,w,h,5,BLUE);
}
static void v13Badge(int x,int y,const String &s,uint16_t c){
  screen->fillRoundRect(x,y,35,17,4,PANEL);
  screen->drawRoundRect(x,y,35,17,4,c);
  v13Text(s,x+4,y+12,c);
}
static String v13Temp(int i,uint32_t now){
  return hasTemp(i,now)?String(temperature[i],1)+"°C":"Н/Д";
}
static String v13Ref(float v){return isfinite(v)?String(v,1):String("--");}
static String v13Flow(){
  return flowSeen?String(flowLpm*1000.0f,1):String("Н/Д");
}
static String v13Total(){
  return flowSeen?String(totalWaterLitres()*1000.0f,0):String("Н/Д");
}
static void v13Frame(uint32_t now){
  screen->fillScreen(BG);
  screen->fillRect(0,0,240,28,PANEL);
  v13Line(0,27,240);
  v13Text("БУСА-1М №001",6,18,WHITE);
  screen->fillCircle(229,11,5,WiFi.getMode()!=WIFI_OFF?GREEN:V13_RED);
  // Local STA status and clock; never infer that the AP is connected to the Internet.
  v13Text(deviceClockValid()?clockStamp(true):"--:--",165,18,GRAY);
  const char* tab[]={"Глав","Темп","Отбор","Граф","Событ","Настр"};
  screen->fillRect(0,276,240,44,PANEL);
  v13Line(0,276,240);
  for(int i=0;i<6;i++){
    const int x=i*40;
    if(i==currentPage){
      screen->fillRoundRect(x+2,279,36,39,4,BLUE);
      v13Line(x+3,279,34,V13_CYAN);
    }
    // Compact pictograms are rendered as vector primitives, not Unicode emoji.
    const int cx=x+20,cy=288;
    switch(i){
      case 0: screen->drawTriangle(cx-7,cy,cx,cy-6,cx+7,cy,WHITE);screen->drawRect(cx-5,cy,10,9,WHITE);break;
      case 1: screen->drawRoundRect(cx-3,cy-7,6,16,3,V13_CYAN);screen->fillCircle(cx,cy+6,3,V13_CYAN);break;
      case 2: screen->drawTriangle(cx,cy-8,cx-6,cy+3,cx+6,cy+3,V13_CYAN);break;
      case 3: screen->drawLine(cx-9,cy+8,cx-9,cy-7,WHITE);screen->drawLine(cx-9,cy+8,cx+9,cy+8,WHITE);screen->drawLine(cx-6,cy+4,cx,cy-4,GREEN);screen->drawLine(cx,cy-4,cx+7,cy+1,GREEN);break;
      case 4: screen->drawRect(cx-6,cy-7,12,15,WHITE);screen->drawFastHLine(cx-3,cy-3,7,WHITE);screen->drawFastHLine(cx-3,cy+1,7,WHITE);break;
      default: screen->drawCircle(cx,cy,7,WHITE);screen->fillCircle(cx,cy,3,WHITE);break;
    }
    v13Text(tab[i],x+((i==2||i==4)?5:7),314,i==currentPage?WHITE:GRAY);
  }
  if(tftView && currentPage!=1 && currentPage!=5){
    v13Badge(199,36,"INFO",V13_CYAN);
  }
}
static void v13HomeIllustration(){
  // A compact schematic illustration; not an operational process diagram.
  v13Panel(126,33,110,151);
  screen->drawRoundRect(141,86,46,76,8,GRAY);
  screen->drawLine(153,97,153,155,V13_CYAN);
  screen->drawLine(161,97,161,155,V13_CYAN);
  screen->drawRoundRect(171,47,13,61,4,GRAY);
  screen->drawLine(163,88,163,53,GRAY);
  screen->drawLine(163,53,177,53,GRAY);
  screen->drawLine(184,74,215,74,V13_CYAN);
  screen->drawLine(215,74,215,142,V13_CYAN);
  screen->drawLine(215,142,202,142,V13_CYAN);
  screen->drawRect(194,143,20,18,GRAY);
  screen->fillRect(196,151,16,8,BLUE);
  screen->drawFastHLine(146,164,37,GRAY);
  v13Text("МОНИТОР",145,180,V13_CYAN);
}
static void v13Home(uint32_t now,bool layout){
  static const char* labels[]={"Вода","Колонна","Куб","Верх колонны"};
  static const uint16_t colors[]={V13_CYAN,GREEN,YELLOW,V13_RED};
  if(layout){
    v13HomeIllustration();
    for(int i=0;i<4;i++){
      const int y=34+i*53;
      v13Panel(4,y,116,48);
      screen->fillCircle(14,y+13,4,colors[i]);
      v13Text(labels[i],24,y+17,WHITE);
    }
    v13Panel(125,189,111,64);
    v13Text("РАСХОД",134,205,V13_CYAN);
    v13Text("СОБРАНО",134,234,V13_CYAN);
    v13Line(132,219,96);
    v13Panel(4,252,232,20);
  }
  for(int i=0;i<4;i++){
    const int y=34+i*53;
    screenField(v13Temp(i,now),24,y+34,84,hasTemp(i,now)?GREEN:YELLOW);
  }
  screenField(v13Flow()+" мл/мин",134,218,96,flowSeen?GREEN:YELLOW);
  screenField(v13Total()+" мл",134,248,96,flowSeen?GREEN:YELLOW);
  screenField(String("Wi-Fi ")+(WiFi.getMode()!=WIFI_OFF?"AP OK":"НЕТ")+"   Реле ВЫКЛ",12,267,216,GREEN);
}
static void v13Temperatures(uint32_t now,bool layout){
  static const char* labels[]={"Вода","Колонна","Куб","Верх колонны"};
  if(layout){
    v13Text("ТЕМПЕРАТУРЫ",9,44,WHITE);
    for(int i=0;i<4;i++){
      int y=51+i*53;
      v13Panel(5,y,230,49);
      if(i==selectedTemp)screen->fillRect(7,y+3,3,43,V13_CYAN);
      v13Text(String(i+1)+". "+labels[i],18,y+19,WHITE);
    }
    v13Text("Ent: изменить  F1: сброс",8,273,GRAY);
  }
  for(int i=0;i<4;i++){
    int y=51+i*53;
    screenField(v13Temp(i,now),147,y+19,81,hasTemp(i,now)?GREEN:YELLOW);
    screenField("Зад:"+v13Ref(refTemp[i]),18,y+40,101,GRAY);
    screenField("Пред:"+v13Ref(noticeTemp[i]),127,y+40,103,GRAY);
  }
  if(editingTemp){
    screen->fillRoundRect(9,141,224,68,7,PANEL);
    screen->drawRoundRect(9,141,224,68,7,YELLOW);
    v13Text("ВВОД T"+String(selectedTemp+1)+(selectedField?" Пред":" Зад"),18,161,WHITE);
    screenField(editTempBuffer.length()?editTempBuffer:"(очистить)",20,183,204,YELLOW);
    v13Text("Ent ОК, Esc отмена",18,201,GRAY);
  }
}
static void v13FlowGraph(int x,int y,int w,int h){
  screen->fillRect(x,y,w,h,PANEL);
  screen->drawFastHLine(x,y+h-1,w,GRAY);
  if(!flowSeen)return;
  float ceiling=0.01f;
  for(float v:flowTrace)if(isfinite(v)&&v>ceiling)ceiling=v;
  for(int i=1;i<40;i++){
    int a=(flowTraceHead+i-1)%40,b=(flowTraceHead+i)%40;
    float va=flowTrace[a],vb=flowTrace[b];
    if(!isfinite(va)||!isfinite(vb))continue;
    int xa=x+(i-1)*(w-3)/39,xb=x+i*(w-3)/39;
    int ya=y+h-2-(int)(fminf(va/ceiling,1.0f)*(h-5));
    int yb=y+h-2-(int)(fminf(vb/ceiling,1.0f)*(h-5));
    screen->drawLine(xa,ya,xb,yb,V13_CYAN);
  }
}
static void v13Collection(uint32_t now,bool layout){
  if(layout){
    v13Text("ОТБОР ЖИДКОСТИ",8,43,WHITE);
    v13Panel(5,51,230,52);v13Panel(5,108,230,52);
    v13Text("Текущий расход",15,70,GRAY);
    v13Text("Собрано жидкости",15,127,GRAY);
    v13Panel(5,166,230,68);
    v13Text("График расхода",13,184,V13_CYAN);
    v13Panel(5,238,230,34);
    v13Text("ВОДА · калибровка не выполнена",11,254,YELLOW);
  }
  screenField(v13Flow()+" мл/мин",18,94,207,flowSeen?GREEN:YELLOW);
  screenField(v13Total()+" мл",18,151,207,flowSeen?GREEN:YELLOW);
  v13FlowGraph(12,190,216,36);
  screenField("Импульсов: "+String(flowPulses()),15,269,208,GRAY);
}
static void v13TemperatureGraph(int x,int y,int w,int h){
  screen->fillRect(x,y,w,h,PANEL);
  for(int j=0;j<=4;j++)screen->drawFastHLine(x,y+j*(h-1)/4,w,GRAY);
  if(v13HistoryCount<2)return;
  static const uint16_t colors[]={V13_CYAN,V13_RED,V13_ORANGE,GREEN};
  for(int k=0;k<4;k++){
    for(int i=1;i<v13HistoryCount;i++){
      int a=(v13HistoryHead+48-v13HistoryCount+i-1)%48;
      int b=(v13HistoryHead+48-v13HistoryCount+i)%48;
      if(!isfinite(v13History[k][a])||!isfinite(v13History[k][b]))continue;
      int x1=x+(i-1)*(w-1)/47,x2=x+i*(w-1)/47;
      int y1=y+h-1-(int)(fminf(fmaxf(v13History[k][a],0.0f),125.0f)*(h-1)/125.0f);
      int y2=y+h-1-(int)(fminf(fmaxf(v13History[k][b],0.0f),125.0f)*(h-1)/125.0f);
      screen->drawLine(x1,y1,x2,y2,colors[k]);
    }
  }
}
static void v13Graphs(bool layout){
  if(layout){
    v13Text("ГРАФИКИ",8,43,WHITE);
    v13Panel(5,49,230,31);
    v13Text("Температуры",13,70,V13_CYAN);v13Text("Расход",159,70,V13_CYAN);
    v13Panel(5,86,230,153);
    v13Text("Реальные измерения",15,106,GRAY);
    v13Panel(5,242,230,30);
    v13Text("Без имитации значений",13,261,YELLOW);
  }
  bool haveRealGraph=false;
  for(int k=0;k<4;k++)for(int j=0;j<v13HistoryCount;j++)if(isfinite(v13History[k][j]))haveRealGraph=true;
  if(v13HistoryCount<2||!haveRealGraph){
    screen->fillRect(12,118,216,107,PANEL);
    v13Text("Нет истории измерений",18,168,YELLOW);
  }else{
    v13TemperatureGraph(12,117,216,110);
  }
}
static void v13EventsPage(bool layout){
  if(layout){v13Text("СОБЫТИЯ",9,43,WHITE);v13Panel(5,50,230,222);}
  screen->fillRect(10,56,220,208,PANEL);
  for(int i=0;i<v13EventCount && i<7;i++){
    int y=73+i*27;
    screen->fillCircle(19,y-5,3,i==0?V13_CYAN:GREEN);
    String txt=v13Events[i];
    // Keep rows readable and leave previous events searchable in browser diagnostics.
    // Event text is short enough for this panel; do not split UTF-8 glyphs.
    v13Text(txt,28,y,WHITE);
    if(i<6)v13Line(12,y+6,217,GRAY);
  }
}
static void v13Settings(uint32_t now,bool layout){
  static const char* menu[]={"Температуры","Отбор жидкости","Wi-Fi","Экран и звук","Система","О системе"};
  if(layout){
    v13Text("НАСТРОЙКИ",8,44,WHITE);
    for(int i=0;i<6;i++){
      int y=51+i*36;
      v13Panel(5,y,230,33);
      if(i==approvedSettingsItem)screen->fillRect(7,y+4,3,25,V13_CYAN);
      v13Text(menu[i],17,y+22,WHITE);
      v13Text(">",221,y+21,V13_CYAN);
    }
  }
  if(tftView){
    screen->fillRoundRect(7,74,226,156,5,PANEL);
    screen->drawRoundRect(7,74,226,156,5,V13_CYAN);
    String title=menu[approvedSettingsItem];
    v13Text(title,17,98,WHITE);
    switch(approvedSettingsItem){
      case 2:
        v13Text(String("STA: ")+(WiFi.status()==WL_CONNECTED?"СВЯЗЬ":"НЕТ"),17,132,GRAY);
        v13Text("AP: "+WiFi.softAPIP().toString(),17,157,GREEN);
        break;
      case 3:
        v13Text(String("TFT: ")+tftProfileName(),17,132,GRAY);
        v13Text("Поворот: "+String(tftRotation),17,158,GRAY);
        v13Text("Изменение через iPhone",17,185,YELLOW);
        break;
      case 4:
        v13Text("T1-T4 диагностика",17,132,GRAY);
        v13Text("Реле отключены",17,158,YELLOW);
        v13Text("Тесты через iPhone",17,185,GRAY);
        break;
      case 5:
        v13Text("БУСА-1М с ИИ №001",17,132,GRAY);
        v13Text("Интерфейс V13",17,158,GREEN);
        v13Text("Наблюдение без управления",17,185,YELLOW);
        break;
      default:
        v13Text("Справочный экран",17,143,GRAY);
        break;
    }
    v13Text("Esc: назад",17,214,V13_CYAN);
  }
}
void renderPortrait(uint32_t now){
  static uint8_t oldPage=255,oldView=255,oldSettings=255;
  v13Collect(now);
  const bool layout=tftDirty||oldPage!=currentPage||oldView!=tftView||
                    oldSettings!=approvedSettingsItem;
  if(layout){
    tftFieldCount=0;
    v13Frame(now);
    oldPage=currentPage;oldView=tftView;oldSettings=approvedSettingsItem;
    tftDirty=false;
  }
  switch(currentPage){
    case 0:v13Home(now,layout);break;
    case 1:v13Temperatures(now,layout);break;
    case 2:v13Collection(now,layout);break;
    case 3:v13Graphs(layout);break;
    case 4:v13EventsPage(layout);break;
    case 5:v13Settings(now,layout);break;
    default:break;
  }
  // Update clock without repainting the whole UI.
  screen->fillRect(162,5,60,19,PANEL);
  v13Text(deviceClockValid()?clockStamp(true):"--:--",165,18,WHITE);
}
