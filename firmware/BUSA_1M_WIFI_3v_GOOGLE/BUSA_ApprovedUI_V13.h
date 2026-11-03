#pragma once
extern volatile int googleState,googleInternet,googleHttp;
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
static uint32_t v13LastCollectAt=0;
static void v13Collect(uint32_t now){
  // The display polls at 5 Hz; histories/event state need at most 1 Hz.
  // Keep front-panel responsiveness without repeated Strings and Wi-Fi queries.
  if(v13LastCollectAt && !elapsed(now,v13LastCollectAt,1000))return;
  v13LastCollectAt=now;
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
  if(flowSeen!=v13OldFlow){v13Log("YF-S401: импульсы воды");v13OldFlow=flowSeen;}
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
  return String("Н/Д"); // Collection sensor absent; YF-S401 reports WATER only
}
static String v13Total(){
  return String("Н/Д"); // Do not confuse water pulses with collected liquid
}
static void v13Frame(uint32_t now){
  screen->fillScreen(BG);
  screen->fillRect(0,0,240,28,PANEL);
  v13Line(0,27,240);
  v13Text("БУСА-1М",6,18,WHITE);
  // Portrait: compact W/G/E legend (Wi-Fi, Google, ESP32) to preserve the clock.
  screen->fillCircle(76,11,3,WiFi.status()==WL_CONNECTED?GREEN:V13_RED);
  v13Text("W",81,18,WHITE);
  screen->fillCircle(101,11,3,(googleEnabled&&googleInternet==1&&(googleHttp==200||googleHttp==302))?GREEN:V13_RED);
  v13Text("G",106,18,WHITE);
  screen->fillCircle(125,11,3,WiFi.softAPgetStationNum()>0?GREEN:V13_RED);
  v13Text("E",130,18,WHITE);
  // Local STA status and clock; never infer that the AP is connected to the Internet.
  v13Text(deviceClockValid()?clockStamp(true):"--:--",165,18,GRAY);
  const char* tab[]={"Глав","Темп","Отбор","Граф","Событ","Настр","Диаг"};
  screen->fillRect(0,276,240,44,PANEL);
  v13Line(0,276,240);
  for(int i=0;i<7;i++){
    const int x=i*34;
    if(i==currentPage){
      screen->fillRoundRect(x+2,279,31,39,4,BLUE);
      v13Line(x+3,279,28,V13_CYAN);
    }
    // Compact pictograms are rendered as vector primitives, not Unicode emoji.
    const int cx=x+17,cy=288;
    switch(i){
      case 0: screen->drawTriangle(cx-7,cy,cx,cy-6,cx+7,cy,WHITE);screen->drawRect(cx-5,cy,10,9,WHITE);break;
      case 1: screen->drawRoundRect(cx-3,cy-7,6,16,3,V13_CYAN);screen->fillCircle(cx,cy+6,3,V13_CYAN);break;
      case 2: screen->drawTriangle(cx,cy-8,cx-6,cy+3,cx+6,cy+3,V13_CYAN);break;
      case 3: screen->drawLine(cx-9,cy+8,cx-9,cy-7,WHITE);screen->drawLine(cx-9,cy+8,cx+9,cy+8,WHITE);screen->drawLine(cx-6,cy+4,cx,cy-4,GREEN);screen->drawLine(cx,cy-4,cx+7,cy+1,GREEN);break;
      case 4: screen->drawRect(cx-6,cy-7,12,15,WHITE);screen->drawFastHLine(cx-3,cy-3,7,WHITE);screen->drawFastHLine(cx-3,cy+1,7,WHITE);break;
      default: screen->drawCircle(cx,cy,7,WHITE);screen->fillCircle(cx,cy,3,WHITE);break;
    }
    v13Text(tab[i],x+2,314,i==currentPage?WHITE:GRAY);
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
  screenField(v13Flow()+" мл/мин",134,218,96,YELLOW);
  screenField(v13Total()+" мл",134,248,96,flowSeen?GREEN:YELLOW);
  screenField(String("Wi-Fi ")+(WiFi.getMode()!=WIFI_OFF?"AP OK":"НЕТ")+"   К1–К4 звук",12,267,216,GREEN);
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
    v13Text("UP/DN: выбор ENT:изм. F1:удал.",8,273,GRAY);
  }
  for(int i=0;i<4;i++){
    int y=51+i*53;
    screenField(v13Temp(i,now),147,y+19,81,hasTemp(i,now)?GREEN:YELLOW);
    screenField(String(i==selectedTemp && selectedField==0?">ВКЛ:":"ВКЛ:")+v13Ref(refTemp[i]),18,y+40,101,(i==selectedTemp && selectedField==0)?V13_CYAN:GRAY);
    screenField(String(i==selectedTemp && selectedField==1?">ВЫКЛ:":"ВЫКЛ:")+v13Ref(noticeTemp[i]),127,y+40,103,(i==selectedTemp && selectedField==1)?V13_CYAN:GRAY);
  }
  if(deleteConfirmPending){
    screen->fillRoundRect(9,132,224,85,7,PANEL);
    screen->drawRoundRect(9,132,224,85,7,YELLOW);
    v13Text("УДАЛИТЬ ПОРОГ T"+String(selectedTemp+1)+
      (selectedField?" ВЫКЛ?":" ВКЛ?"),17,155,WHITE);
    v13Text("ENT - удалить",17,181,YELLOW);
    v13Text("ESC - отменить",17,202,GREEN);
  }else if(editingTemp){
    screen->fillRoundRect(9,141,224,68,7,PANEL);
    screen->drawRoundRect(9,141,224,68,7,YELLOW);
    v13Text("ВВОД T"+String(selectedTemp+1)+(selectedField?" ВЫКЛ":" ВКЛ"),18,161,WHITE);
    screenField(editTempBuffer.length()?editTempBuffer:"(пусто)",20,183,204,YELLOW);
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
    v13Text("График отбора",13,184,V13_CYAN);
    v13Panel(5,238,230,34);
    v13Text("Нет датчика отбора",11,254,YELLOW);
  }
  screenField(v13Flow()+" мл/мин",18,94,207,flowSeen?GREEN:YELLOW);
  screenField(v13Total()+" мл",18,151,207,flowSeen?GREEN:YELLOW);
  screen->fillRect(12,188,216,38,PANEL);
  v13Text("Пока нет датчика отбора",15,210,YELLOW);
  screenField("YF-S401 вода: "+String(flowPulses())+" имп.",15,269,208,GRAY);
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
  screen->fillRect(0,32,240,244,PANEL);
  v13Text("ГРАФИКИ",8,45,WHITE);
  for(int j=0;j<2;j++){
    int x=4+j*117;
    screen->fillRoundRect(x,51,113,28,5,graphSubView==j?BLUE:PANEL);
    screen->drawRoundRect(x,51,113,28,5,V13_CYAN);
    v13Text(j==0?"ТЕМПЕРАТУРЫ":"ВОДА И ОТБОР",x+5,70,WHITE);
  }
  screen->fillRoundRect(7,86,226,152,5,0x0000);
  screen->drawRoundRect(7,86,226,152,5,V13_CYAN);
  if(graphSubView==0){
    static const uint16_t colors[]={V13_CYAN,V13_ORANGE,V13_RED,GREEN};
    for(int k=0;k<4;k++){
      const int y=92+k*34;
      if(k)screen->drawFastHLine(11,y-2,218,GRAY);
      v13Text("T"+String(k+1),12,y+13,colors[k]);
      const int last=(v13HistoryHead+47)%48;
      const float latest=v13HistoryCount?v13History[k][last]:NAN;
      v13Text(isfinite(latest)?String(latest,1):String("Н/Д"),35,y+13,WHITE);
      float low=INFINITY,high=-INFINITY;
      for(int i=0;i<v13HistoryCount;i++){
        const int idx=(v13HistoryHead+48-v13HistoryCount+i)%48;
        const float t=v13History[k][idx];
        if(isfinite(t)){low=fminf(low,t);high=fmaxf(high,t);}
      }
      if(!isfinite(low)||!isfinite(high))continue;
      const float pad=fmaxf(0.2f,(high-low)*0.22f);
      low-=pad;high+=pad;
      for(int i=0;i<v13HistoryCount;i++){
        const int idx=(v13HistoryHead+48-v13HistoryCount+i)%48;
        const float v=v13History[k][idx];
        if(!isfinite(v))continue;
        int x=89+(i*136)/47;
        int yt=y+27-(int)(fminf(1.0f,fmaxf(0.0f,(v-low)/(high-low)))*23.0f);
        const int prior=(idx+47)%48;
        if(!i||!isfinite(v13History[k][prior]))screen->drawPixel(x,yt,colors[k]);
        else{
          const int yp=y+27-(int)(fminf(1.0f,fmaxf(0.0f,(v13History[k][prior]-low)/(high-low)))*23.0f);
          screen->drawLine(89+((i-1)*136)/47,yp,x,yt,colors[k]);
        }
      }
    }
    v13Text("Точки через 5 секунд",13,259,GRAY);
  }else{
    v13Text("ВОДА · импульсы/2c",15,108,V13_CYAN);
    const int px=16,py=115,pw=207,ph=40;
    uint32_t maxv=1;
    for(int i=0;i<waterSamplesRecorded;i++){
      const int idx=(flowTraceHead+40-waterSamplesRecorded+i)%40;
      if(waterPulseHistory[idx]>maxv)maxv=waterPulseHistory[idx];
    }
    for(int i=0;i<waterSamplesRecorded;i++){
      const int idx=(flowTraceHead+40-waterSamplesRecorded+i)%40;
      const int x=px+(i*pw)/39;
      const int y=py+ph-1-(int)((uint64_t)waterPulseHistory[idx]*(ph-3)/maxv);
      if(!i)screen->drawPixel(x,y,V13_CYAN);
      else{
        const int prev=(flowTraceHead+40-waterSamplesRecorded+i-1)%40;
        const int yp=py+ph-1-(int)((uint64_t)waterPulseHistory[prev]*(ph-3)/maxv);
        screen->drawLine(px+((i-1)*pw)/39,yp,x,y,V13_CYAN);
      }
    }
    screen->drawFastHLine(12,172,216,GRAY);
    v13Text("ОТБОР · нет датчика",15,191,YELLOW);
    v13Text("Без выдуманных данных",15,217,GRAY);
    v13Text("Вода не откалибрована",15,259,GRAY);
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
 static const char* names[]={"Температура","Wi-Fi","Экран","Google","О системе"};
 if(!tftView){
  if(layout){
   v13Text("НАСТРОЙКИ",8,44,WHITE);
   for(int i=0;i<5;i++){
    const int y=52+i*39;
    v13Panel(5,y,230,35);
    if(i==approvedSettingsItem)screen->fillRect(7,y+3,4,29,V13_CYAN);
    v13Text(names[i],17,y+22,WHITE);
   }
  }
  return;
 }
 // Full child page rather than an overlay across the Settings list.
 if(layout)v13Panel(5,52,230,247);
 v13Text(names[approvedSettingsItem],17,77,WHITE);
 switch(approvedSettingsItem){
  case 0:
   // The full T1–T4 list is visible at all times.
   for(int i=0;i<4;i++){
    const int y=106+i*44;
    if(i==calSensor)screen->fillRect(13,y-15,4,24,V13_CYAN);
    float v=(calEditing && calSensor==i)?calDraft:tempOffset[i];
    v13Text("T"+String(i+1),23,y,WHITE);
    v13Text(String(v>=0?"+":"")+String(v,1)+" °C",103,y,WHITE);
   }
   break;
  case 1:
   switch(wifiUiStage){
    case WIFIUI_SCANNING:
     v13Text("Поиск сети",17,119,WHITE);
     v13Text("Поиск...",17,150,WHITE);break;
    case WIFIUI_NETWORKS:
     v13Text("Поиск сети",17,109,WHITE);
     for(int row=0;row<4;row++){
       int idx=(wifiUiIndex/4)*4+row;
       if(idx>=wifiUiCount)break;
       const int y=134+row*31;
       if(idx==wifiUiIndex)screen->fillRect(14,y-13,3,22,V13_CYAN);
       String name=wifiUiNetworks[idx];
       if(name.length()>31)name=name.substring(0,28)+"...";
       v13Text(name,21,y,WHITE);
     }
     break;
    case WIFIUI_PASSWORD:{
     v13Text("Сеть: "+wifiUiSelectedSsid,17,119,WHITE);
     v13Text(String("Регистр: ")+wifiUiModeCaption(),17,158,WHITE);
     v13Text("Пароль: "+wifiUiPasswordMask(),17,189,WHITE);
     v13Panel(19,215,200,36);
     v13Text("Сохранить",64,240,WHITE);
     break;
    }
    case WIFIUI_CONNECTING:
     v13Text("Подключение...",17,137,WHITE);
     break;
    default:
     v13Text("Поиск сети",17,128,WHITE);
     v13Text(WiFi.status()==WL_CONNECTED?"Wi-Fi подключён":"Нет подключения",17,168,WHITE);
     break;
   }
   break;
  case 2:{
   static const char* themes[]={"Светлая","Тёмная","Рисунок БУСА"};
   v13Text("Выбор темы",17,105,WHITE);
   v13Text(String("Выбрана: ")+themes[tftThemeMode],17,134,WHITE);
   for(int i=0;i<3;i++){
     const int y=167+i*35;
     if(i==themeDraft)screen->fillRect(14,y-14,4,24,V13_CYAN);
     v13Text(themes[i],24,y,WHITE);
   }
   break;
  }
  case 3:
   v13Text(!busaGoogleSavedCredentials()?"Не настроено":
      (googleEnabled?"Подключено":"Отключено"),17,114,WHITE);
   for(int i=0;i<2;i++){
     const int y=153+i*41;
     v13Panel(15,y-21,214,35);
     if(i==googleUiChoice)screen->fillRect(19,y-18,4,26,V13_CYAN);
     v13Text(i==0?"Подключить":"Отключить",29,y+1,WHITE);
   }
   if(!busaGoogleSavedCredentials())
     v13Text("Настройка: приложение",17,253,WHITE);
   break;
  default:{
   const bool googleOk=googleEnabled && googleInternet==1 &&
     googleState==5 && (googleHttp==200 || googleHttp==302);
   const uint32_t secs=millis()/1000;
   v13Text(String("Wi-Fi подключён: ")+(WiFi.status()==WL_CONNECTED?"да":"нет"),17,112,WHITE);
   v13Text(String("Google: ")+(googleOk?"да":"нет"),17,143,WHITE);
   v13Text(String("ESP32 Wi-Fi: ")+((WiFi.getMode()==WIFI_AP || WiFi.getMode()==WIFI_AP_STA)?"да":"нет"),17,174,WHITE);
   v13Text("Прошивка: V15.0",17,205,WHITE);
   v13Text("Работа: "+String(secs/60)+" мин "+String(secs%60)+" сек",17,236,WHITE);
  }
 }
}
static void v13Diagnostics(uint32_t now,bool layout){
 static const char* names[]={"Экран","Клавиатура","Реле","Датчики","Wi-Fi/Google","ESP32","Расходомер"};
 if(layout){
  v13Text("ДИАГНОСТИКА",8,43,WHITE);
  for(int i=0;i<7;i++){
   const int y=48+i*29;v13Panel(5,y,230,27);
   if(i==diagModule)screen->fillRect(7,y+3,3,21,V13_CYAN);
   v13Text(names[i],17,y+19,WHITE);
  }
 }
 if(tftView){
  screen->fillRoundRect(12,99,220,117,6,PANEL);
  screen->drawRoundRect(12,99,220,117,6,V13_CYAN);
  v13Text(names[diagModule],23,125,WHITE);
  switch(diagModule){
   case 0:v13Text("ST7789 · OK",23,157,WHITE);break;
   case 1:v13Text(pcfDetected?"PCF8575 · есть":"PCF8575 · нет",23,157,WHITE);break;
   case 2:v13Text("K1-K4 · ЗВУК",23,157,WHITE);break;
   case 3:v13Text("T1-T4 · DS18B20",23,157,WHITE);break;
   case 4:v13Text("Wi-Fi / Google",23,157,WHITE);break;
   case 5:v13Text("ESP32-S3",23,157,WHITE);break;
   default:v13Text(String("YF: ")+flowPulses()+" имп.",23,157,WHITE);break;
  }
  v13Text("ESC назад",23,192,WHITE);
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
    case 6:v13Diagnostics(now,layout);break;
    default:break;
  }
  // Refresh status dots with no full-screen redraw, preserving W/G/E labels.
  screen->fillCircle(76,11,3,WiFi.status()==WL_CONNECTED?GREEN:V13_RED);
  screen->fillCircle(101,11,3,(googleEnabled&&googleInternet==1&&(googleHttp==200||googleHttp==302))?GREEN:V13_RED);
  screen->fillCircle(125,11,3,WiFi.softAPgetStationNum()>0?GREEN:V13_RED);
  // Update clock without repainting the whole UI.
  screen->fillRect(162,5,60,19,PANEL);
  v13Text(deviceClockValid()?clockStamp(true):"--:--",165,18,WHITE);
}
