#pragma once
/*
  BUSA-1M-001 БУСА 1М WiFi 3v-GOOGLE
  ESP32-S3 N8R2 44-pin, TFT GMT028-05 V1.1 320x240 landscape, monitoring-only.
  LOW-VOLTAGE MONITORING ONLY. DO NOT DRIVE RELAYS OR POWER OUTPUTS.
  Approved hardware scope: mains power controller and motorized valve are
  REMOVED from this project; they are operated manually, outside ESP32.
  Approved informational section "Отбор" is retained without actuator control.
  Real 4 x DS18B20 + YF-S401 cooling-water pulse readings, no simulated values.
  Hardware compatibility of TFT controller, PCF8575 keypad and pulse output 
  must be verified on a powered-off low-voltage bench before connecting.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <WebServer.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <esp_system.h>
#include <Wire.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_ILI9341.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <math.h>

constexpr uint8_t TPIN[4]={38,39,40,41};
constexpr uint8_t KPIN[4]={7,6,5,4}; // Reference only: never configured as outputs
constexpr uint8_t BUS_SDA=8,BUS_SCL=9;
constexpr uint8_t TFT_CS=10,TFT_MOSI=11,TFT_SCK=12,TFT_DC=13,TFT_RST=14;
constexpr uint8_t FLOW_GPIO=15; // INPUT ONLY, signal level must be <=3.3 V
constexpr bool RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false;
static_assert(!RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,"Physical outputs must stay locked");
static_assert(FLOW_GPIO==15 && TPIN[0]==38 && TPIN[3]==41 && BUS_SDA==8 && BUS_SCL==9,"GPIO changed");
const char VERSION[]="БУСА 1М WiFi 5v-MANUAL-ONLY";

OneWire ow1(TPIN[0]),ow2(TPIN[1]),ow3(TPIN[2]),ow4(TPIN[3]);
DallasTemperature ds1(&ow1),ds2(&ow2),ds3(&ow3),ds4(&ow4);
DallasTemperature* sensors[4]={&ds1,&ds2,&ds3,&ds4};
float temperature[4]={NAN,NAN,NAN,NAN};
bool tempPresent[4]={false,false,false,false};
uint32_t tempAt[4]={0,0,0,0},tempCycle=0,tempRequestAt=0;
bool tempConverting=false;
bool elapsed(uint32_t now,uint32_t then,uint32_t ms){return (uint32_t)(now-then)>=ms;}
void sampleTemperatures(uint32_t now){
  if(tempConverting && elapsed(now,tempRequestAt,825)){
    for(int i=0;i<4;i++){
      float v=sensors[i]->getTempCByIndex(0);
      if(tempPresent[i] && isfinite(v) && v>=-55 && v<=125 && v!=DEVICE_DISCONNECTED_C){
        temperature[i]=v;tempAt[i]=now;
      }else{temperature[i]=NAN;tempAt[i]=0;}
    }
    tempConverting=false;
  }
  if(!tempConverting && (tempCycle==0||elapsed(now,tempCycle,2500))){
    tempCycle=now;
    for(int i=0;i<4;i++){
      sensors[i]->begin();
      tempPresent[i]=sensors[i]->getDeviceCount()>0;
      if(tempPresent[i]){
        sensors[i]->setResolution(12);
        sensors[i]->setWaitForConversion(false);
        sensors[i]->requestTemperatures();
      }else {temperature[i]=NAN;tempAt[i]=0;}
    }
    tempConverting=true;tempRequestAt=now;
  }
  for(int i=0;i<4;i++)if(tempAt[i] && elapsed(now,tempAt[i],7000)){
    temperature[i]=NAN;tempAt[i]=0;
  }
}
bool hasTemp(int n,uint32_t now){
  return tempAt[n] && !elapsed(now,tempAt[n],7000) && isfinite(temperature[n]);
}

// YF-S401 pulse reader for ordinary cooling WATER only. No process control.
// 98Hz per litre/minute is a provisional example, NOT factory-calibrated for
// this individual sensor. Range and pulse format depend on sensor revision.
// Never interpret 0 pulses as proof of cooling water or as a safety interlock.
portMUX_TYPE flowMux=portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pulseCounter=0;
void IRAM_ATTR flowISR(){
  portENTER_CRITICAL_ISR(&flowMux);
  ++pulseCounter;
  portEXIT_CRITICAL_ISR(&flowMux);
}
uint32_t flowPulses(){
  uint32_t n;
  portENTER_CRITICAL(&flowMux);n=pulseCounter;portEXIT_CRITICAL(&flowMux);
  return n;
}
constexpr float FLOW_HZ_PER_LPM=98.0f; // must be calibrated by water-volume test
uint32_t flowCycle=0,flowLastCount=0;
float flowLpm=0.0f,flowTrace[40]={};
bool flowReady=false,flowSeen=false;
uint8_t flowTraceHead=0;
void sampleFlow(uint32_t now){
  if(!flowCycle){flowCycle=now;return;}
  uint32_t dt=now-flowCycle;
  if(dt<2000)return;
  uint32_t count=flowPulses();
  uint32_t diff=count-flowLastCount;
  flowLastCount=count;flowCycle=now;
  flowLpm=(float)diff*60000.0f/(dt*FLOW_HZ_PER_LPM);
  if(diff)flowSeen=true;
  flowReady=true;
  flowTrace[flowTraceHead++%40]=flowLpm;
}
float totalWaterLitres(){return (float)flowPulses()/(FLOW_HZ_PER_LPM*60.0f);}

// I2C devices: PCF8575 keypad is monitored; PCA9685 ACK-only (never PWM).
bool i2cOnline=false,pcfDetected=false,pcaDetected=false;
uint8_t pcfAddr=0;uint32_t lastBusProbe=0;
uint32_t keyCount=0,lastKeyScan=0,pendingAt=0;
int pendingKey=-1;bool keyHeld=false;
String keyMessage="NO DATA";
Preferences prefs;
bool ack(uint8_t address){
  Wire.beginTransmission(address);return Wire.endTransmission()==0;
}
void probeI2c(uint32_t now){
  if(!i2cOnline)return;
  pcfDetected=false;pcfAddr=0;
  for(uint8_t i=0x20;i<=0x27;i++)if(ack(i)){
    pcfDetected=true;pcfAddr=i;break;
  }
  pcaDetected=ack(0x40);lastBusProbe=now;
}
bool pcfWrite(uint16_t v){
  if(!pcfDetected)return false;
  Wire.beginTransmission(pcfAddr);
  Wire.write((uint8_t)v);Wire.write((uint8_t)(v>>8));
  return Wire.endTransmission()==0;
}
bool pcfRead(uint16_t &v){
  if(!pcfDetected||Wire.requestFrom(pcfAddr,(uint8_t)2)!=(uint8_t)2)return false;
  v=(uint8_t)Wire.read();v|=(uint16_t)((uint8_t)Wire.read())<<8;return true;
}
uint8_t currentPage=0; // Six approved UI pages
uint8_t settingsSubTab=0,tftView=0;
bool tftDirty=true;
float refTemp[4]={NAN,NAN,NAN,NAN},noticeTemp[4]={NAN,NAN,NAN,NAN};
// Informational display preferences. Never used for physical outputs.
int selectedTemp=0,selectedField=0;
bool editingTemp=false;
String editTempBuffer="";
bool storeTempReference(int sensor,int field,float value){
  if(sensor<0||sensor>3||field<0||field>1||!isfinite(value)||value<0||value>125)return false;
  String key=String(field?"nt":"rf")+String(sensor);
  if(prefs.putFloat(key.c_str(),value)==0)return false;
  (field?noticeTemp[sensor]:refTemp[sensor])=value;
  return true;
}

// Physical faceplate, row-major by user photo. Exact nine-wire matrix
// connection must still be confirmed during keypad diagnostic testing.
const char* const KEY_CAPS[20]={
 "F1","F2","#","*","1","2","3","UP","4","5","6","DOWN",
 "7","8","9","ESC","LEFT","0","RIGHT","ENT"
};
void keypadAction(int k){
  if(editingTemp){
    if(k==0){if(editTempBuffer.length())editTempBuffer.remove(editTempBuffer.length()-1);} // F1 delete
    else if(k==1){if(editTempBuffer.indexOf('.')<0)editTempBuffer+='.';} // F2 decimal point
    else if(k==15){editingTemp=false;editTempBuffer="";} // Esc cancel
    else if(k==19){ // Ent save informational value
      if(editTempBuffer.length() && storeTempReference(selectedTemp,selectedField,editTempBuffer.toFloat())){
        editingTemp=false;editTempBuffer="";
      }else keyMessage="Проверь значение 0..125";
    }else{
      int digit=-1;
      const int digitKey[10]={17,4,5,6,8,9,10,12,13,14};
      for(int d=0;d<10;d++)if(k==digitKey[d])digit=d;
      if(digit>=0 && editTempBuffer.length()<7)editTempBuffer+=String(digit);
    }
    tftDirty=true;return;
  }
  if(currentPage==1){ // Temperatures page: choose sensor/field, Ent edits
    if(k==7)selectedTemp=(selectedTemp+3)%4;
    else if(k==11)selectedTemp=(selectedTemp+1)%4;
    else if(k==2||k==16)selectedField=0; // # = reference
    else if(k==3||k==18)selectedField=1; // * = notice
    else if(k==19){
      editingTemp=true;
      float v=selectedField?noticeTemp[selectedTemp]:refTemp[selectedTemp];
      editTempBuffer=isfinite(v)?String(v,1):String("");
    }
    else if(k==15){currentPage=0;tftView=0;}
    else if(k==1){currentPage=4;tftView=0;} // F2 diagnostics
    else if(k==0){} // F1 = delete, no input outside editing
    else if(k==4||k==17){currentPage=0;tftView=0;}
    else if(k==6){currentPage=2;tftView=0;}
    else if(k==8){currentPage=3;tftView=0;}
    else if(k==9){currentPage=4;tftView=0;}
    else if(k==10){currentPage=5;tftView=0;}
    else if(k==12){currentPage=5;settingsSubTab=0;tftView=0;}
    else if(k==13){currentPage=5;settingsSubTab=1;tftView=0;}
    else if(k==14){currentPage=4;tftView=0;}
    tftDirty=true;return;
  }
  if(k==7){tftView=0;} // Up
  else if(k==11){tftView=1;} // Down: secondary view
  else if(k==2||k==16){ // # or LEFT
    if(currentPage==5)settingsSubTab=(settingsSubTab+3)%4;
    else currentPage=(currentPage+5)%6;
    tftView=0;
  }else if(k==3||k==18){ // * or RIGHT
    if(currentPage==5)settingsSubTab=(settingsSubTab+1)%4;
    else currentPage=(currentPage+1)%6;
    tftView=0;
  }else if(k==15){currentPage=0;tftView=0;} // Esc
  else if(k==19){if(currentPage==5)settingsSubTab=(settingsSubTab+1)%4;tftView=0;} // Ent
  else if(k==1){currentPage=4;tftView=0;} // F2 diagnostics
  else if(k==17||k==4){currentPage=0;tftView=0;} // 0, 1 Home
  else if(k==5){currentPage=1;selectedTemp=0;tftView=0;} // 2
  else if(k==6){currentPage=2;tftView=0;} // 3
  else if(k==8){currentPage=3;tftView=0;} // 4
  else if(k==9){currentPage=4;tftView=0;} // 5
  else if(k==10){currentPage=5;tftView=0;} // 6
  else if(k==12){currentPage=5;settingsSubTab=0;tftView=0;} // 7
  else if(k==13){currentPage=5;settingsSubTab=1;tftView=0;} // 8
  else if(k==14){currentPage=4;tftView=0;} // 9
  // F1 is Delete only when editing a number.
  tftDirty=true;
}

void pollKeys(uint32_t now){
  if(!pcfDetected||!elapsed(now,lastKeyScan,70))return;
  lastKeyScan=now;
  // Hardware 5 rows P0..P4, 4 columns P5..P8 is provisional.
  int pressed=-1;
  for(uint8_t row=0;row<5;row++){
    if(!pcfWrite((uint16_t)(0xffffu & ~(1u<<row))))return;
    delayMicroseconds(150);
    uint16_t data=0xffff;
    if(!pcfRead(data))return;
    for(uint8_t c=0;c<4;c++){
      if(!(data & (1u<<(5+c))))pressed=(pressed==-1)?(row*4+c):-2;
    }
  }
  pcfWrite(0xffff);
  if(pressed!=pendingKey){pendingKey=pressed;pendingAt=now;keyHeld=false;}
  if(pressed>=0 && !keyHeld && elapsed(now,pendingAt,65)){
    keyMessage=String(KEY_CAPS[pressed])+" (R"+String(pressed/4+1)+
      " C"+String(pressed%4+1)+")";
    ++keyCount;keyHeld=true;
    keypadAction(pressed);
  }
}

Adafruit_ST7789 screenSt(TFT_CS,TFT_DC,TFT_RST);
Adafruit_ILI9341 screenIli(TFT_CS,TFT_DC,TFT_RST);
Adafruit_GFX* screen=nullptr;
U8G2_FOR_ADAFRUIT_GFX ru;
uint8_t tftProfile=1;
bool tftStarted=false;
const uint16_t BG=0x080f,PANEL=0x10a3,BLUE=0x1c9f,GREEN=0x07e0,WHITE=0xffff,
               YELLOW=0xffe0,GRAY=0xbdf7;
const char* pages[6]={"Главная","Температуры","Отбор","Графики","События","Настройки"};
// Defined below with the web server; needed by the TFT renderer.
extern String stationSsid,otaMessage,nearbyNames[5];
extern uint8_t nearbyCount;
void ruText(const String &v,int x,int y,uint16_t color=WHITE){
  ru.setForegroundColor(color);
  ru.setCursor(x,y);
  ru.print(v);
}

void panelAt(int x,int y,int w,int h){
  screen->fillRoundRect(x,y,w,h,5,PANEL);
  screen->drawRoundRect(x,y,w,h,5,BLUE);
}
void screenField(const String &v,int x,int y,int w,uint16_t color=WHITE){
  screen->fillRect(x,y-13,w,16,PANEL);
  ruText(v,x,y,color);
}
void startScreen(){
  Serial0.println("TFT: HW SPI landscape; controller model unverified");
  SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS);
  SPI.setFrequency(16000000); // moderate stable starting frequency
  if(tftProfile==2){screenIli.begin();screen=&screenIli;}
  else{screenSt.init(240,320);screen=&screenSt;}
  screen->setRotation(1); // 320x240 landscape, both supported profiles
  screen->fillScreen(BG);
  ru.begin(*screen);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
  ru.setFontMode(1);
  tftStarted=true;tftDirty=true;
  Serial0.println("TFT: landscape init done");
}
void renderScreen(uint32_t now){
  if(!tftStarted||!screen)return;
  static uint8_t drawnPage=255,drawnTab=255,drawnView=255;
  bool layout=tftDirty||drawnPage!=currentPage||
     drawnTab!=settingsSubTab||drawnView!=tftView;
  if(layout){
    screen->fillScreen(BG); // ONLY on real navigation, not on timed updates
    panelAt(3,2,314,28);
    ruText(String("БУСА №001  ·  ")+pages[currentPage],9,21,BLUE);
    drawnPage=currentPage;drawnTab=settingsSubTab;drawnView=tftView;
    tftDirty=false;
    if(currentPage==0||currentPage==1){
      if(tftView==0){
        const char* names[]={"Вода","Колонна","Куб","Верх колонны"};
        for(int i=0;i<4;i++){
          int x=(i%2)?163:4,y=(i/2)?112:35;
          panelAt(x,y,153,72);ruText(String(currentPage==1&&i==selectedTemp?"> ":"")+names[i],x+9,y+18,GRAY);
        }
      }else{
        panelAt(4,36,312,162);
        ruText("Диагностика температур DS18B20",12,56,BLUE);
        ruText("Только реальные показания датчиков",12,169,GRAY);
      }
    }else if(currentPage==2){
      panelAt(4,37,312,161);
      ruText("Отбор вручную. Датчика нет",12,57,YELLOW);
      ruText("Только контроль воды YF-S401",12,94,BLUE);
      ruText("Импульсы воды",12,145,GRAY);
      ruText("Калибровка расхода не проведена",12,177,YELLOW);
    }else if(currentPage==3){
      panelAt(4,37,312,161);
      ruText("График расхода воды (история)",12,57,BLUE);
      screen->drawFastHLine(18,169,282,GRAY);
    }else if(currentPage==4){
      panelAt(4,37,312,161);
      ruText("Диагностика входов и модулей",12,57,BLUE);
    }else{
      panelAt(4,37,312,161);
      const char* tabs[]={"Wi-Fi","Google","Обновление","Модули"};
      ruText(String("Настройки / ")+tabs[settingsSubTab],12,59,BLUE);
    }
    screen->drawFastHLine(0,217,320,BLUE);
    ruText("# / *: вкладки   UP/DOWN: страницы",5,231,GRAY);
  }
  if(currentPage<=1){
    if(tftView==0){
      for(int i=0;i<4;i++){
        int x=(i%2)?163:4,y=(i/2)?112:35;
        screenField(hasTemp(i,now)?String(temperature[i],1)+" C":"НЕТ ДАННЫХ",
             x+9,y+41,135,hasTemp(i,now)?GREEN:YELLOW);
        const float r=refTemp[i],a=noticeTemp[i];
        screenField(String(selectedField==0?"Зад:":"Пред:")+
                    (isfinite(selectedField==0?r:a)?String(selectedField==0?r:a,1):String("--")),
                    x+9,y+61,135,GRAY);
      }
      if(editingTemp){
         screenField(String("РЕДАКТ T")+String(selectedTemp+1)+
           (selectedField?" Пред ":" Зад ")+editTempBuffer,11,202,300,YELLOW);
      }else if(currentPage==1){
         screenField("T1-T4: UP/DOWN  #/\x2a: поле  Ent: ввод",11,202,300,GRAY);
      }else{
         screenField(String("Вода: ")+(flowReady?String(flowLpm,2)+" л/мин":"--"),
                    11,202,300,flowReady?GREEN:YELLOW);
      }
    }else{
      for(int i=0;i<4;i++){
        screenField(String("T")+String(i+1)+" GPIO"+String(TPIN[i])+
           (tempPresent[i]?" : НАЙДЕН":" : НЕТ ДАТЧИКА"),
           14,83+i*21,286,tempPresent[i]?GREEN:YELLOW);
      }
    }
  }else if(currentPage==2){
    screenField(String("Расход: ")+(flowReady?String(flowLpm,2)+" л/мин":"--"),
                12,119,292,flowReady?GREEN:YELLOW);
    screenField(String("Импульсы: ")+String(flowPulses()),12,161,292,GRAY);
  }else if(currentPage==3){
    screen->fillRect(17,68,287,100,PANEL); // repaint chart, not whole TFT
    for(uint8_t i=1;i<40;i++){
      uint8_t a=(flowTraceHead+i-1)%40,b=(flowTraceHead+i)%40;
      int x1=20+(i-1)*7,x2=20+i*7;
      int y1=164-(int)fminf(88.0f,flowTrace[a]*19.0f);
      int y2=164-(int)fminf(88.0f,flowTrace[b]*19.0f);
      screen->drawLine(x1,y1,x2,y2,GREEN);
    }
    screenField("Только импульсы: не защита",12,187,292,YELLOW);
  }else if(currentPage==4){
    screenField(String("PCF8575: ")+(pcfDetected?"НАЙДЕН":"НЕТ"),12,84,292,pcfDetected?GREEN:YELLOW);
    screenField(String("PCA9685: ")+(pcaDetected?"НАЙДЕН":"НЕТ"),12,107,292,pcaDetected?GREEN:YELLOW);
    screenField(String("Нажатий: ")+String(keyCount),12,130,292,GRAY);
    screenField(String("Последняя: ")+keyMessage,12,154,292,GRAY);
    screenField(String("Аптайм: ")+String(now/1000)+" сек",12,181,292,GRAY);
  }else if(settingsSubTab==0){
    screenField(String("STA: ")+(WiFi.status()==WL_CONNECTED?"ПОДКЛЮЧЕНО":"ЛОКАЛЬНО"),12,88,295,GREEN);
    screenField(String("SSID: ")+stationSsid.substring(0,29),12,113,295,GRAY);
    screenField(String("IP: ")+(WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():"--"),12,138,295,GRAY);
    screenField("Подключение: веб-приложение",12,175,295,BLUE);
  }else if(settingsSubTab==1){
    screenField("Google: проверка в веб-приложении",12,91,295,GRAY);
    screenField("Настройка: Настройки / Google",12,122,295,BLUE);
    screenField("Реле заблокированы",12,161,295,YELLOW);
  }else if(settingsSubTab==2){
    screenField("OTA: только локальный APP BIN",12,91,295,BLUE);
    screenField(otaMessage.substring(0,37),12,123,295,GRAY);
    screenField("Не использовать merged.bin",12,159,295,YELLOW);
  }else{
    screenField(String("PCF8575: ")+(pcfDetected?"НАЙДЕН":"НЕТ"),12,91,295,pcfDetected?GREEN:YELLOW);
    screenField(String("PCA9685: ")+(pcaDetected?"НАЙДЕН":"НЕТ"),12,118,295,pcaDetected?GREEN:YELLOW);
    screenField("Ручное управление - вне ESP32",12,160,295,GRAY);
  }
}

WebServer server(80);
bool otaActive=false,otaSuccess=false,otaRestart=false,otaWriteAllowed=false;
uint32_t otaBytes=0,otaRestartAt=0,otaTokenIssued=0,startedAt=0;
String otaToken="",otaMessage="Ожидание",nearbyNames[5];
uint8_t nearbyCount=0;
bool screenAttempted=false;
bool apClient(){return server.client().localIP()==WiFi.softAPIP();}

String apPassword="";
bool wifiRunning=false;
uint32_t lastRender=0,lastUart=0,lastStationCheck=0;
String consoleLine="",stationSsid="",stationPass="";
String quoteJson(const String &x){
  String out="\"";
  for(unsigned i=0;i<x.length();i++){
    char c=x[i];
    if(c=='\"'||c=='\\'){out+='\\';out+=c;}
    else if(c=='\n')out+="\\n";
    else if(c=='\r')out+="\\r";
    else out+=c;
  }
  return out+"\"";
}
String otaJSON(); // forward declaration
String cloudJSON(); // forward declaration; read-only Google status
String statusJSON(){
  uint32_t now=millis();
  String s="{\"device\":\"BUSA-1M-001\",\"firmware\":\""+String(VERSION)+"\"";
  s+=",\"mode\":\"LOW_VOLTAGE_MONITOR_ONLY\",\"relays_enabled\":false,\"pwm_enabled\":false";
  s+=",\"configuration\":{\"manual_external_controls\":true,\"mains_control_in_project\":false,\"motorized_valve_in_project\":false}";
  s+=",\"ui_page\":"+String(currentPage+1)+",\"ui_subtab\":"+String(settingsSubTab)+",\"tft_view\":"+String(tftView)+",\"uptime_ms\":"+String(now);
  s+=",\"display_references\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"target\":"+(isfinite(refTemp[i])?String(refTemp[i],1):String("null"));
    s+=",\"notice\":"+(isfinite(noticeTemp[i])?String(noticeTemp[i],1):String("null"))+"}";
  }
  s+="]";
  s+=",\"temperatures\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"name\":"+quoteJson(String(i==0?"Вода":i==1?"Колонна":i==2?"Куб":"Верх колонны"));
    s+=",\"gpio\":"+String(TPIN[i])+",\"present\":"+(tempPresent[i]?"true":"false");
    s+=",\"c\":"+(hasTemp(i,now)?String(temperature[i],2):String("null"))+"}";
  }
  s+="],\"water\":{\"gpio\":15,\"pulses\":"+String(flowPulses());
  s+=",\"lpm\":"+(flowReady?String(flowLpm,3):String("null"));
  s+=",\"total_l_estimate\":"+String(totalWaterLitres(),3);
  s+=",\"calibrated\":false,\"seen_pulses\":"+(flowSeen?String("true"):String("false"))+"}";
  s+=",\"collection\":{\"sensor_present\":false,\"ml_per_min\":null,\"total_ml\":null}";
  s+=",\"modules\":{\"pcf8575\":"+(pcfDetected?String("true"):String("false"));
  s+=",\"pca9685_ack_only\":"+(pcaDetected?String("true"):String("false"));
  s+=",\"tft_attempted\":"+(tftStarted?String("true"):String("false"));
  s+=",\"last_key\":"+quoteJson(keyMessage)+"}";
  s+=",\"ap\":{\"enabled\":"+(wifiRunning?String("true"):String("false"));
  s+=",\"ip\":"+quoteJson(WiFi.softAPIP().toString())+"}";
  s+=",\"ota\":"+otaJSON();
  s+=",\"bluetooth_enabled\":false";
  s+=",\"google\":"+cloudJSON();
  s+=",\"station\":{\"ssid\":"+quoteJson(stationSsid);
  s+=",\"connected\":"+(WiFi.status()==WL_CONNECTED?String("true"):String("false"));
  s+=",\"ip\":"+quoteJson(WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():String(""))+"}";
  s+="}";
  return s;
}
// Manual local-only OTA; cloud/automatic OTA is NOT enabled.
String otaJSON(){
 return String("{\"active\":")+(otaActive?"true":"false")
   +",\"success\":"+(otaSuccess?"true":"false")
   +",\"bytes\":"+String(otaBytes)+",\"message\":"+quoteJson(otaMessage)+"}";
}
void uploadOTA(){
 HTTPUpload &u=server.upload();
 if(u.status==UPLOAD_FILE_START){
  otaWriteAllowed=false;otaActive=false;otaSuccess=false;otaBytes=0;
  if(!apClient()||otaToken.length()<16||(uint32_t)(millis()-otaTokenIssued)>90000
     ||server.header("X-Busa-Token")!=otaToken||!u.filename.endsWith(".bin")){
    otaMessage="Отказ: локальная авторизация / формат BIN";return;
  }
  otaToken="";
  if(!Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH)){
    otaMessage="Не найден OTA-раздел";Update.printError(Serial0);return;
  }
  otaWriteAllowed=true;otaActive=true;otaMessage="Записывается";
 }else if(u.status==UPLOAD_FILE_WRITE){
  if(!otaWriteAllowed)return;
  if(otaBytes==0 && (u.currentSize<4||u.buf[0]!=0xe9)){
    otaMessage="Неверный образ APP BIN";Update.abort();otaActive=false;otaWriteAllowed=false;return;
  }
  if(Update.write(u.buf,u.currentSize)!=u.currentSize){
    otaMessage="Ошибка записи";Update.printError(Serial0);
    Update.abort();otaActive=false;otaWriteAllowed=false;return;
  }
  otaBytes+=u.currentSize;
 }else if(u.status==UPLOAD_FILE_END){
  if(!otaWriteAllowed)return;
  otaSuccess=Update.end(true);otaActive=false;otaWriteAllowed=false;
  otaMessage=otaSuccess?"Успешно, перезагрузка":"BIN не прошёл проверку";
  if(!otaSuccess)Update.printError(Serial0);
  if(otaSuccess){otaRestart=true;otaRestartAt=millis();}
 }else if(u.status==UPLOAD_FILE_ABORTED){
  if(otaWriteAllowed)Update.abort();
  otaWriteAllowed=false;otaActive=false;otaMessage="Отменено";
 }
}
#include "BUSA_Google.h"
const char WEB_PAGE[] PROGMEM=R"HTMLBUSA(
<!doctype html><html lang="ru"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>БУСА-1М №001</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{font:15px system-ui;margin:0;padding:0 12px 20px;background:#061323;color:#d0efff}header{position:sticky;top:0;background:#091c31;padding:14px 0 9px;border-bottom:1px solid #237fc5}h1{font-size:20px;color:#64c6ff;margin:0 0 10px}h2{font-size:16px;color:#81caff;margin:2px 0 12px}nav{display:flex;overflow-x:auto;gap:6px;padding-bottom:4px}button{font:inherit;cursor:pointer;border:1px solid #258bcc;background:#123d61;color:#fff;border-radius:8px;padding:10px}nav button{font-size:12px;white-space:nowrap}nav .on{background:#2379b1;border-color:#7dc9ff}main{max-width:680px;margin:auto}.card{background:#09233c;border:1px solid #2383c7;border-radius:12px;padding:13px;margin:11px 0}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.tile{border:1px solid #265a80;border-radius:9px;padding:9px}.tile small{display:block;color:#9fbfd0}.tile strong{display:block;color:#64f2a2;margin:5px 0;font-size:20px}.row{display:flex;justify-content:space-between;border-bottom:1px solid #204664;padding:9px 0;gap:10px}.row:last-child{border:0}.green{color:#70e99b}.warn{color:#f4cc74}.hint{color:#97b4c9;font-size:12px}.page[hidden],.settingCard[hidden]{display:none}.settingsTabs{display:grid;grid-template-columns:1fr 1fr;gap:5px;margin:12px 0}.settingsTabs button.on{background:#2379b1;border-color:#7dc9ff}input,select{width:100%;background:#102f4b;border:1px solid #2d84bd;color:#fff;font:inherit;padding:12px;border-radius:8px}label{display:block;margin:12px 0 5px}.full{width:100%;margin:12px 0}#response{color:#f3d783;white-space:pre-wrap}canvas{width:100%;height:170px;border:1px solid #28618f;border-radius:8px}
</style></head><body>
<header><h1>БУСА-1М с ИИ · №001</h1><nav>
<button data-page="0" class="on">Главная</button><button data-page="1">Температуры</button>
<button data-page="2">Отбор</button><button data-page="3">Графики</button><button data-page="4">События</button><button data-page="5">Настройки</button>
</nav><div style="display:flex;gap:7px;padding-top:8px"><button type="button" id="tftUp" style="flex:1">TFT ↑</button><button type="button" id="tftDown" style="flex:1">TFT ↓</button></div></header><main>
<section class="page" data-section="0"><div class="card"><h2>Температуры</h2><div class="grid">
<div class="tile"><small>Вода</small><strong id="t0">—</strong></div><div class="tile"><small>Колонна</small><strong id="t1">—</strong></div>
<div class="tile"><small>Куб</small><strong id="t2">—</strong></div><div class="tile"><small>Верх колонны</small><strong id="t3">—</strong></div></div></div>
<div class="card"><h2>Схема установки</h2><div class="grid"><div class="tile">Куб → Колонна</div><div class="tile">→ Верх колонны</div></div><p class="hint">Схематичное отображение без управления исполнительными механизмами</p></div>
<div class="card"><h2>Отбор</h2><div class="grid"><div class="tile"><small>Расход, мл/мин</small><b class="warn">НЕТ ДАТЧИКА</b></div><div class="tile"><small>Собрано, мл</small><b class="warn">НЕТ ДАННЫХ</b></div></div></div>
<div class="card"><h2>Система</h2><div class="row">Состояние <b class="green">Диагностика</b></div><div class="row">Режим <b class="green">Внешнее управление вручную</b></div><div class="row">Блок 220 В и электрокран <b class="green">ИСКЛЮЧЕНЫ</b></div><div class="row">Выходы ESP32 <b class="warn">ОТКЛЮЧЕНЫ</b></div><div class="row">Wi-Fi <b id="mainwifi">—</b></div><div class="row">Напряжение <b class="warn">НЕТ ДАТЧИКА</b></div></div>
<div class="card"><h2>Вода охлаждения · YF-S401</h2><div class="row">Расход, л/мин <b id="flow">—</b></div><div class="row">Импульсы <b id="pulses">—</b></div><p class="warn">Не откалибровано. 0 импульсов не подтверждает охлаждение.</p></div></section>
<section class="page" data-section="1" hidden><div class="card"><h2>Температуры · DS18B20</h2><div id="temperatures">Ожидание...</div>
<p class="hint">Информационные поля для отображения на TFT. Не управляют реле и не заменяют защиту.</p>
<div class="grid">
<div><label>T1 Вода · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref0"></div>
<div><label>T1 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt0"></div>
<div><label>T2 Колонна · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref1"></div>
<div><label>T2 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt1"></div>
<div><label>T3 Куб · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref2"></div>
<div><label>T3 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt2"></div>
<div><label>T4 Верх · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref3"></div>
<div><label>T4 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt3"></div>
</div><button id="refsSave" type="button" class="full">Сохранить значения отображения</button>
<p id="refsResult" class="hint">Редактирование только через локальную сеть БУСА.</p></div></section>
<section class="page" data-section="2" hidden><div class="card"><h2>Отбор · ручной режим</h2><p class="hint">Блок 220 В и электропривод крана исключены. ESP32 не управляет ими.</p><div class="row">Расход, мл/мин <b class="warn">НЕТ ДАТЧИКА</b></div><div class="row">Собрано, мл <b class="warn">НЕТ ДАННЫХ</b></div><p class="hint">Датчик воды охлаждения не измеряет поток отбора.</p></div><div class="card"><h2>Вода охлаждения</h2><div class="row">Расход, л/мин <b id="flow2">—</b></div><div class="row">Объём, л (оценка) <b id="total">—</b></div></div></section>
<section class="page" data-section="3" hidden><div class="card"><h2>Графики текущей сессии</h2><p>Вода, °C</p><canvas id="graphT" width="600" height="200"></canvas><p>Вода охлаждения, л/мин</p><canvas id="graphF" width="600" height="200"></canvas><p class="hint">Только реальные данные с момента открытия приложения.</p></div></section>
<section class="page" data-section="4" hidden><div class="card"><h2>События</h2><div id="events">Ожидание...</div><p class="hint">Журнал текущей сессии приложения</p></div></section>
<section class="page" data-section="5" hidden><div id="settingsTabs" class="settingsTabs"><button data-stab-btn="0" class="on">Wi-Fi</button><button data-stab-btn="1">Google</button><button data-stab-btn="2">Обновление</button><button data-stab-btn="3">Диагностика</button></div><div class="card settingCard" data-stab="0"><h2>Wi-Fi</h2>
<div id="wstatus">Проверка подключения...</div><button type="button" class="full" id="scan">Найти сети Wi-Fi 2,4 ГГц</button>
<label for="net">Доступные сети Wi-Fi</label><select id="net" required><option value="">Выберите сеть из списка</option></select><p class="hint">Если нужной сети нет, включите на телефоне раздачу интернета и повторите поиск.</p>
<label for="pass">Пароль Wi-Fi</label><input id="pass" type="password" minlength="8" maxlength="63" placeholder="Введите пароль самостоятельно">
<button type="button" class="full" id="save">Сохранить и подключить</button>
<p id="response" role="status"></p><p class="hint">Настройка доступна при подключении к защищённой сети БУСА. Пароль остаётся на контроллере.</p></div>
<div class="card settingCard" data-stab="1" hidden><h2>Google Диск · дистанционный просмотр</h2>
<div class="row">Канал Google <b id="googleState">НЕ НАСТРОЕН</b></div>
<p class="hint">Передача только измерений каждые 30 секунд. Локальная сеть работает без интернета.
Нужен опубликованный Google Apps Script и уникальный токен. Автоматического управления нет.</p>
<label for="googleUrl">Адрес Apps Script (ссылка /exec)</label>
<input id="googleUrl" type="url" placeholder="https://script.google.com/macros/s/.../exec">
<label for="googleToken">Секрет устройства (токен из Apps Script)</label>
<input id="googleToken" type="password" autocomplete="off" placeholder="Вставить на устройстве, не в чат">
<button id="googleSave" type="button" class="full">Сохранить подключение Google</button>
<button id="googleStop" type="button" class="full">Отключить передачу в Google</button>
<p id="googleResult" role="status" class="hint">Отправка выключена до настройки.</p>
</div><div class="card settingCard" data-stab="2" hidden><h2>Обновление ПО · вручную</h2>
<p class="hint">Только через локальную сеть БУСА. Выберите APP BIN для ESP32-S3 N8R2,
НЕ объединённый merged BIN. Только проверенные диагностические сборки.
Перед обновлением отключите внешние исполнительные цепи.</p>
<input type="file" accept=".bin" id="firmwareFile">
<button type="button" class="full" id="manualUpdate">Выбрать и обновить</button>
<p id="updateResult" role="status">Ожидание подтверждения</p>
</div><div class="card settingCard" data-stab="3" hidden><h2>Диагностика</h2><div class="row">Прошивка <b id="ver">—</b></div><div class="row">Время работы <b id="uptime">—</b></div><div class="row">PCF8575 <b id="pcf">—</b></div><div class="row">PCA9685 <b id="pca">—</b></div><div class="row">TFT <b id="tft">—</b></div></div></section>
<footer class="hint">БУСА-1М №001 · только мониторинг · управление внешними устройствами вручную · физические выходы заблокированы</footer></main>
<script>
const get=id=>document.getElementById(id),logs=[],tlog=[],flog=[];let active=0,connected=false,pollBusy=false,settingsActive=0,lastLocalNav=0,refsInit=false;
function message(s){logs.unshift(new Date().toLocaleTimeString('ru-RU')+' · '+s);logs.splice(12);get('events').textContent=logs.join('\n')}
function page(n,send=true){active=n;document.querySelectorAll('[data-section]').forEach(e=>e.hidden=Number(e.dataset.section)!==n);document.querySelectorAll('nav button').forEach(e=>e.classList.toggle('on',Number(e.dataset.page)===n));if(send){lastLocalNav=Date.now();fetch('/api/page?n='+(n+1)).catch(()=>{})}if(n===3)graphs()}
function settingsTab(n,send=true){settingsActive=n;document.querySelectorAll('.settingCard').forEach(e=>e.hidden=Number(e.dataset.stab)!==n);document.querySelectorAll('[data-stab-btn]').forEach(e=>e.classList.toggle('on',Number(e.dataset.stabBtn)===n));if(send){lastLocalNav=Date.now();fetch('/api/settings-tab?n='+n).catch(()=>{})}}
document.querySelectorAll('[data-stab-btn]').forEach(b=>b.onclick=()=>settingsTab(Number(b.dataset.stabBtn)));
get('tftUp').onclick=()=>fetch('/api/tft/scroll?dir=up').catch(()=>{});
get('tftDown').onclick=()=>fetch('/api/tft/scroll?dir=down').catch(()=>{});
get('refsSave').onclick=async()=>{
 let body=new URLSearchParams();
 for(let i=0;i<4;i++){
   for(let k of ['ref','nt']){
     let v=get(k+i).value.trim();
     if(v&&(!Number.isFinite(Number(v))||Number(v)<0||Number(v)>125)){
       get('refsResult').textContent='Допустимы только значения 0..125 °C';return;
     }
     body.set(k+i,v);
   }
 }
 let r=await fetch('/api/display-refs',{method:'POST',body});
 get('refsResult').textContent=await r.text();
 if(r.ok)refsInit=false;
};

document.querySelectorAll('nav button').forEach(e=>e.onclick=()=>page(Number(e.dataset.page)));
function plot(id,values){const c=get(id),x=c.getContext('2d');x.fillStyle='#061b2c';x.fillRect(0,0,600,200);const a=values.filter(v=>v!==null);if(!a.length){x.fillStyle='#d9bd75';x.fillText('НЕТ ДАННЫХ',24,60);return}let lo=Math.min(...a),hi=Math.max(...a);if(hi-lo<.1)hi=lo+.1;x.strokeStyle='#70e99b';x.beginPath();values.forEach((v,i)=>{if(v===null)return;let xx=10+i*580/80,yy=185-(v-lo)/(hi-lo)*165;if(i===0||values[i-1]===null)x.moveTo(xx,yy);else x.lineTo(xx,yy)});x.stroke()}
function graphs(){plot('graphT',tlog);plot('graphF',flog)}
async function update(){if(pollBusy)return;pollBusy=true;try{let r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error(r.status);let s=await r.json();if(!connected)message('Связь с ESP32 восстановлена');connected=true;if(Date.now()-lastLocalNav>1800){if(typeof s.ui_page==='number'&&s.ui_page-1!==active)page(s.ui_page-1,false);if(typeof s.ui_subtab==='number'&&s.ui_subtab!==settingsActive)settingsTab(s.ui_subtab,false);}let names=['Вода','Колонна','Куб','Верх колонны'];
s.temperatures.forEach((t,i)=>{get('t'+i).textContent=t.c===null?'НЕТ ДАННЫХ':Number(t.c).toFixed(1)+' °C';if(i===0){tlog.push(t.c===null?null:Number(t.c));if(tlog.length>80)tlog.shift()}});
get('temperatures').textContent=names.map((v,i)=>v+': '+get('t'+i).textContent).join(' | ');
if(!refsInit && Array.isArray(s.display_references)){
  s.display_references.forEach((r,i)=>{get('ref'+i).value=r.target===null?'':r.target;get('nt'+i).value=r.notice===null?'':r.notice});
  refsInit=true;
}
let f=s.water.lpm===null?null:Number(s.water.lpm);get('flow').textContent=f===null?'НЕТ ДАННЫХ':f.toFixed(2);get('flow2').textContent=get('flow').textContent;get('pulses').textContent=s.water.pulses;get('total').textContent=Number(s.water.total_l_estimate).toFixed(3);
flog.push(f);if(flog.length>80)flog.shift();let ss=s.station.connected?'ПОДКЛЮЧЕНО: '+s.station.ssid+' · '+s.station.ip:(s.station.ssid?'Ожидание подключения: '+s.station.ssid:'Wi-Fi не настроен. Выберите сеть ниже.');get('wstatus').textContent=ss;get('mainwifi').textContent=s.station.connected?'Подключено':'Локальная сеть';
if(s.station.connected&&s.station.ssid){
 let select=get('net'),found=Array.from(select.options).some(o=>o.value===s.station.ssid);
 if(!found)select.add(new Option(s.station.ssid+' (подключена)',s.station.ssid));
 if(!select.value)select.value=s.station.ssid;
}
const cs=s.google;
get('googleState').textContent=!cs.configured?'НЕ НАСТРОЕН':cs.state===5?'ОТВЕТ ПОЛУЧЕН':cs.state===4?'ПЕРЕДАНО, ПРОВЕРЬТЕ ТАБЛИЦУ':cs.state===3?'ОТПРАВКА':cs.state===2?'ВРЕМЯ':cs.state===1?'НЕТ ИНТЕРНЕТА':cs.state===6?'ОШИБКА':cs.state===7?'ОТКЛОНЕНО':'ОЖИДАНИЕ';
get('googleResult').textContent='HTTP '+cs.http+' · удачных подтверждений '+cs.sent;
get('updateResult').textContent=s.ota.message+(s.ota.active?' '+s.ota.bytes+' байт':'');
get('ver').textContent=s.firmware;get('uptime').textContent=Math.floor(s.uptime_ms/1000)+' сек';get('pcf').textContent=s.modules.pcf8575?'Найден':'Не найден';get('pca').textContent=s.modules.pca9685_ack_only?'Найден':'Не найден';get('tft').textContent=s.modules.tft_attempted?'Инициализирован':'Не подтверждён';if(active===3)graphs();
}catch(e){if(connected)message('Потеряна связь с ESP32');connected=false}finally{pollBusy=false}}
get('googleSave').onclick=async()=>{
const url=get('googleUrl').value.trim(),token=get('googleToken').value.trim();
if(!url.startsWith('https://script.google.com/macros/s/')||!url.endsWith('/exec')||!/^[a-fA-F0-9]{64}$/.test(token)){
 get('googleResult').textContent='Требуется ссылка Apps Script /exec и секрет из 64 шестнадцатеричных символов';return;
}
if(!confirm('Включить отправку показаний в свой Google Диск? Только мониторинг.'))return;
let resp=await fetch('/api/google/config',{method:'POST',body:new URLSearchParams({url,token})});
get('googleResult').textContent=await resp.text();if(resp.ok)get('googleToken').value='';
};
get('googleStop').onclick=async()=>{
if(!confirm('Отключить отправку диагностических данных в Google?'))return;
const resp=await fetch('/api/google/off',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
get('googleResult').textContent=await resp.text();
};
get('manualUpdate').onclick=async function(){
  const file=get('firmwareFile').files[0];
  if(!file || !file.name.endsWith('.bin') || file.size<60000 || file.size>3340000){
    get('updateResult').textContent='Нужен APP BIN подходящего размера';return;
  }
  if(!confirm('Подтвердите ручное обновление диагностической прошивки. ESP32-S3 N8R2, внешние выходы отключены, выбран APP BIN?'))return;
  this.disabled=true;
  try{
    const begin=await fetch('/api/ota/prepare',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
    if(!begin.ok)throw Error(await begin.text());
    const grant=await begin.json(),body=new FormData();
    body.append('firmware',file,file.name);
    get('updateResult').textContent='Загрузка, не отключайте USB';
    const reply=await fetch('/api/ota/upload',{method:'POST',headers:{'X-Busa-Token':grant.token},body});
    if(!reply.ok)throw Error(await reply.text());
    get('updateResult').textContent='Успешно. БУСА перезапускается';
  }catch(e){get('updateResult').textContent='Ошибка: '+e.message}
  this.disabled=false;
};
get('net').onchange=()=>{if(get('net').value)get('response').textContent='Выбрана сеть: '+get('net').value+'. Введите пароль для подключения.'};
get('scan').onclick=async function(){this.disabled=true;get('response').textContent='Идёт поиск сетей...';
try{await fetch('/api/wifi/scan?start=1');let j=null;for(let i=0;i<12;i++){await new Promise(r=>setTimeout(r,1000));let x=await fetch('/api/wifi/scan',{cache:'no-store'});j=await x.json();if(!j.scanning)break}
let sel=get('net');sel.innerHTML='<option value="">Выберите сеть</option>';let seen=new Set();(j.networks||[]).forEach(w=>{if(!w.ssid||seen.has(w.ssid))return;seen.add(w.ssid);sel.add(new Option(w.ssid+' ('+w.rssi+' dBm)',w.ssid))});get('response').textContent='Найдено: '+(sel.options.length-1)}
catch(e){get('response').textContent='Ошибка поиска: '+e.message}this.disabled=false};
get('save').onclick=async function(){let ssid=get('net').value,pass=get('pass').value;if(!ssid||pass.length<8||pass.length>63){get('response').textContent='Выбери сеть из списка и введи пароль (8–63 символа)';return}this.disabled=true;get('response').textContent='Сохраняю Wi-Fi...';
try{let r=await fetch('/api/wifi',{method:'POST',body:new URLSearchParams({ssid,pass})});get('response').textContent=await r.text();if(r.ok){get('pass').value='';message('Настройки сети сохранены: '+ssid)}}catch(e){get('response').textContent='Ошибка: '+e.message}this.disabled=false};
message('Веб-приложение запущено');update();setInterval(update,1500);
</script></body></html>
)HTMLBUSA";

void handleCommand(String x){
  x.trim();x.toUpperCase();
  if(x=="STATUS") {Serial0.println(statusJSON());Serial.println(statusJSON());}
  else if(x.startsWith("PAGE ")){
    int n=x.substring(5).toInt();
    if(n>=1&&n<=6){currentPage=(uint8_t)(n-1);editingTemp=false;tftView=0;tftDirty=true;lastRender=0;Serial0.printf("PAGE=%d\n",n);}
  }
  else if(x=="TFT ST7789"||x=="TFT ILI9341"){
    tftProfile=(x=="TFT ILI9341")?2:1;
    prefs.putUChar("tft",tftProfile);
    Serial0.println("TFT profile saved. Reset board to apply.");
  }
  else Serial0.println("Commands: STATUS | PAGE 1..6 | TFT ST7789 | TFT ILI9341");
}
void readConsole(){
  // CH343 UART0 (COM6 on current board) is diagnostic console.
  while(Serial0.available()){
    char c=(char)Serial0.read();
    if(c=='\n'){handleCommand(consoleLine);consoleLine="";}
    else if(c!='\r'&&consoleLine.length()<80)consoleLine+=c;
  }
}
void setup(){
  Serial0.begin(115200);
  Serial0.println("\nBUSA 1M WiFi 4v LANDSCAPE DIAG BOOT; physical outputs LOCKED");
  Serial.begin(115200);
  delay(400);
  pinMode(FLOW_GPIO,INPUT); // requires an electrically verified 3.3 V logic signal
  attachInterrupt(digitalPinToInterrupt(FLOW_GPIO),flowISR,FALLING);
  Serial0.println("WATER GPIO15 interrupt attached (input only)");
  prefs.begin("busa-r04",false);
  tftProfile=prefs.getUChar("tft",1);
  startedAt=millis();
  if(tftProfile!=2)tftProfile=1;
  stationSsid=prefs.getString("ssid","");
  stationPass=prefs.getString("pass","");
  for(int i=0;i<4;i++){
    String k=String("rf")+String(i),j=String("nt")+String(i);
    refTemp[i]=prefs.getFloat(k.c_str(),NAN);
    noticeTemp[i]=prefs.getFloat(j.c_str(),NAN);
    if(!isfinite(refTemp[i])||refTemp[i]<0||refTemp[i]>125)refTemp[i]=NAN;
    if(!isfinite(noticeTemp[i])||noticeTemp[i]<0||noticeTemp[i]>125)noticeTemp[i]=NAN;
  }
  setupGoogleCloud();
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false); // Favor interactive local response over power saving
  // Private STA password must be provisioned into NVS after public GitHub build;
  // no STA password is embedded in this public source. Local AP fallback remains.
  apPassword="BUSA"+String((uint32_t)ESP.getEfuseMac(),HEX);
  wifiRunning=WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str());
  Serial0.printf("AP=%d IP=%s SSID=BUSA-1M-001-R04 PASS=%s\n",
                 (int)wifiRunning,WiFi.softAPIP().toString().c_str(),apPassword.c_str());
  if(stationSsid.length() && stationPass.length()>=8){
    WiFi.setAutoReconnect(true);
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
    Serial0.printf("STA connect requested: SSID=%s; password is hidden\n",stationSsid.c_str());
  }else Serial0.println("STA config missing: preprovision NVS before use; AP stays available");
  server.on("/",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",WEB_PAGE);});
  server.on("/api/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",statusJSON());
  });

  addGoogleRoutes();
  server.on("/api/ota/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",otaJSON());
  });
  server.on("/api/ota/prepare",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"||otaActive){
      server.send(403,"text/plain","Требуется локальная сеть БУСА и согласие");return;
    }
    char token[17];
    snprintf(token,sizeof(token),"%08lx%08lx",
      (unsigned long)esp_random(),(unsigned long)esp_random());
    otaToken=token;otaTokenIssued=millis();
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"token\":")+quoteJson(otaToken)+"}");
  });
  server.on("/api/ota/upload",HTTP_POST,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(otaSuccess?200:400,"application/json",otaJSON());
  },uploadOTA);
  server.on("/api/wifi/scan",HTTP_GET,[]{
    if(server.client().localIP()!=WiFi.softAPIP()){
      server.send(403,"application/json","{\"error\":\"Only protected local AP\"}");return;
    }
    if(server.hasArg("start") && server.arg("start")=="1"){
      WiFi.scanDelete();
      WiFi.scanNetworks(true,false);
    }
    int n=WiFi.scanComplete();
    if(n<0){server.send(200,"application/json","{\"scanning\":true,\"networks\":[]}");return;}
    nearbyCount=0;
    for(int i=0;i<n && nearbyCount<5;i++){
      if(WiFi.SSID(i).length()>0)nearbyNames[nearbyCount++]=WiFi.SSID(i);
    }
    String out="{\"scanning\":false,\"networks\":[";
    for(int i=0;i<n&&i<28;i++){
      if(i)out+=",";
      out+="{\"ssid\":"+quoteJson(WiFi.SSID(i));
      out+=",\"rssi\":"+String(WiFi.RSSI(i))+"}";
    }
    out+="]}";WiFi.scanDelete();
    server.send(200,"application/json; charset=utf-8",out);
  });
  server.on("/api/wifi",HTTP_POST,[]{
    // Credentials may only be provisioned while connected to the protected local AP.
    if(server.client().localIP()!=WiFi.softAPIP()){
      server.send(403,"text/plain; charset=utf-8","Настройка Wi-Fi доступна только через сеть БУСА.");return;
    }
    String ssid=server.arg("ssid"),pass=server.arg("pass");
    ssid.trim();
    if(ssid.isEmpty()||ssid.length()>32||pass.length()<8||pass.length()>63){
      server.send(400,"text/plain; charset=utf-8","Недопустимое имя сети или длина пароля (8–63).");return;
    }
    // NVS is encrypted only if board flash encryption is enabled; keep local AP secure.
    bool ok=(prefs.putString("ssid",ssid)>0 && prefs.putString("pass",pass)>0);
    if(!ok){server.send(500,"text/plain; charset=utf-8","Не удалось сохранить Wi-Fi настройки");return;}
    stationSsid=ssid;stationPass=pass;
    WiFi.disconnect();WiFi.begin(stationSsid.c_str(),stationPass.c_str());
    lastStationCheck=millis();
    server.send(200,"text/plain; charset=utf-8","Wi-Fi сохранён. Подключение к сети запущено.");
  });
  server.on("/api/page",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<1||n>6){server.send(400,"application/json","{\"error\":\"page 1..6\"}");return;}
    currentPage=(uint8_t)(n-1);editingTemp=false;tftView=0;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/display-refs",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain; charset=utf-8","Нужна локальная точка доступа БУСА");return;}
    float target[4],notice[4];
    for(int i=0;i<4;i++){
      String a=server.arg(String("ref")+i),b=server.arg(String("nt")+i);
      target[i]=a.length()?a.toFloat():NAN;
      notice[i]=b.length()?b.toFloat():NAN;
      if((isfinite(target[i])&&(target[i]<0||target[i]>125))||
         (isfinite(notice[i])&&(notice[i]<0||notice[i]>125))){
         server.send(400,"text/plain","Диапазон 0..125");return;
      }
    }
    for(int i=0;i<4;i++){
      String k=String("rf")+i,j=String("nt")+i;
      if(isfinite(target[i]))storeTempReference(i,0,target[i]);
      else{prefs.remove(k.c_str());refTemp[i]=NAN;}
      if(isfinite(notice[i]))storeTempReference(i,1,notice[i]);
      else{prefs.remove(j.c_str());noticeTemp[i]=NAN;}
    }
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8","Справочные значения сохранены. Реле не затронуты.");
  });
  server.on("/api/settings-tab",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<0||n>3){server.send(400,"text/plain","bad subtab");return;}
    settingsSubTab=(uint8_t)n;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/tft/scroll",HTTP_GET,[]{
    String dir=server.arg("dir");
    if(dir!="up"&&dir!="down"){server.send(400,"text/plain","bad dir");return;}
    tftView=(dir=="down")?1:0;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  const char *headers[]={"X-Busa-Token"};
  server.collectHeaders(headers,1);
  server.begin();
  Serial0.println("WEB started; temperature scan & I2C next");
  i2cOnline=Wire.begin(BUS_SDA,BUS_SCL,100000);
  if(i2cOnline){Wire.setTimeOut(25);probeI2c(millis());}
  Serial0.printf("I2C=%d PCF=%d PCA=%d\n",(int)i2cOnline,(int)pcfDetected,(int)pcaDetected);
  // Prioritize the AP and web UI at startup; start screen later.
  Serial0.println("TFT: delayed start scheduled (hardware SPI)");
  launchGoogleCloud();
  Serial0.println("SETUP DONE: relay GPIO4/5/6/7 NEVER configured/driven");
}
void loop(){
  uint32_t now=millis();
  server.handleClient();
  if(stationSsid.length() && stationPass.length()>=8 && WiFi.status()!=WL_CONNECTED && elapsed(now,lastStationCheck,20000)){
    lastStationCheck=now;
    WiFi.reconnect();
    Serial0.println("STA reconnect attempt; AP unaffected");
  }
  readConsole();
  if(otaRestart && elapsed(now,otaRestartAt,4000))ESP.restart();
  if(googleRestart && elapsed(now,googleRestartAt,3500))ESP.restart();
  if(!screenAttempted && elapsed(now,startedAt,8000)){
    screenAttempted=true;startScreen();
  }
  sampleTemperatures(now);
  sampleFlow(now);
  if(i2cOnline){
    if(elapsed(now,lastBusProbe,6000))probeI2c(now);
    pollKeys(now);
  }
  if(tftStarted && !otaActive && (tftDirty||lastRender==0 || elapsed(now,lastRender,2500))){
    lastRender=now;
    renderScreen(now);
  }
  if(elapsed(now,lastUart,5000)){
    lastUart=now;
    Serial0.printf("N8R2 HEARTBEAT ms=%lu wifi=%d i2c=%d tft=%d p=%lu flow=%.3f\n",
      (unsigned long)now,(int)wifiRunning,(int)i2cOnline,(int)tftStarted,
      (unsigned long)flowPulses(),(double)flowLpm);
  }
  delay(2);
}
