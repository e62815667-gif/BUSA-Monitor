/*
  BUSA-1M-001 БУСА 1М WiFi 2v
  ESP32-S3 N8R2 44-pin, TFT GMT028-05 V1.1 240x320 portrait.
  LOW-VOLTAGE MONITORING ONLY. DO NOT DRIVE RELAYS OR POWER OUTPUTS.
  Real 4 x DS18B20 + YF-S401 cooling-water pulse readings, no simulated values.
  Hardware compatibility of TFT controller, PCF8575 keypad and pulse output 
  must be verified on a powered-off low-voltage bench before connecting.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <WebServer.h>
#include <Update.h>
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
const char VERSION[]="БУСА 1М WiFi 2v";

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
void pollKeys(uint32_t now){
  if(!pcfDetected||!elapsed(now,lastKeyScan,70))return;
  lastKeyScan=now;
  // PROVISIONAL mapping 5 rows -> P0..P4, 4 cols -> P5..P8;
  // confirm physical matrix with keypad diagnostics, do not claim mapping verified.
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
    keyMessage="R"+String(pressed/4+1)+" C"+String(pressed%4+1);
    ++keyCount;keyHeld=true;
    // Do not guess which keycap is UP, DOWN, MENU, etc.
    // Page can be selected via USB command / local diagnostic web UI.
  }
}

Adafruit_ST7789 screenSt(TFT_CS,TFT_DC,TFT_RST);
Adafruit_ILI9341 screenIli(TFT_CS,TFT_DC,TFT_RST);
Adafruit_GFX* screen=nullptr;
U8G2_FOR_ADAFRUIT_GFX ru;
Preferences prefs;
uint8_t tftProfile=1;
bool tftStarted=false;
const uint16_t BG=0x080f,PANEL=0x10a3,BLUE=0x1c9f,GREEN=0x07e0,WHITE=0xffff,
               YELLOW=0xffe0,GRAY=0xbdf7;
const char* pages[6]={"Главная","Температуры","Отбор","Графики","События","Настройки"};
void ruText(const String &v,int x,int y,uint16_t color=WHITE){
  ru.setForegroundColor(color);
  ru.setCursor(x,y);
  ru.print(v);
}
void card(int y,int height){
  screen->fillRoundRect(5,y,230,height,5,PANEL);
  screen->drawRoundRect(5,y,230,height,5,BLUE);
}
void startScreen(){
  Serial0.println("TFT: HW SPI begin (model controller UNVERIFIED)");
  SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS); // Approved GPIO12 SCK, GPIO11 MOSI, GPIO10 CS
  if(tftProfile==2){
    screenIli.begin();screen=&screenIli;
  }else{
    screenSt.init(240,320);screen=&screenSt;
  }
  screen->setRotation(0); // approved PORTRAIT 240x320
  screen->fillScreen(BG);
  ru.begin(*screen);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
  ru.setFontMode(1);
  tftStarted=true;
  Serial0.println("TFT: init done");
}
void renderScreen(uint32_t now){
  if(!tftStarted||!screen)return;
  screen->fillScreen(BG);
  screen->fillRoundRect(3,2,234,29,5,PANEL);
  ruText(String("БУСА №001 | ")+pages[currentPage],7,21,BLUE);
  if(currentPage<=1){
    const char* names[]={"Вода","Колонна","Куб","Верх колонны"};
    for(int i=0;i<4;i++){
      int y=39+i*42;card(y,38);
      ruText(names[i],11,y+14,GRAY);
      ruText(hasTemp(i,now)?String(temperature[i],1)+" °C":"НЕТ ДАННЫХ",
             115,y+27,hasTemp(i,now)?GREEN:YELLOW);
    }
    if(currentPage==0){
      card(212,42);
      ruText("Вода охлаждения",11,228,GRAY);
      ruText(flowReady?String(flowLpm,2)+" л/мин":"НЕТ ДАННЫХ",105,246,flowReady?GREEN:YELLOW);
      card(259,42);
      ruText("Отбор: нет датчика",11,274,YELLOW);
      ruText("Реле K1-K4: ВЫКЛ",11,292,YELLOW);
    }else{
      card(212,84);
      ruText("DS18B20: реальные данные",11,234,GRAY);
      ruText("Нет датчика -> нет показаний",11,254,YELLOW);
      ruText("Защита: выходы отключены",11,278,GRAY);
    }
  }else if(currentPage==2){
    card(40,119);
    ruText("Отбор (другая линия)",11,61,BLUE);
    ruText("Нет утверждённого датчика",11,85,YELLOW);
    ruText("Расход: -- мл/мин",11,111,GRAY);
    ruText("Собрано: -- мл",11,137,GRAY);
    card(164,132);
    ruText("Охлаждающая вода YF-S401",11,186,BLUE);
    ruText(flowReady?String(flowLpm,2)+" л/мин":"ОЖИДАНИЕ",11,213,flowReady?GREEN:YELLOW);
    ruText(String("Импульсы: ")+flowPulses(),11,235,GRAY);
    ruText("Калибровка не выполнена",11,259,YELLOW);
    ruText("Не использовать как защиту",11,282,YELLOW);
  }else if(currentPage==3){
    card(40,256);
    ruText("График воды (80 сек.)",11,61,BLUE);
    screen->drawFastHLine(16,243,205,GRAY);
    for(uint8_t i=1;i<40;i++){
      uint8_t prev=(flowTraceHead+i-1)%40,next=(flowTraceHead+i)%40;
      int x1=18+(i-1)*5,x2=18+i*5;
      int y1=243-(int)fminf(145.0f,flowTrace[prev]*30.0f);
      int y2=243-(int)fminf(145.0f,flowTrace[next]*30.0f);
      screen->drawLine(x1,y1,x2,y2,GREEN);
    }
    ruText("Только измеренные импульсы",11,266,GRAY);
    ruText("Коэфф. требует проверки",11,285,YELLOW);
  }else if(currentPage==4){
    card(40,256);
    ruText("Диагностические события",11,62,BLUE);
    ruText(String("Время: ")+(now/1000)+" сек",11,88,GRAY);
    ruText(String("PCF8575: ")+(pcfDetected?"OK":"не найден"),11,114,GRAY);
    ruText(String("PCA9685: ")+(pcaDetected?"OK":"не найден"),11,140,GRAY);
    ruText(String("Нажатий: ")+keyCount,11,166,GREEN);
    ruText(String("Импульсы воды: ")+flowPulses(),11,192,GRAY);
    ruText("Аппаратная проверка нужна",11,246,YELLOW);
  }else{
    card(40,256);
    ruText(VERSION,11,62,BLUE);
    ruText(String("Wi-Fi: ")+(WiFi.status()==WL_CONNECTED?"ПОДКЛ.":"ЛОКАЛЬНО"),11,87,GREEN);
    ruText(String("SSID: ")+stationSsid.substring(0,19),11,111,GRAY);
    ruText(String("IP: ")+(WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():"—"),11,135,GRAY);
    ruText("Выбор сети: веб-меню",11,163,BLUE);
    if(nearbyCount)ruText(nearbyNames[0].substring(0,23),11,185,GRAY);
    ruText("Обновление: через веб",11,213,BLUE);
    ruText(otaMessage.substring(0,23),11,235,GRAY);
    ruText("Реле: ЗАБЛОКИРОВАНЫ",11,270,YELLOW);
  }
  screen->drawFastHLine(0,303,240,BLUE);
  ruText(String(currentPage+1)+"/6  Wi-Fi: BUSA-1M-001",5,317,GRAY);
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
String otaJSON(); // forward declaration, implemented after status serializer
String statusJSON(){
  uint32_t now=millis();
  String s="{\"device\":\"BUSA-1M-001\",\"firmware\":\""+String(VERSION)+"\"";
  s+=",\"mode\":\"LOW_VOLTAGE_MONITOR_ONLY\",\"relays_enabled\":false,\"pwm_enabled\":false";
  s+=",\"ui_page\":"+String(currentPage+1)+",\"uptime_ms\":"+String(now);
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
const char WEB_PAGE[] PROGMEM=R"HTMLBUSA(
<!doctype html><html lang="ru"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>БУСА-1М №001</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{font:15px system-ui;margin:0;padding:0 12px 20px;background:#061323;color:#d0efff}header{position:sticky;top:0;background:#091c31;padding:14px 0 9px;border-bottom:1px solid #237fc5}h1{font-size:20px;color:#64c6ff;margin:0 0 10px}h2{font-size:16px;color:#81caff;margin:2px 0 12px}nav{display:flex;overflow-x:auto;gap:6px;padding-bottom:4px}button{font:inherit;cursor:pointer;border:1px solid #258bcc;background:#123d61;color:#fff;border-radius:8px;padding:10px}nav button{font-size:12px;white-space:nowrap}nav .on{background:#2379b1;border-color:#7dc9ff}main{max-width:680px;margin:auto}.card{background:#09233c;border:1px solid #2383c7;border-radius:12px;padding:13px;margin:11px 0}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.tile{border:1px solid #265a80;border-radius:9px;padding:9px}.tile small{display:block;color:#9fbfd0}.tile strong{display:block;color:#64f2a2;margin:5px 0;font-size:20px}.row{display:flex;justify-content:space-between;border-bottom:1px solid #204664;padding:9px 0;gap:10px}.row:last-child{border:0}.green{color:#70e99b}.warn{color:#f4cc74}.hint{color:#97b4c9;font-size:12px}.page[hidden]{display:none}input,select{width:100%;background:#102f4b;border:1px solid #2d84bd;color:#fff;font:inherit;padding:12px;border-radius:8px}label{display:block;margin:12px 0 5px}.full{width:100%;margin:12px 0}#response{color:#f3d783;white-space:pre-wrap}canvas{width:100%;height:170px;border:1px solid #28618f;border-radius:8px}
</style></head><body>
<header><h1>БУСА-1М с ИИ · №001</h1><nav>
<button data-page="0" class="on">Главная</button><button data-page="1">Температуры</button>
<button data-page="2">Отбор</button><button data-page="3">Графики</button><button data-page="4">События</button><button data-page="5">Настройки</button>
</nav></header><main>
<section class="page" data-section="0"><div class="card"><h2>Температуры</h2><div class="grid">
<div class="tile"><small>Вода</small><strong id="t0">—</strong></div><div class="tile"><small>Колонна</small><strong id="t1">—</strong></div>
<div class="tile"><small>Куб</small><strong id="t2">—</strong></div><div class="tile"><small>Верх колонны</small><strong id="t3">—</strong></div></div></div>
<div class="card"><h2>Схема установки</h2><div class="grid"><div class="tile">Куб → Колонна</div><div class="tile">→ Верх колонны</div></div><p class="hint">Схематичное отображение без управления исполнительными механизмами</p></div>
<div class="card"><h2>Отбор</h2><div class="grid"><div class="tile"><small>Расход, мл/мин</small><b class="warn">НЕТ ДАТЧИКА</b></div><div class="tile"><small>Собрано, мл</small><b class="warn">НЕТ ДАННЫХ</b></div></div></div>
<div class="card"><h2>Система</h2><div class="row">Состояние <b class="green">Диагностика</b></div><div class="row">Реле <b class="warn">ЗАБЛОКИРОВАНЫ</b></div><div class="row">Wi-Fi <b id="mainwifi">—</b></div><div class="row">Напряжение <b class="warn">НЕТ ДАТЧИКА</b></div></div>
<div class="card"><h2>Вода охлаждения · YF-S401</h2><div class="row">Расход, л/мин <b id="flow">—</b></div><div class="row">Импульсы <b id="pulses">—</b></div><p class="warn">Не откалибровано. 0 импульсов не подтверждает охлаждение.</p></div></section>
<section class="page" data-section="1" hidden><div class="card"><h2>Температуры · DS18B20</h2><div id="temperatures">Ожидание...</div><p class="warn">Уставки защитной автоматики недоступны в диагностической версии.</p></div></section>
<section class="page" data-section="2" hidden><div class="card"><h2>Отбор</h2><div class="row">Расход, мл/мин <b class="warn">НЕТ ДАТЧИКА</b></div><div class="row">Собрано, мл <b class="warn">НЕТ ДАННЫХ</b></div><p class="hint">Датчик воды охлаждения не измеряет поток отбора.</p></div><div class="card"><h2>Вода охлаждения</h2><div class="row">Расход, л/мин <b id="flow2">—</b></div><div class="row">Объём, л (оценка) <b id="total">—</b></div></div></section>
<section class="page" data-section="3" hidden><div class="card"><h2>Графики текущей сессии</h2><p>Вода, °C</p><canvas id="graphT" width="600" height="200"></canvas><p>Вода охлаждения, л/мин</p><canvas id="graphF" width="600" height="200"></canvas><p class="hint">Только реальные данные с момента открытия приложения.</p></div></section>
<section class="page" data-section="4" hidden><div class="card"><h2>События</h2><div id="events">Ожидание...</div><p class="hint">Журнал текущей сессии приложения</p></div></section>
<section class="page" data-section="5" hidden><div class="card"><h2>Wi-Fi</h2>
<div id="wstatus">Проверка подключения...</div><button type="button" class="full" id="scan">Найти сети Wi-Fi 2,4 ГГц</button>
<label for="net">Доступные сети Wi-Fi</label><select id="net" required><option value="">Выберите сеть из списка</option></select><p class="hint">Если нужной сети нет, включите на телефоне раздачу интернета и повторите поиск.</p>
<label for="pass">Пароль Wi-Fi</label><input id="pass" type="password" minlength="8" maxlength="63" placeholder="Введите пароль самостоятельно">
<button type="button" class="full" id="save">Сохранить и подключить</button>
<p id="response" role="status"></p><p class="hint">Настройка доступна при подключении к защищённой сети БУСА. Пароль остаётся на контроллере.</p></div>
<div class="card"><h2>Обновление ПО · вручную</h2>
<p class="hint">Только через локальную сеть БУСА. Выберите APP BIN для ESP32-S3 N8R2,
НЕ объединённый merged BIN. Только проверенные диагностические сборки.
Перед обновлением отключите внешние исполнительные цепи.</p>
<input type="file" accept=".bin" id="firmwareFile">
<button type="button" class="full" id="manualUpdate">Выбрать и обновить</button>
<p id="updateResult" role="status">Ожидание подтверждения</p>
</div><div class="card"><h2>Диагностика</h2><div class="row">Прошивка <b id="ver">—</b></div><div class="row">Время работы <b id="uptime">—</b></div><div class="row">PCF8575 <b id="pcf">—</b></div><div class="row">PCA9685 <b id="pca">—</b></div><div class="row">TFT <b id="tft">—</b></div></div></section>
<footer class="hint">БУСА-1М №001 · диагностический режим · реле заблокированы</footer></main>
<script>
const get=id=>document.getElementById(id),logs=[],tlog=[],flog=[];let active=0,connected=false,pollBusy=false;
function message(s){logs.unshift(new Date().toLocaleTimeString('ru-RU')+' · '+s);logs.splice(12);get('events').textContent=logs.join('\n')}
function page(n){active=n;document.querySelectorAll('[data-section]').forEach(e=>e.hidden=Number(e.dataset.section)!==n);document.querySelectorAll('nav button').forEach(e=>e.classList.toggle('on',Number(e.dataset.page)===n));fetch('/api/page?n='+(n+1)).catch(()=>{});if(n===3)graphs()}
document.querySelectorAll('nav button').forEach(e=>e.onclick=()=>page(Number(e.dataset.page)));
function plot(id,values){const c=get(id),x=c.getContext('2d');x.fillStyle='#061b2c';x.fillRect(0,0,600,200);const a=values.filter(v=>v!==null);if(!a.length){x.fillStyle='#d9bd75';x.fillText('НЕТ ДАННЫХ',24,60);return}let lo=Math.min(...a),hi=Math.max(...a);if(hi-lo<.1)hi=lo+.1;x.strokeStyle='#70e99b';x.beginPath();values.forEach((v,i)=>{if(v===null)return;let xx=10+i*580/80,yy=185-(v-lo)/(hi-lo)*165;if(i===0||values[i-1]===null)x.moveTo(xx,yy);else x.lineTo(xx,yy)});x.stroke()}
function graphs(){plot('graphT',tlog);plot('graphF',flog)}
async function update(){if(pollBusy)return;pollBusy=true;try{let r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error(r.status);let s=await r.json();if(!connected)message('Связь с ESP32 восстановлена');connected=true;let names=['Вода','Колонна','Куб','Верх колонны'];
s.temperatures.forEach((t,i)=>{get('t'+i).textContent=t.c===null?'НЕТ ДАННЫХ':Number(t.c).toFixed(1)+' °C';if(i===0){tlog.push(t.c===null?null:Number(t.c));if(tlog.length>80)tlog.shift()}});
get('temperatures').textContent=names.map((v,i)=>v+': '+get('t'+i).textContent).join(' | ');
let f=s.water.lpm===null?null:Number(s.water.lpm);get('flow').textContent=f===null?'НЕТ ДАННЫХ':f.toFixed(2);get('flow2').textContent=get('flow').textContent;get('pulses').textContent=s.water.pulses;get('total').textContent=Number(s.water.total_l_estimate).toFixed(3);
flog.push(f);if(flog.length>80)flog.shift();let ss=s.station.connected?'ПОДКЛЮЧЕНО: '+s.station.ssid+' · '+s.station.ip:(s.station.ssid?'Ожидание подключения: '+s.station.ssid:'Wi-Fi не настроен. Выберите сеть ниже.');get('wstatus').textContent=ss;get('mainwifi').textContent=s.station.connected?'Подключено':'Локальная сеть';
if(s.station.connected&&s.station.ssid){
 let select=get('net'),found=Array.from(select.options).some(o=>o.value===s.station.ssid);
 if(!found)select.add(new Option(s.station.ssid+' (подключена)',s.station.ssid));
 if(!select.value)select.value=s.station.ssid;
}
get('updateResult').textContent=s.ota.message+(s.ota.active?' '+s.ota.bytes+' байт':'');
get('ver').textContent=s.firmware;get('uptime').textContent=Math.floor(s.uptime_ms/1000)+' сек';get('pcf').textContent=s.modules.pcf8575?'Найден':'Не найден';get('pca').textContent=s.modules.pca9685_ack_only?'Найден':'Не найден';get('tft').textContent=s.modules.tft_attempted?'Инициализирован':'Не подтверждён';if(active===3)graphs();
}catch(e){if(connected)message('Потеряна связь с ESP32');connected=false}finally{pollBusy=false}}
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
    if(n>=1&&n<=6){currentPage=(uint8_t)(n-1);lastRender=0;Serial0.printf("PAGE=%d\n",n);}
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
  Serial0.println("\nBUSA 1M WiFi 2v BOOT; physical outputs LOCKED");
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
    currentPage=(uint8_t)(n-1);lastRender=0;
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
  if(!screenAttempted && elapsed(now,startedAt,8000)){
    screenAttempted=true;startScreen();
  }
  sampleTemperatures(now);
  sampleFlow(now);
  if(i2cOnline){
    if(elapsed(now,lastBusProbe,6000))probeI2c(now);
    pollKeys(now);
  }
  if(tftStarted && !otaActive && (lastRender==0 || elapsed(now,lastRender,5000))){
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
