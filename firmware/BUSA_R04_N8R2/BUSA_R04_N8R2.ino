/*
  BUSA-1M-001 R04-N8R2.1
  ESP32-S3 N16R8 44-pin, TFT GMT028-05 V1.1 240x320 portrait.
  LOW-VOLTAGE MONITORING ONLY. DO NOT DRIVE RELAYS OR POWER OUTPUTS.
  Real 4 x DS18B20 + YF-S401 cooling-water pulse readings, no simulated values.
  Hardware compatibility of TFT controller, PCF8575 keypad and pulse output 
  must be verified on a powered-off low-voltage bench before connecting.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
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
const char VERSION[]="R04-N8R2.1";

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

Adafruit_ST7789 screenSt(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCK,TFT_RST);
Adafruit_ILI9341 screenIli(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCK,TFT_RST);
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
  Serial0.println("TFT: init start (model controller UNVERIFIED)");
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
    ruText("GPIO15: расход воды (вход)",11,91,GRAY);
    ruText("SPI: 10/11/12/13/14",11,119,GRAY);
    ruText("I2C: SDA8 SCL9",11,147,GRAY);
    ruText("K1-K4: только мониторинг",11,175,YELLOW);
    ruText("Реле и PCA: заблокированы",11,203,YELLOW);
    ruText("TFT: профиль не проверен",11,231,GRAY);
  }
  screen->drawFastHLine(0,303,240,BLUE);
  ruText(String(currentPage+1)+"/6  Wi-Fi: BUSA-1M-001",5,317,GRAY);
}

WebServer server(80);
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
  s+="}";
  return s;
}
const char WEB_PAGE[] PROGMEM=R"HTMLBUSA(
<!doctype html><html lang="ru"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>БУСА №001 — мониторинг</title>
<style>
body{margin:0;padding:12px;background:#061323;color:#b7e5f3;font:15px system-ui}
main{max-width:440px;margin:auto}.box{border:1px solid #1475ce;border-radius:12px;padding:13px;margin:10px 0;background:#09213a}
h1,h2{margin:5px 0 12px}strong{color:#65e899}.warn{color:#f7d766}
button{background:#124878;color:#fff;border:1px solid #2599e6;border-radius:7px;padding:11px;margin:3px}
</style><main><h1>БУСА-1М №001</h1>
<div class="box">Мониторинг низкого напряжения. <span class="warn">Реле заблокированы.</span></div>
<div class="box"><h2>Температуры</h2><div id="temps">Ожидание...</div></div>
<div class="box"><h2>Вода охлаждения</h2><div id="water">Ожидание...</div></div>
<div class="box"><h2>Отбор</h2>Датчик отсутствует. Значения не имитируются.</div>
<div class="box"><h2>Экран 240×320</h2>
<button onclick="page(1)">Главная</button><button onclick="page(2)">Температуры</button>
<button onclick="page(3)">Отбор</button><button onclick="page(4)">Графики</button>
<button onclick="page(5)">События</button><button onclick="page(6)">Настройки</button></div>
<div class="box"><h2>Модули</h2><div id="modules">Ожидание...</div></div>
<div class="box warn">Измерения воды требуют калибровки, отсутствие импульсов не подтверждает наличие охлаждения. Силовые цепи не подключать.</div>
<script>
async function page(n){try{await fetch('/api/page?n='+n)}catch(e){}}
async function read(){
 try{
 const s=await (await fetch('/api/status',{cache:'no-store'})).json();
 document.querySelector('#temps').innerHTML=s.temperatures.map(x=>x.name+': <strong>'+(x.c==null?'НЕТ ДАННЫХ':x.c+' °C')+'</strong>').join('<hr>');
 document.querySelector('#water').innerHTML='Расход: <strong>'+(s.water.lpm==null?'НЕТ ДАННЫХ':s.water.lpm+' л/мин')+'</strong><p>Импульсы: '+s.water.pulses+'; объём (оценка): '+s.water.total_l_estimate+' л</p><small class="warn">Коэффициент не откалиброван</small>';
 document.querySelector('#modules').textContent='PCF8575: '+s.modules.pcf8575+', PCA9685: '+s.modules.pca9685_ack_only+', TFT инициализирован: '+s.modules.tft_attempted+', кнопка: '+s.modules.last_key;
 }catch(e){document.querySelector('#modules').textContent='Нет связи: '+e}
}
setInterval(read,2000);read();
</script></main></html>
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
  // COM4 CH343 UART0 is the primary diagnostic console.
  while(Serial0.available()){
    char c=(char)Serial0.read();
    if(c=='\n'){handleCommand(consoleLine);consoleLine="";}
    else if(c!='\r'&&consoleLine.length()<80)consoleLine+=c;
  }
}
void setup(){
  Serial0.begin(115200);
  Serial0.println("\nBUSA R04-N8R2.1 BOOT; physical outputs LOCKED");
  Serial.begin(115200);
  delay(400);
  pinMode(FLOW_GPIO,INPUT); // requires an electrically verified 3.3 V logic signal
  attachInterrupt(digitalPinToInterrupt(FLOW_GPIO),flowISR,FALLING);
  Serial0.println("WATER GPIO15 interrupt attached (input only)");
  prefs.begin("busa-r04",false);
  tftProfile=prefs.getUChar("tft",1);
  if(tftProfile!=2)tftProfile=1;
  stationSsid=prefs.getString("ssid","");
  stationPass=prefs.getString("pass","");
  WiFi.mode(WIFI_AP_STA);
  // Lab-only unique AP password; no credentials stored in public repository.
  apPassword="BUSA"+String((uint32_t)ESP.getEfuseMac(),HEX);
  wifiRunning=WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str());
  Serial0.printf("AP=%d IP=%s SSID=BUSA-1M-001-R04 PASS=%s\n",
                 (int)wifiRunning,WiFi.softAPIP().toString().c_str(),apPassword.c_str());
  if(stationSsid.length())WiFi.begin(stationSsid.c_str(),stationPass.c_str());
  server.on("/",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",WEB_PAGE);});
  server.on("/api/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",statusJSON());
  });
  server.on("/api/page",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<1||n>6){server.send(400,"application/json","{\"error\":\"page 1..6\"}");return;}
    currentPage=(uint8_t)(n-1);lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.begin();
  Serial0.println("WEB started; temperature scan & I2C next");
  i2cOnline=Wire.begin(BUS_SDA,BUS_SCL,100000);
  if(i2cOnline){Wire.setTimeOut(25);probeI2c(millis());}
  Serial0.printf("I2C=%d PCF=%d PCA=%d\n",(int)i2cOnline,(int)pcfDetected,(int)pcaDetected);
  // Display starts after AP and web to improve crash diagnostics.
  startScreen();
  Serial0.println("SETUP DONE: relay GPIO4/5/6/7 NEVER configured/driven");
}
void loop(){
  uint32_t now=millis();
  server.handleClient();
  readConsole();
  sampleTemperatures(now);
  sampleFlow(now);
  if(i2cOnline){
    if(elapsed(now,lastBusProbe,6000))probeI2c(now);
    pollKeys(now);
  }
  if(lastRender==0 || elapsed(now,lastRender,1900)){
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
