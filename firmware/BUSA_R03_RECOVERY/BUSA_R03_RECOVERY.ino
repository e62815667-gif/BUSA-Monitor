/*
 * BUSA-1M-001 R03-RECOVERY — low-voltage diagnostics only.
 * Based on approved schemes 1–7 and actual ESP32-S3 N16R8 44-pin table.
 * Never drives relay K1..K4, never changes PCA9685 outputs.
 * TFT is optional ST7789 profile, unverified for actual GMT028-05 V1.1.
 * Peripheral tests default OFF, require electrical checks before arming.
 */
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_ILI9341.h>
#include <math.h>
constexpr uint8_t TPIN[4]={38,39,40,41};
constexpr uint8_t KPIN[4]={7,6,5,4}; // reference ONLY; never pinMode or digitalWrite
constexpr uint8_t BUS_SDA=8,BUS_SCL=9;
constexpr uint8_t TFT_CS=10,TFT_MOSI=11,TFT_SCK=12,TFT_DC=13,TFT_RST=14;
static_assert(TPIN[0]==38 && TPIN[3]==41 && BUS_SDA==8 && BUS_SCL==9,"Pin map changed");
constexpr bool RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false;
static_assert(!RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,"Outputs must be disabled");
static const char* VERSION="R03-RECOVERY.5";
OneWire ow1(38),ow2(39),ow3(40),ow4(41);
DallasTemperature ds1(&ow1),ds2(&ow2),ds3(&ow3),ds4(&ow4);
DallasTemperature* ds[4]={&ds1,&ds2,&ds3,&ds4};
// Software SPI guarantees approved pin selection even with board-specific defaults.
Adafruit_ST7789 tftSt(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCK,TFT_RST);
Adafruit_ILI9341 tftIli(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCK,TFT_RST);
Adafruit_GFX* display=nullptr;
String displayProfile="DISARMED";
uint8_t displayMode=1; // default ST7789; 2=ILI9341

WebServer web(80);
Preferences wifiPrefs;
String stationSsid="iPhone 12",stationPassword="123456789";
uint32_t lastStationRetry=0;
float t[4]={NAN,NAN,NAN,NAN};
bool sensorFound[4]={false,false,false,false};
uint32_t updated[4]={0,0,0,0};
bool busArmed=false,temperatureArmed=false,keyboardArmed=false,tftArmed=false;
bool pcfFound=false,pcaFound=false; uint8_t pcfAddress=0;
uint32_t lastProbe=0,lastConv=0,lastTempStart=0,lastKeyScan=0,lastTft=0;
bool converting=false;
String lastKey="NO DATA",serialLine,apPass; uint32_t keyEvents=0;
int pendingKey=-1;uint32_t pendingSince=0;bool held=false;
bool due(uint32_t now,uint32_t then,uint32_t interval){return (uint32_t)(now-then)>=interval;}
bool acknowledge(uint8_t addr){Wire.beginTransmission(addr);return Wire.endTransmission()==0;}
void probe(){
  if(!busArmed)return;
  pcfFound=false;pcfAddress=0;
  for(uint8_t addr=0x20;addr<=0x27;addr++){
    if(acknowledge(addr)){pcfFound=true;pcfAddress=addr;break;}
  }
  pcaFound=acknowledge(0x40);
  lastProbe=millis();
}
bool keypadWrite(uint16_t bits){
  if(!pcfFound)return false;
  Wire.beginTransmission(pcfAddress);
  Wire.write((uint8_t)bits);Wire.write((uint8_t)(bits>>8));
  return Wire.endTransmission()==0;
}
bool keypadRead(uint16_t& bits){
  if(!pcfFound||Wire.requestFrom(pcfAddress,(uint8_t)2)!=(uint8_t)2)return false;
  bits=(uint8_t)Wire.read();bits|=(uint16_t)((uint8_t)Wire.read())<<8;return true;
}
void pollKeyboard(uint32_t now){
  if(!keyboardArmed||!pcfFound||!due(now,lastKeyScan,30))return;
  lastKeyScan=now;
  constexpr uint8_t rows[5]={0,1,2,3,4};
  constexpr uint8_t cols[4]={5,6,7,8}; // P10 = bit 8
  int pressed=-1;bool ok=true;
  for(uint8_t r=0;r<5;r++){
    uint16_t values=0;
    if(!keypadWrite((uint16_t)(0xffffu & ~(1u<<rows[r])))){ok=false;break;}
    delayMicroseconds(180);
    if(!keypadRead(values)){ok=false;break;}
    for(uint8_t c=0;c<4;c++)if(!(values&(1u<<cols[c]))){
      pressed=pressed==-1 ? (r*4+c) : -2;
    }
  }
  keypadWrite(0xffff);
  if(!ok){keyboardArmed=false;lastKey="I2C ERROR";return;}
  if(pressed!=pendingKey){pendingKey=pressed;pendingSince=now;held=false;}
  if(pressed>=0&&!held&&due(now,pendingSince,50)){
    lastKey="ROW "+String(pressed/4+1)+" COL "+String(pressed%4+1);
    ++keyEvents;held=true;
  }
}
void pollTemperature(uint32_t now){
  if(!temperatureArmed)return;
  if(converting&&due(now,lastConv,850)){
    converting=false;
    for(int i=0;i<4;i++){
      float v=ds[i]->getTempCByIndex(0);
      if(sensorFound[i]&&isfinite(v)&&v>=-55&&v<=125&&v!=DEVICE_DISCONNECTED_C){
        t[i]=v;updated[i]=now;
      }else{t[i]=NAN;updated[i]=0;}
    }
  }
  if(!converting&&(lastTempStart==0||due(now,lastTempStart,2300))){
    lastTempStart=now;
    for(int i=0;i<4;i++){
      ds[i]->begin();sensorFound[i]=(ds[i]->getDeviceCount()>0);
      if(sensorFound[i]){
        ds[i]->setResolution(12);
        ds[i]->setWaitForConversion(false);
        ds[i]->requestTemperatures();
      }else{t[i]=NAN;updated[i]=0;}
    }
    converting=true;lastConv=now;
  }
  for(int i=0;i<4;i++)if(updated[i]&&due(now,updated[i],6500)){
    updated[i]=0;t[i]=NAN;
  }
}
String yes(bool v){return v?"true":"false";}
String num(float v){return isfinite(v)?String(v,2):"null";}
// Indication channels CH0..CH10 are VIRTUAL ONLY; GPIO/PCA9685 outputs never switched.
// Former valve indicators CH6/CH7 are reserved and always off.
String virtualLeds(){
  uint32_t now=millis();bool valid[4];
  for(int i=0;i<4;i++)valid[i]=temperatureArmed&&updated[i]&&!due(now,updated[i],6500)&&isfinite(t[i]);
  bool missing=!(valid[0]&&valid[1]&&valid[2]&&valid[3]);
  bool wifi=WiFi.softAPgetStationNum()>0;
  bool channels[11]={valid[0],valid[1],valid[2],valid[3],
    false,true,false,false,wifi,!wifi,missing};
  String j="[";
  for(int i=0;i<11;i++){if(i)j+=",";j+=yes(channels[i]);}
  return j+"]";
}
String jsonQuoted(const String& value){
  String o="\"";
  for(size_t i=0;i<value.length();i++){
    char c=value[i];
    if(c=='"'||c=='\\'){o+='\\';o+=c;}
    else if(c=='\n')o+="\\n";
    else if(c=='\r')o+="\\r";
    else if((unsigned char)c<32)o+=' ';
    else o+=c;
  }
  return o+"\"";
}
String statusJson(){
  String s="{\"device\":\"BUSA-1M-001\",\"firmware\":\""+String(VERSION)+"\"";
  s+=",\"mode\":\"LOW_VOLTAGE_DIAGNOSTIC\",\"uptime_ms\":"+String(millis());
  s+=",\"pins\":{\"T\":[38,39,40,41],\"K\":[7,6,5,4],\"i2c\":[8,9],\"tft\":[10,11,12,13,14]}";
  s+=",\"modules\":{\"i2c_armed\":"+yes(busArmed);
  s+=",\"pcf8575_detected\":"+yes(pcfFound);
  s+=",\"pcf8575_addr\":"+String(pcfFound?pcfAddress:0);
  s+=",\"pca9685_detected_read_only\":"+yes(pcaFound);
  s+=",\"keyboard_armed\":"+yes(keyboardArmed);
  s+=",\"temperatures_armed\":"+yes(temperatureArmed);
  s+=",\"tft_armed\":"+yes(tftArmed)+"},\"temperatures\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"id\":"+String(i+1)+",\"present\":"+yes(sensorFound[i]);
    s+=",\"value_c\":"+num(t[i])+",\"valid\":"+yes(updated[i]&&isfinite(t[i]))+"}";
  }
  s+="],\"keypad\":{\"last\":\""+lastKey+"\",\"events\":"+String(keyEvents)+"}";
  s+=",\"display_profile\":\""+displayProfile+"\",\"led_channels_virtual\":"+virtualLeds();
  s+=",\"relays_physical_enabled\":false,\"leds_physical_enabled\":false,\"relay_contacts\":null";
  s+=",\"ssid\":\"BUSA-1M-001-R03\",\"ip\":\""+WiFi.softAPIP().toString()+"\"";
  s+=",\"station\":{\"ssid\":"+jsonQuoted(stationSsid);
  s+=",\"connected\":"+yes(WiFi.status()==WL_CONNECTED);
  s+=",\"ip\":"+jsonQuoted(WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():String(""));
  s+=",\"rssi\":"+String(WiFi.status()==WL_CONNECTED?WiFi.RSSI():0);
  s+="}}";
  return s;
}
const char PAGE[] PROGMEM=R"BUSAHTML(<!doctype html><html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>БУСА-1М №001 — R03</title><style>
body{font:16px system-ui;background:#101b28;color:#eef5fa;margin:0;padding:16px}
main{max-width:640px;margin:auto}section{background:#243746;padding:14px;border-radius:12px;margin:12px 0}
h1{font-size:22px}h2{font-size:18px}small{color:#bbd1e1}pre{white-space:pre-wrap;overflow-wrap:anywhere}button,input,select{font:inherit;max-width:100%;padding:7px;margin:5px 0}input{box-sizing:border-box}button{cursor:pointer}
</style></head><body><main><h1>БУСА-1М №001</h1>
<small>R03 RECOVERY • Мониторинг 3,3 В • Физические выходы K1–K4 и LED отключены</small>
<section id="temp">T1–T4: ожидание</section><section id="mods">Модули: ожидание</section>
<section id="keys">Клавиатура: ожидание</section>
<section><h2>Дисплей GMT028-05 V1.1</h2>
<p>Экран автоматически показывает T1–T4, Wi-Fi и состояние модулей.</p>
<label>Драйвер:
<select id="tftDriver"><option value="1">ST7789 (основной)</option><option value="2">ILI9341 (альтернативный)</option></select></label>
<button type="button" onclick="selectScreen()">Применить и перезапустить экран</button>
<p id="tftMessage">Если экран белый, выберите другой драйвер.</p>
<small>Изменение драйвера перезапустит ESP32, но не включит реле.</small></section>
<section><h2>Wi-Fi — интернет через iPhone 12</h2>
<p id="station">Проверка подключения...</p>
<p>Резервная сеть ESP32: BUSA-1M-001-R03</p>
<button type="button" onclick="scanWifi()">Найти сети рядом</button>
<select id="networks" onchange="document.getElementById('wifiSsid').value=this.value"><option value="">Выбрать найденную сеть</option></select>
<div><label>Имя сети: <input id="wifiSsid" value="iPhone 12" autocomplete="off"></label></div>
<div><label>Пароль: <input id="wifiPassword" type="password" value="123456789" autocomplete="off"></label></div>
<button type="button" onclick="connectWifi()">Подключить</button><p id="wifiMessage"></p>
<small>Для интернета включите точку доступа на iPhone 12. Сеть самой БУСА остаётся доступна.</small></section>
<section><small>Проверка модулей разрешается только после сверки электропитания и проводки.
Команды через USB: BUS ON, TEMP ON, KEY ON, TFT ST7789 / TFT ILI9341.
Контроллер GMT028-05 не установлен: выбираем профиль после сверки маркировки.
Реле и светодиоды физически не включаются.</small></section>
<section><pre id="raw"></pre></section>
<script>
async function read(){try{
let s=await (await fetch('/api/status',{cache:'no-store'})).json();
document.getElementById('temp').textContent=s.temperatures.map(x=>'T'+x.id+': '+(x.valid?x.value_c+' °C':'Нет данных')).join(' | ');
document.getElementById('mods').textContent='PCF8575: '+s.modules.pcf8575_detected+'; PCA9685: '+s.modules.pca9685_detected_read_only+'; TFT: '+s.modules.tft_armed;
document.getElementById('keys').textContent='Клавиатура: '+s.keypad.last+' ('+s.keypad.events+' нажатий)';
document.getElementById('tftDriver').value=s.display_profile==='ILI9341'?'2':'1';
document.getElementById('station').textContent=s.station.connected?
  ('Подключено: '+s.station.ssid+' • IP '+s.station.ip):
  ('Не подключено: '+s.station.ssid+' (проверьте имя сети iPhone)');
document.getElementById('raw').textContent=JSON.stringify(s,null,2);
}catch(e){document.getElementById('raw').textContent='Нет связи: '+e;}}
async function selectScreen(){
  let m=document.getElementById('tftMessage');
  const profile=document.getElementById('tftDriver').value;
  m.textContent='Сохраняю драйвер экрана...';
  try{
    const r=await fetch('/display/select',{method:'POST',body:new URLSearchParams({profile})});
    m.textContent=(await r.json()).message||'Проверьте экран после перезапуска';
  }catch(e){m.textContent='Переподключитесь к сети БУСА через 15 секунд';}
}
async function scanWifi(){let m=document.getElementById('wifiMessage');
m.textContent='Поиск сетей...';
try{let r=await (await fetch('/wifi/scan')).json();let el=document.getElementById('networks');
el.textContent='';let p=document.createElement('option');p.value='';p.textContent='Выбрать сеть';el.appendChild(p);
for(let x of r.networks){let o=document.createElement('option');o.value=x.ssid;o.textContent=x.ssid+' ('+x.rssi+' dBm)';el.appendChild(o)}
m.textContent='Найдено сетей: '+r.networks.length;}catch(e){m.textContent='Ошибка поиска: '+e}}
async function connectWifi(){let m=document.getElementById('wifiMessage');
let ssid=document.getElementById('wifiSsid').value,pass=document.getElementById('wifiPassword').value;
if(!ssid||pass.length<8){m.textContent='Введите имя сети и пароль (от 8 символов)';return}
m.textContent='Подключение...';
try{let form=new URLSearchParams({ssid:ssid,pass:pass});
let r=await fetch('/wifi/connect',{method:'POST',body:form});
m.textContent=(await r.json()).message||'Проверьте статус сети через 15 секунд';
}catch(e){m.textContent='Ошибка подключения: '+e}}
setInterval(read,1800);read();</script></main></body></html>)BUSAHTML";
// The SPI display does not provide a verified readable controller ID.
// Select a driver explicitly and persist the preference between reboots.
void tftStart(uint8_t which){
  displayMode=(which==2)?2:1;
  if(displayMode==1){
    tftSt.init(240,320);display=&tftSt;displayProfile="ST7789";
  }else{
    tftIli.begin();display=&tftIli;displayProfile="ILI9341";
  }
  display->setRotation(1); // 320x240 landscape, matching actual screen orientation
  display->fillScreen(0x0841);
  tftArmed=true;lastTft=0;
}
void tftDraw(uint32_t now){
  if(!tftArmed||!display||(lastTft&&!due(now,lastTft,1500)))return;
  lastTft=now;
  const uint16_t BG=0x0841,PANEL=0x1082,WHITE=0xffff,GOOD=0x07e0,
                 BAD=0xf800,CYAN=0x07ff,WARN=0xffe0,GRAY=0xbdf7;
  display->fillScreen(BG);
  display->setTextWrap(false);
  display->setTextColor(WHITE);
  display->setTextSize(2);
  display->setCursor(5,5);display->print("BUSA-1M #001");
  display->setTextSize(1);display->setTextColor(CYAN);
  display->setCursor(5,25);display->print("R03.5 LIVE DIAGNOSTICS");
  // Built-in Adafruit GFX font is ASCII: English labels, Russian names in web UI.
  const char* labels[4]={"WATER","COLUMN","TANK","TOP"};
  for(int i=0;i<4;i++){
    const int x=4+(i%2)*159, y=38+(i/2)*68;
    display->fillRoundRect(x,y,152,64,6,PANEL);
    display->drawRoundRect(x,y,152,64,6,GRAY);
    display->setTextSize(1);
    display->setTextColor(CYAN);
    display->setCursor(x+6,y+7);display->print(labels[i]);
    const bool valid=temperatureArmed&&updated[i]&&isfinite(t[i])&&!due(now,updated[i],6500);
    display->setTextSize(2);
    display->setTextColor(valid?GOOD:WARN);
    display->setCursor(x+7,y+30);
    if(valid)display->printf("%.1f C",t[i]);
    else display->print("NO DATA");
  }
  display->setTextSize(1);
  display->setCursor(5,179);display->setTextColor(WHITE);
  display->printf("PCF8575:%s  PCA9685:%s  KEYS:%s",
    pcfFound?"OK":"--",pcaFound?"OK":"--",keyboardArmed?"ON":"OFF");
  display->setCursor(5,193);display->setTextColor(CYAN);
  if(WiFi.status()==WL_CONNECTED)
    display->printf("iPhone WiFi OK  IP:%s",WiFi.localIP().toString().c_str());
  else display->print("iPhone WiFi: NO LINK / AP ACTIVE");
  display->setCursor(5,207);display->setTextColor(GRAY);
  display->printf("AP: %s",WiFi.softAPIP().toString().c_str());
  display->setCursor(5,223);display->setTextColor(BAD);
  display->print("K1-K4: OFF  OUTPUTS LOCKED");
}
void help(){
  Serial.println("Commands: STATUS | BUS ON/OFF | SCAN | TEMP ON/OFF | KEY ON/OFF | TFT ST7789 / TFT ILI9341");
  Serial.println("Only after module voltage and wiring checked; K1-K4 + PCA outputs disabled.");
}
void cmd(String s){
  s.trim();s.toUpperCase();
  if(s=="STATUS")Serial.println(statusJson());
  else if(s=="BUS ON"){
    busArmed=Wire.begin(BUS_SDA,BUS_SCL,100000);
    if(busArmed){Wire.setTimeOut(25);probe();}
    Serial.println(busArmed?"I2C armed":"I2C error");
  }else if(s=="BUS OFF"){
    if(keyboardArmed)keypadWrite(0xffff);
    keyboardArmed=false;busArmed=false;pcfFound=false;pcaFound=false;
    Wire.end();Serial.println("I2C off");
  }else if(s=="SCAN"){
    if(busArmed){probe();Serial.printf("PCF=%d addr=0x%02X PCA=%d\n",pcfFound,pcfAddress,pcaFound);}
    else Serial.println("I2C disarmed: BUS ON only after wiring verification");
  }else if(s=="TEMP ON"){
    temperatureArmed=true;lastTempStart=0;Serial.println("Temperature monitor armed");
  }else if(s=="TEMP OFF"){
    temperatureArmed=false;converting=false;
    for(int i=0;i<4;i++){t[i]=NAN;updated[i]=0;sensorFound[i]=false;}
    Serial.println("Temperatures off");
  }else if(s=="KEY ON"){
    if(!busArmed||!pcfFound)Serial.println("Keypad unavailable: first BUS ON");
    else if(!keypadWrite(0xffff))Serial.println("Keypad I2C failure");
    else{keyboardArmed=true;lastKey="NO KEY YET";Serial.println("Keypad armed (row/col ONLY)");}
  }else if(s=="KEY OFF"){
    keyboardArmed=false;if(busArmed&&pcfFound)keypadWrite(0xffff);
    Serial.println("Keypad off");
  }else if(s=="TFT ST7789"){
    wifiPrefs.putUChar("tft",1);
    Serial.println("ST7789 saved. Reboot to select driver.");
  }else if(s=="TFT ILI9341"){
    wifiPrefs.putUChar("tft",2);
    Serial.println("ILI9341 saved. Reboot to select driver.");
  }else help();
}
void readUsb(){
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(ch=='\n'){cmd(serialLine);serialLine="";}
    else if(ch!='\r'&&serialLine.length()<64)serialLine+=ch;
  }
}
void setup(){
  Serial.begin(115200);delay(1600);
  // Automatic low-voltage display and read-only temperature monitoring.
  displayMode=1;
  temperatureArmed=true;
  WiFi.mode(WIFI_AP_STA);
  wifiPrefs.begin("busa-wifi",false);
  stationSsid=wifiPrefs.getString("ssid","iPhone 12");
  stationPassword=wifiPrefs.getString("pass","123456789");
  displayMode=wifiPrefs.getUChar("tft",1);
  if(displayMode!=2)displayMode=1;
  // Fixed password requested for isolated low-voltage lab verification.
  // This password is NOT suitable for a public or permanent deployment.
  apPass="123456789";
  bool wifi=WiFi.softAP("BUSA-1M-001-R03",apPass.c_str());
  WiFi.setAutoReconnect(true);
  WiFi.begin(stationSsid.c_str(),stationPassword.c_str());
  lastStationRetry=millis();
  web.on("/",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",PAGE);});
  web.on("/api/status",HTTP_GET,[]{web.sendHeader("Cache-Control","no-store");web.send(200,"application/json",statusJson());});
  web.on("/display/select",HTTP_POST,[]{
    uint8_t newDriver=(uint8_t)web.arg("profile").toInt();
    if(newDriver!=1&&newDriver!=2){
      web.send(400,"application/json","{\"message\":\"Неизвестный драйвер\"}");
      return;
    }
    wifiPrefs.putUChar("tft",newDriver);
    web.send(200,"application/json","{\"message\":\"Драйвер сохранён. Перезапуск через 1 секунду.\"}");
    delay(1000);
    ESP.restart();
  });
  web.on("/wifi/scan",HTTP_GET,[]{
    // Wi-Fi scan may briefly interrupt AP traffic. Physical outputs untouched.
    int found=WiFi.scanNetworks(false,true);
    String output="{\"networks\":[";
    for(int i=0;i<found;i++){
      if(i)output+=",";
      output+="{\"ssid\":"+jsonQuoted(WiFi.SSID(i))+",\"rssi\":"+String(WiFi.RSSI(i))+"}";
    }
    output+="]}";
    WiFi.scanDelete();
    web.send(200,"application/json; charset=utf-8",output);
  });
  web.on("/wifi/connect",HTTP_POST,[]{
    String requested=web.arg("ssid"),password=web.arg("pass");
    if(requested.length()==0||requested.length()>32||password.length()<8||password.length()>63){
      web.send(400,"application/json","{\"message\":\"Неверные имя сети или пароль\"}");
      return;
    }
    stationSsid=requested;stationPassword=password;
    wifiPrefs.putString("ssid",stationSsid);
    wifiPrefs.putString("pass",stationPassword);
    web.send(200,"application/json","{\"message\":\"Настройки сохранены. Проверьте статус через 15 секунд.\"}");
    WiFi.disconnect();
    WiFi.begin(stationSsid.c_str(),stationPassword.c_str());
    lastStationRetry=millis();
  });
  web.begin();
  // TFT init uses approved GPIO10-14; relay GPIO7/6/5/4 are never configured.
  tftStart(displayMode);
  // PCF8575 and PCA9685 probing: ACK-only, no output/register writes.
  busArmed=Wire.begin(BUS_SDA,BUS_SCL,100000);
  if(busArmed){Wire.setTimeOut(25);probe();}
  Serial.println("\nBUSA-1M-001 R03-RECOVERY.5 — outputs ALWAYS OFF");
  Serial.printf("WiFi AP %s. SSID BUSA-1M-001-R03, password: %s, http://%s/\n",
    wifi?"started":"failed",apPass.c_str(),WiFi.softAPIP().toString().c_str());
  Serial.println("TFT and TEMP automatic. I2C probes are read-only. KEY/RELAY/PWM outputs OFF.");
  help();
}
void loop(){
  uint32_t now=millis();readUsb();web.handleClient();
  if(busArmed&&due(now,lastProbe,5000))probe();
  pollTemperature(now);pollKeyboard(now);tftDraw(now);
  if(WiFi.status()!=WL_CONNECTED&&due(now,lastStationRetry,30000)){
    lastStationRetry=now;
    WiFi.disconnect();
    WiFi.begin(stationSsid.c_str(),stationPassword.c_str());
  }
  delay(2);
}
