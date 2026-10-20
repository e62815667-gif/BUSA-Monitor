// ESP32-S3 N8R2 isolated USB-bench display check. No actuators, no sensing of process.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
constexpr uint8_t LCD_CS=10,LCD_MOSI=11,LCD_SCK=12,LCD_DC=13,LCD_RST=14,BUS_SDA=8,BUS_SCL=9;
Arduino_DataBus* bus=new Arduino_ESP32SPI(LCD_DC,LCD_CS,LCD_SCK,LCD_MOSI,GFX_NOT_DEFINED);
Arduino_GFX* lcd=new Arduino_ST7796(bus,LCD_RST,1,true);
WebServer web(80);
bool lcdOk=false,touchOk=false,down=false;
uint16_t tx=0,ty=0; uint32_t hits=0,lastDraw=0;
String pass;
const char page[] PROGMEM=R"web(
<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>БУСА: проверка TFT</title><style>body{font:18px system-ui;background:#102031;color:white;max-width:600px;margin:auto;padding:20px}b{color:#70ebaa}</style>
<h2>Диагностика экрана и сенсора</h2><p>Изолированный стенд ESP32-S3, никаких исполнительных цепей.</p>
<div>ST7796U SPI: <b id="lcd">—</b><p>FT6336U I²C: <b id="touch">—</b></p>
<p>Касания: <b id="hits">—</b></p><p>X/Y: <b id="xy">—</b></p><p>Работает: <b id="up">—</b> сек.</p></div>
<script>async function run(){try{let x=await(await fetch('/api/status',{cache:'no-store'})).json();for(let k of ['lcd','touch','hits','xy','up'])document.getElementById(k).textContent=x[k]}catch(e){}}run();setInterval(run,1000);</script>
)web";
void draw(){
 if(!lcdOk)return;
 lcd->fillScreen(0x0841);lcd->setTextSize(3);lcd->setTextColor(0xffff);
 lcd->setCursor(12,14);lcd->print("BUSA: TFT TOUCH BENCH");
 lcd->setTextSize(2);lcd->setTextColor(0x07e0);
 lcd->setCursor(12,75);lcd->print("ST7796U 480x320 SPI");
 lcd->setCursor(12,112);lcd->print("FT6336U: ");lcd->print(touchOk?"FOUND":"NOT FOUND");
 lcd->setTextColor(0xffe0);lcd->setCursor(12,170);lcd->print("Touch X/Y: ");lcd->print(tx);lcd->print(" / ");lcd->print(ty);
 lcd->setCursor(12,210);lcd->print("Touch events: ");lcd->print(hits);
 lcd->setTextColor(0xffff);lcd->setCursor(12,285);lcd->print("AP: BUSA-TFT-TEST");
}
void scanTouch(){
 Wire.beginTransmission(0x38);Wire.write(0x02);
 if(Wire.endTransmission(false)!=0)return;
 if(Wire.requestFrom((uint8_t)0x38,(uint8_t)5)!=5)return;
 const uint8_t n=Wire.read(),xh=Wire.read(),xl=Wire.read(),yh=Wire.read(),yl=Wire.read();
 const bool pressed=(n&0x0f)>0&&(n&0x0f)<=2;
 if(pressed){tx=((xh&15)<<8)|xl;ty=((yh&15)<<8)|yl;if(!down)hits++;}
 down=pressed;
}
void setup(){
 Serial0.begin(115200);Serial.begin(115200);
 WiFi.mode(WIFI_AP);
 pass="BUSA"+String((uint32_t)ESP.getEfuseMac(),HEX);
 bool wifi=WiFi.softAP("BUSA-TFT-TEST",pass.c_str());
 Serial0.printf("BUSA bench display test. AP=%d PASS=%s IP=%s\n",wifi,pass.c_str(),WiFi.softAPIP().toString().c_str());
 web.on("/",[]{web.send_P(200,"text/html; charset=utf-8",page);});
 web.on("/api/status",[]{
  String a=String("{\"lcd\":\"")+(lcdOk?"ready":"not ready")+"\",\"touch\":\""+(touchOk?"found":"not found")
   +"\",\"hits\":"+String(hits)+",\"xy\":\""+String(tx)+" / "+String(ty)+"\",\"up\":"+String(millis()/1000)+"}";
  web.sendHeader("Cache-Control","no-store");web.send(200,"application/json",a);
 });
 web.begin();
 Wire.begin(BUS_SDA,BUS_SCL,100000);Wire.setTimeOut(25);
 Wire.beginTransmission(0x38);touchOk=Wire.endTransmission()==0;
 lcdOk=lcd->begin(27000000);if(lcdOk){lcd->setRotation(1);draw();}
 Serial0.printf("ST7796U=%d FT6336U=%d; NO OUTPUTS ENABLED\n",lcdOk,touchOk);
}
void loop(){
 web.handleClient();
 static uint32_t lastTouch=0;
 if(touchOk&&millis()-lastTouch>=70){lastTouch=millis();scanTouch();}
 if(lcdOk&&millis()-lastDraw>=1500){lastDraw=millis();draw();}
 delay(2);
}
