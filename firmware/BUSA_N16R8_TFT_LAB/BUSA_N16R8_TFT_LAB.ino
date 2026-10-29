// BUSA N16R8 read-only LAB test (NOT full release): TFT, four DS18B20 and PSRAM
// TFT 7-pin: CS=10, DC=11, RST=12, SDA/MOSI=13, SCK=14, 3V3, GND.
// T1..T4 DATA: 16/17/18/21. No relay output pins are configured.
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>
constexpr uint8_t TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14;
constexpr uint8_t TPINS[4]={16,17,18,21};
static_assert(TFT_CS==10&&TFT_DC==11&&TFT_RST==12&&TFT_MOSI==13&&TFT_SCK==14,"TFT pin mismatch");
Adafruit_ST7789 tft(TFT_CS,TFT_DC,TFT_RST);
OneWire o1(16),o2(17),o3(18),o4(21);
DallasTemperature d1(&o1),d2(&o2),d3(&o3),d4(&o4);
DallasTemperature* sensors[4]={&d1,&d2,&d3,&d4};
WebServer web(80);
float tempC[4]={NAN,NAN,NAN,NAN};
uint32_t lastReadMs=0;
const char* VERSION="BUSA-N16R8-TFT-LAB-20261029";
String statusJson(){
 String s="{\"firmware\":\""+String(VERSION)+"\"";
 s+=",\"flash_bytes\":"+String(ESP.getFlashChipSize());
 s+=",\"psram_bytes\":"+String(ESP.getPsramSize());
 s+=",\"free_psram\":"+String(ESP.getFreePsram());
 s+=",\"free_heap\":"+String(ESP.getFreeHeap());
 s+=",\"temperatures_c\":[";
 for(int i=0;i<4;i++){if(i)s+=",";s+=isfinite(tempC[i])?String(tempC[i],2):String("null");}
 return s+"]}";
}
void draw(){
 tft.fillScreen(ST77XX_BLACK);
 tft.setTextColor(ST77XX_CYAN);tft.setTextSize(2);tft.setCursor(5,8);tft.print("BUSA N16R8 LAB");
 tft.setTextColor(ST77XX_WHITE);
 for(int i=0;i<4;i++){
  tft.setCursor(8,45+i*31);tft.print("T");tft.print(i+1);tft.print(" ");
  if(isfinite(tempC[i])){tft.print(tempC[i],1);tft.print(" C");}
  else tft.print("NO DATA");
 }
 tft.setTextSize(1);tft.setCursor(8,177);
 tft.print("Flash: ");tft.print(ESP.getFlashChipSize()/1048576UL);tft.print("MB");
 tft.setCursor(8,194);tft.print("PSRAM: ");
 tft.print(ESP.getPsramSize()/1048576UL);tft.print("MB");
 tft.setCursor(8,214);tft.print("192.168.4.1/api/status");
}
void sample(){
 for(int i=0;i<4;i++){
  sensors[i]->requestTemperatures();
  float t=sensors[i]->getTempCByIndex(0);
  tempC[i]=(isfinite(t)&&t>=-55&&t<=125&&t!=85)?t:NAN;
 }
}
void setup(){
 Serial.begin(115200);delay(250);
 SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS);
 tft.init(240,320);tft.setRotation(1);
 for(auto* sensor:sensors){sensor->begin();sensor->setResolution(10);}
 WiFi.mode(WIFI_AP);
 String ssid="BUSA-LAB-"+String((uint32_t)(ESP.getEfuseMac()&0xFFFFFF),HEX);
 WiFi.softAP(ssid.c_str(),"BusaLab2026");
 web.on("/",HTTP_GET,[]{web.send(200,"text/plain","BUSA LAB /api/status");});
 web.on("/api/status",HTTP_GET,[]{web.sendHeader("Cache-Control","no-store");web.send(200,"application/json",statusJson());});
 web.begin();Serial.println("LAB SSID: "+ssid);
 Serial.printf("FLASH=%u PSRAM=%u\n",(unsigned)ESP.getFlashChipSize(),(unsigned)ESP.getPsramSize());
 draw();
}
void loop(){
 web.handleClient();
 if(millis()-lastReadMs>=3000UL){lastReadMs=millis();sample();draw();}
 delay(1);
}
