#pragma once
// BUSA-1M-001, SPI TFT 240x320 diagnostic status (ST7789/ILI9341 profile).
// SPI panel is write-only: this cannot identify a TFT controller automatically.
#include <Arduino.h>
#include <SPI.h>
#ifndef BUSA_TFT_DRIVER
#define BUSA_TFT_DRIVER 1 // 1=ST7789, 2=ILI9341. Must match real GMT028-05 panel.
#endif
struct TftMonitor {
 static constexpr uint8_t PIN_CS=10,PIN_MOSI=11,PIN_SCK=12,PIN_RST=15,PIN_DC=16;
 bool enabled=false;
 uint32_t lastDraw=0;
 void command(uint8_t c){digitalWrite(PIN_DC,LOW);digitalWrite(PIN_CS,LOW);SPI.transfer(c);digitalWrite(PIN_CS,HIGH);}
 void data(uint8_t d){digitalWrite(PIN_DC,HIGH);digitalWrite(PIN_CS,LOW);SPI.transfer(d);digitalWrite(PIN_CS,HIGH);}
 void reset(){digitalWrite(PIN_RST,LOW);delay(25);digitalWrite(PIN_RST,HIGH);delay(130);}
 void init(){
   pinMode(PIN_CS,OUTPUT);pinMode(PIN_DC,OUTPUT);pinMode(PIN_RST,OUTPUT);
   digitalWrite(PIN_CS,HIGH);digitalWrite(PIN_DC,HIGH);
   SPI.begin(PIN_SCK,-1,PIN_MOSI,PIN_CS);SPI.beginTransaction(SPISettings(8000000,MSBFIRST,SPI_MODE0));
   reset();command(0x01);delay(130);command(0x11);delay(130);
   command(0x3A);data(0x55);command(0x36);data(0x00);
#if BUSA_TFT_DRIVER==1
   command(0x21); // ST7789 inverted-color mode
#endif
   command(0x29);delay(50);enabled=true;fill(0,0,240,320,0x0000);
 }
 void region(int x,int y,int w,int h){
  uint16_t x1=x+w-1,y1=y+h-1;
  command(0x2A);data(x>>8);data(x);data(x1>>8);data(x1);
  command(0x2B);data(y>>8);data(y);data(y1>>8);data(y1);
  command(0x2C);
 }
 void fill(int x,int y,int w,int h,uint16_t color){
  if(!enabled||x<0||y<0||x+w>240||y+h>320||w<1||h<1)return;
  region(x,y,w,h);uint8_t buf[64];
  for(int i=0;i<64;i+=2){buf[i]=color>>8;buf[i+1]=color&255;}
  digitalWrite(PIN_DC,HIGH);digitalWrite(PIN_CS,LOW);
  size_t n=(size_t)w*h*2;
  while(n){size_t k=n<sizeof(buf)?n:sizeof(buf);SPI.writeBytes(buf,k);n-=k;}
  digitalWrite(PIN_CS,HIGH);
 }
 static const uint8_t* glyph(char c){
  // Five-pixel columns in seven raster rows, for status and numeric telemetry.
  static const uint8_t blank[7]={0,0,0,0,0,0,0};
  static const uint8_t digits[10][7]={
   {14,17,19,21,25,17,14},{4,12,4,4,4,4,14},{14,17,1,2,4,8,31},
   {30,1,1,14,1,1,30},{2,6,10,18,31,2,2},{31,16,30,1,1,17,14},
   {6,8,16,30,17,17,14},{31,1,2,4,8,8,8},{14,17,17,14,17,17,14},
   {14,17,17,15,1,2,12}};
  if(c>='0'&&c<='9')return digits[c-'0'];
  static const uint8_t letters[26][7]={
   {14,17,17,31,17,17,17},{30,17,17,30,17,17,30},
   {15,16,16,16,16,16,15},{30,17,17,17,17,17,30},
   {31,16,16,30,16,16,31},{31,16,16,30,16,16,16},
   {15,16,16,23,17,17,15},{17,17,17,31,17,17,17},
   {14,4,4,4,4,4,14},{7,2,2,2,18,18,12},
   {17,18,20,24,20,18,17},{16,16,16,16,16,16,31},
   {17,27,21,21,17,17,17},{17,25,21,19,17,17,17},
   {14,17,17,17,17,17,14},{30,17,17,30,16,16,16},
   {14,17,17,17,21,18,13},{30,17,17,30,20,18,17},
   {15,16,16,14,1,1,30},{31,4,4,4,4,4,4},
   {17,17,17,17,17,17,14},{17,17,17,17,17,10,4},
   {17,17,17,21,21,21,10},{17,17,10,4,10,17,17},
   {17,17,10,4,4,4,4},{31,1,2,4,8,16,31}};
  if(c>='A'&&c<='Z')return letters[c-'A'];
  static const uint8_t dot[7]={0,0,0,0,0,6,6};
  static const uint8_t minus[7]={0,0,0,31,0,0,0};
  static const uint8_t colon[7]={0,4,4,0,4,4,0};
  static const uint8_t slash[7]={1,2,2,4,8,8,16};
  if(c=='.')return dot;if(c=='-')return minus;if(c==':')return colon;if(c=='/')return slash;
  return blank;
 }
 void printAt(int x,int y,const char *s,uint16_t color=0xffff,uint8_t scale=2){
  if(!enabled)return;
  for(;*s;s++,x+=6*scale){
    if(x+5*scale>240)break;
    auto *g=glyph(*s);
    for(int r=0;r<7;r++)for(int col=0;col<5;col++)if(g[r]&(1<<(4-col)))
      fill(x+col*scale,y+r*scale,scale,scale,color);
  }
 }
 void draw(const float values[4],const bool valid[4],bool bridge,bool keyboard,bool wifi){
  if(!enabled||uint32_t(millis()-lastDraw)<1200)return;
  lastDraw=millis();fill(0,0,240,320,0x0000);
  fill(0,0,240,38,0x0210);printAt(10,10,"BUSA 001",0xffff,2);
  printAt(8,49,"MONITOR",0x07ff,2);
  for(int i=0;i<4;i++){
   char b[24];if(valid[i])snprintf(b,sizeof(b),"T%d: %.1f C",i+1,values[i]);
   else snprintf(b,sizeof(b),"T%d: NO DATA",i+1);
   printAt(8,82+i*36,b,valid[i]?0x07e0:0xf800,2);
  }
  printAt(8,242,bridge?"DS2482 OK":"DS2482 NO",bridge?0x07e0:0xf800,2);
  printAt(8,265,keyboard?"KEYPAD OK":"KEYPAD NO",keyboard?0x07e0:0xf800,2);
  printAt(8,288,wifi?"WIFI OK":"WIFI NO",wifi?0x07e0:0xf800,2);
 }
};
