#pragma once
// BUSA-1M-001 V14: detailed RGB565 screen artwork translated from six user-approved PNG designs.
// Works on 320x240 ST7789/ILI9341. Passive drawing only, NEVER controls any GPIO.
// Screen pixel density is physical 320x240; shadows and layered shapes add visual detail.
// RGB565 high-contrast light palette, based on the six approved mockups.
static const uint16_t HQ_NAVY=0xffdf,HQ_DEEP=0xffff,HQ_GLOW=0xfca0,
 HQ_AZURE=0xd4a5,HQ_SKY=0x234c,HQ_CYAN=0x0519,HQ_LIGHT=0x2104,
 HQ_LIME=0x2664,HQ_AQUA=0x767c,HQ_GLASS=0xf638,HQ_STEEL=0xad55,
 HQ_SILVER=0xe73c,HQ_METAL=0xb5b6,HQ_DARKMETAL=0x6b4d;
static uint16_t hqMix(uint16_t a,uint16_t b,uint8_t mix){
 uint16_t ar=(a>>11)&31,ag=(a>>5)&63,ab=a&31,br=(b>>11)&31,bg=(b>>5)&63,bb=b&31;
 return (((ar*(255-mix)+br*mix)/255)<<11)|(((ag*(255-mix)+bg*mix)/255)<<5)|((ab*(255-mix)+bb*mix)/255);
}
static void hqBackground(){
  // Solid light cream, not an expensive striped gradient.
  screen->fillScreen(HQ_NAVY);
  screen->drawFastHLine(0,49,320,HQ_STEEL);
  screen->drawFastHLine(0,210,320,HQ_STEEL);
}
static void hqPanel(int x,int y,int w,int h){
  if(w<7 || h<7)return;
  screen->fillRoundRect(x,y,w,h,5,HQ_DEEP);
  screen->drawRoundRect(x,y,w,h,5,HQ_STEEL);
  screen->drawRoundRect(x+1,y+1,w-2,h-2,4,HQ_GLASS);
  screen->drawFastHLine(x+6,y+1,w-12,HQ_GLOW);
}
static void hqBulb(int x,int y,int r,uint16_t c){
 screen->fillCircle(x,y,r+3,0x008e);
 screen->fillCircle(x,y,r+1,HQ_AZURE);
 screen->fillCircle(x,y,r,c);
 screen->fillCircle(x-1,y-1,1,HQ_LIGHT);
}
static void hqSymbol(uint8_t p,int cx,int cy,int k,uint16_t c=HQ_CYAN){
 switch(p){
 case 0:
  screen->fillTriangle(cx-k,cy,cx,cy-k,cx+k,cy,c);
  screen->fillRect(cx-k+2,cy,k*2-4,k-1,c);
  screen->fillRect(cx-2,cy+2,4,k-3,HQ_NAVY);break;
 case 1:
  screen->drawRoundRect(cx-3,cy-k,6,k*2-3,3,HQ_SKY);
  screen->drawFastVLine(cx,cy-k+4,k*2-8,c);
  hqBulb(cx,cy+k-2,3,c);break;
 case 2:
  screen->fillTriangle(cx,cy-k,cx-k+2,cy+3,cx+k-2,cy+3,c);
  screen->fillCircle(cx,cy+2,k-3,c);
  screen->drawLine(cx-2,cy-1,cx-3,cy+2,HQ_LIGHT);break;
 case 3:
  screen->drawFastVLine(cx-k,cy-k,k*2,c);
  screen->drawFastHLine(cx-k,cy+k,k*2,c);
  screen->drawLine(cx-k+2,cy+2,cx-2,cy-4,HQ_SKY);
  screen->drawLine(cx-2,cy-4,cx+3,cy,HQ_SKY);
  screen->drawLine(cx+3,cy,cx+k-1,cy-k+1,HQ_LIME);break;
 case 4:
  screen->drawRoundRect(cx-k+2,cy-k,k*2-4,k*2,2,c);
  for(int z=0;z<3;z++)screen->drawFastHLine(cx-k+5,cy-k+4+z*5,k*2-11,HQ_SKY);break;
 default:
  screen->drawCircle(cx,cy,k-2,c);
  screen->drawCircle(cx,cy,k-3,c);
  screen->fillCircle(cx,cy,3,HQ_LIGHT);
  screen->drawFastHLine(cx-k,cy,4,c);screen->drawFastHLine(cx+k-4,cy,4,c);
  screen->drawFastVLine(cx,cy-k,4,c);screen->drawFastVLine(cx,cy+k-4,4,c);
 }
}
static void hqRig(int x,int y,int w,int h){
 // Layered illustration inspired by approved MAIN reference art.
 // Decorative representation only; no schematic/electrical/process instructions.
 hqPanel(x,y,w,h);
 auto X=[&](int n){return x+3+n*(w-6)/100;};
 auto Y=[&](int n){return y+3+n*(h-6)/100;};
 // Equipment shadows and nickel-steel surfaces
 int vl=X(8),vr=X(53),top=Y(64),bottom=Y(88);
 screen->fillRoundRect(vl+3,top+3,vr-vl,bottom-top+6,5,0x31a6);
 screen->fillRoundRect(vl,top,vr-vl,bottom-top+5,5,HQ_METAL);
 for(int k=0;k<6;k++){
  int xx=vl+2+k*(vr-vl-4)/5;
  screen->drawFastVLine(xx,top+4,bottom-top-1,hqMix(HQ_DARKMETAL,HQ_SILVER,(uint8_t)(k*44)));
 }
 screen->fillRoundRect(vl,top,vr-vl,6,2,HQ_SILVER);
 screen->drawFastHLine(vl+4,bottom+3,vr-vl-8,HQ_STEEL);
 screen->drawRoundRect(vl+2,top+7,vr-vl-4,bottom-top-10,4,HQ_DARKMETAL);
 screen->fillRect(X(29),Y(76),4,Y(85)-Y(76),HQ_GLOW);
 // Rising column with clamps, fittings, blue cooling highlights
 int cl=X(34),cr=X(49),ct=Y(10),cb=Y(64);
 screen->fillRoundRect(cl,ct,cr-cl,cb-ct,3,HQ_DARKMETAL);
 screen->fillRoundRect(cl+2,ct+2,cr-cl-4,cb-ct-4,2,HQ_METAL);
 screen->drawFastVLine(cl+4,ct+5,cb-ct-10,HQ_LIGHT);
 screen->drawFastVLine(cr-4,ct+5,cb-ct-10,HQ_GLASS);
 for(int k=0;k<6;k++){
  int yy=ct+5+k*(cb-ct-10)/6;
  screen->drawFastHLine(cl-3,yy,cr-cl+6,HQ_SILVER);
  screen->drawFastHLine(cl-3,yy+2,cr-cl+6,HQ_DARKMETAL);
 }
 hqBulb(cr-5,Y(25),2,HQ_CYAN);
 hqBulb(cr-5,Y(54),2,HQ_CYAN);
 // polished right-hand condenser with highlighted coil
 int rx=X(62),ry=Y(18),rw=X(27)-X(0),rh=Y(48)-Y(0);
 screen->fillRoundRect(rx,ry,rw,rh,4,HQ_SILVER);
 screen->fillRoundRect(rx+2,ry+2,rw-4,rh-4,3,HQ_DEEP);
 for(int j=0;j<6;j++){
  int sy=ry+5+j*(rh-10)/6;
  screen->drawLine(rx+5,sy,rx+rw-6,sy+4,HQ_GLOW);
  screen->drawLine(rx+rw-6,sy+4,rx+5,sy+7,HQ_SKY);
 }
 screen->drawLine(X(42),Y(12),X(42),Y(7),HQ_SILVER);
 screen->drawLine(X(42),Y(7),X(75),Y(7),HQ_SILVER);
 screen->drawLine(X(75),Y(7),X(75),Y(18),HQ_SILVER);
 screen->drawLine(X(49),Y(59),X(70),Y(59),HQ_GLOW);

 // Cooling water only: a distinct YF-S401 flow sensor on the BLUE cooling line.
 // This is a labelled illustrative symbol, not a claim of calibrated flow.
 int fy=Y(58),fx=X(66);
 screen->drawLine(X(49),fy,fx,fy,HQ_CYAN);
 screen->fillRoundRect(fx,fy-7,18,14,3,HQ_SKY);
 screen->drawRoundRect(fx+2,fy-5,14,10,2,HQ_CYAN);
 screen->drawLine(fx+4,fy,fx+12,fy,HQ_DEEP);
 screen->fillTriangle(fx+14,fy,fx+10,fy-3,fx+10,fy+3,HQ_DEEP);
 // Small legible on-image label: YF and the dedicated cold-water arrow.
 screen->drawFastHLine(fx+18,fy,8,HQ_CYAN);
 screen->fillTriangle(fx+27,fy,fx+23,fy-3,fx+23,fy+3,HQ_CYAN);
 // Output bottle + liquid level as *static illustrative color*, NOT sensor reading
 int jarx=X(74),jary=Y(73),jarw=X(20)-X(0),jarh=Y(22)-Y(0);
 screen->drawRoundRect(jarx,jary,jarw,jarh,3,HQ_SILVER);
 screen->fillRect(jarx+2,jary+jarh/2,jarw-4,jarh/2-2,HQ_GLASS);
 screen->drawFastVLine(jarx+3,jary+3,jarh-6,HQ_SKY);
 screen->drawFastHLine(jarx+2,jary+3,jarw-4,HQ_CYAN);
 for(int j=0;j<4;j++)screen->drawPixel(jarx+5+j*3,jary+jarh-5-j%2,HQ_GLOW);
}
