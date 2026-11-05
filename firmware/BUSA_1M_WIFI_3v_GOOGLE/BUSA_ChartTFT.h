#pragma once
// Display-only shared chart for 240x320 and 320x240. No equipment commands.
static void busaChartText(const String& s,int x,int y,uint16_t color,bool small=false){
 ru.setFont(small?u8g2_font_5x7_t_cyrillic:u8g2_font_6x13_t_cyrillic);ruText(s,x,y,color);ru.setFont(u8g2_font_6x13_t_cyrillic);
}
static void busaDrawChart(uint32_t now){
 const int w=screen->width(),h=screen->height();const bool portrait=h>w;
 const uint16_t bg=tftThemeMode==0?0xffff:tftThemeMode==1?0x0000:0x08c5;
 const uint16_t fg=tftThemeMode==0?0x2147:0xe73e,muted=tftThemeMode==0?0x536e:0x8c97;
 const uint16_t line=tftThemeMode==0?0xcdd8:0x222b,selected=0x0396;
 screen->fillRect(0,28,w,(portrait?274:210)-28,bg);
 busaChartText("ГРАФИКИ",8,portrait?44:40,fg);
 const char* tabs[]={"Температуры","Расход","Питание"};const int ty=portrait?51:46,tw=(w-16)/3;
 for(int i=0;i<3;i++){
  const int x=5+i*(tw+2);screen->fillRoundRect(x,ty,tw,24,4,i==graphSubView?selected:bg);
  screen->drawRoundRect(x,ty,tw,24,4,i==graphSubView&&!tftView?0x07ff:line);
  busaChartText(tabs[i],x+4,ty+16,i==graphSubView?0xffff:fg,true);
 }
 const char* names[]={"Вода","Колонна","Куб","Верх"};const uint16_t colors[]={0x067f,0xfaeb,0xfdc4,0x5f31};
 if(graphSubView==0){
  for(int k=0;k<4;k++){
   const int x=portrait?10+(k%2)*117:10+k*77;const int y=portrait?88+(k/2)*13:84;
   screen->drawFastHLine(x,y-3,9,colors[k]);busaChartText(names[k],x+13,y,fg,true);
  }
 }
 busaChartText(graphSubView==0?"Температура, C":graphSubView==1?"Отбор, мл/мин":"Питание, В",8,portrait?116:97,muted,true);
 const int px=30,py=portrait?123:104,pw=w-41,ph=portrait?107:64;
 for(int i=0;i<=5;i++){
  const int y=py+ph-i*ph/5;screen->drawFastHLine(px,y,pw,line);
  if(graphSubView==0)busaChartText(String(i*25),2,y+3,muted,true);
 }
 const uint8_t hours=BUSA_GRAPH_HOURS[graphHoursIndex%4];
 for(int j=0;j<=3;j++){
  const int x=px+j*(pw-1)/3;screen->drawFastVLine(x,py,ph,line);
  const time_t stamp=time(nullptr)+(time_t)clockUtcOffsetMinutes*60-(time_t)(3-j)*hours*1200;
  struct tm tmv;gmtime_r(&stamp,&tmv);char label[6];strftime(label,sizeof(label),"%H:%M",&tmv);
  busaChartText(deviceClockValid()?String(label):String("--:--"),j==0?px:j==3?w-29:x-12,py+ph+12,muted,true);
 }
 int points=0;
 if(graphSubView==0){
  const int bins=min((int)hours*60,160);int16_t indices[160];busaChartHistory.select(now,hours,indices,bins);
  for(int k=0;k<4;k++){
   int priorX=0,priorY=0;bool prior=false;
   for(int b=0;b<bins;b++){
    const float v=busaChartHistory.value(indices[b],k);
    if(!isfinite(v)){prior=false;continue;}
    ++points;const int x=px+(int)((b+0.5f)*pw/bins),y=py+ph-(int)(fminf(125,fmaxf(0,v))*ph/125);
    if(prior)screen->drawLine(priorX,priorY,x,y,colors[k]);else screen->fillCircle(x,y,1,colors[k]);
    prior=true;priorX=x;priorY=y;
   }
  }
 }
 if(graphSubView!=0||!points){
  const char* msg=graphSubView==1?"Нет датчика отбора":graphSubView==2?"Нет данных питания":"Нет измерений";
  busaChartText(msg,px+5,py+ph/2+4,muted,true);
 }
 const int ry=portrait?247:188,rw=(w-19)/4;
 for(int i=0;i<4;i++){
  const int x=5+i*(rw+3);screen->fillRoundRect(x,ry,rw,21,3,i==graphHoursIndex?selected:bg);
  screen->drawRoundRect(x,ry,rw,21,3,i==graphHoursIndex&&tftView?0x07ff:line);
  busaChartText(String(BUSA_GRAPH_HOURS[i])+" ч",x+rw/2-9,ry+14,i==graphHoursIndex?0xffff:fg,true);
 }
}
static void busaDrawRelayStrip(int y){
 const int w=screen->width(),width=w/4;
 for(int i=0;i<4;i++){
  const bool on=busaSoundActive(i);const uint16_t fg=on?0xffff:0x9cd3,bg=on?(i==3?0x9829:0x134d):0x10e5;
  const int x=i*width+1;screen->fillRoundRect(x,y,width-3,27,3,bg);
  screen->drawRoundRect(x,y,width-3,27,3,on?(i==3?0xf9cd:0x4ed8):0x322c);
  screen->fillCircle(x+6,y+6,2,on?(i==3?0xf9cd:0x4ed8):0x630c);
  busaChartText(BUSA_RELAY_LABELS[i],x+3,y+20,fg,true);
 }
}
