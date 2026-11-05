#pragma once
// Passive TFT drawing; no sensor, network, NVS or GPIO calls.
static void busaCalcText(const String& s,int x,int y,uint16_t c){ru.setFont(u8g2_font_5x7_t_cyrillic);ruText(s,x,y,c);}
static String busaCalcNumber(double v){String s(v,2);s.replace('.',',');return s;}
static void busaCalcDraw(){
 const int w=screen->width(),h=screen->height();const bool portrait=h>w,dark=tftThemeMode==1;
 const int bottom=portrait?274:210;const uint16_t bg=dark?0x0000:0xffff,fg=dark?0xffff:0x2104,accent=dark?0xffe0:0x0338;
 screen->fillRect(0,27,w,bottom-27,bg);
 busaCalcText("КАЛЬКУЛЯТОР",8,38,fg);
 if(!tftView){
  for(int i=0;i<3;i++){int y=62+i*31;if(i==busaCalcMode)screen->fillRoundRect(5,y-13,w-10,24,3,dark?0x2104:0xd69a);
   busaCalcText(String(i==busaCalcMode?"> ":"  ")+busaCalcNames[i],10,y,fg);
  }
  busaCalcText("Вверх/вниз: выбрать; ENT: открыть",8,bottom-22,accent);
 }else{
  busaCalcText(busaCalcNames[busaCalcMode],w/2,38,accent);
  const int gap=portrait?20:16;int y=56;
  for(int i=0;i<busaCalcCounts[busaCalcMode];i++,y+=gap){
   if(i==busaCalcField)screen->fillRoundRect(5,y-11,w-10,gap-1,2,dark?0x2104:0xd69a);
   busaCalcText(busaCalcLabels[busaCalcMode][i],8,y,fg);
   String value=(busaCalcEditing&&i==busaCalcField)?busaCalcEdit:busaCalcInputs[busaCalcMode][i];
   if(!value.length())value="--";
   if(busaCalcEditing&&i==busaCalcField)value=value.substring(0,busaCalcCursor)+"|"+value.substring(busaCalcCursor);
   busaCalcText(value,w-85,y,accent);
  }
  y+=6;
  if(!busaCalcComplete)busaCalcText("Введите все поля; ENT: ввод",8,y,fg);
  else if(!busaCalcResult.ok)busaCalcText("Проверьте диапазоны/доли",8,y,accent);
  else if(busaCalcMode==0){busaCalcText("Долить воды: "+busaCalcNumber(busaCalcResult.water)+" л",8,y,accent);busaCalcText("Объём: "+busaCalcNumber(busaCalcResult.total)+" л",8,y+14,fg);}
  else if(busaCalcMode==1)busaCalcText("При 20 °C: "+busaCalcNumber(busaCalcResult.strength)+" % об.",8,y,accent);
  else{
   busaCalcText("АС: "+busaCalcNumber(busaCalcResult.absolute)+" л",8,y,fg);
   busaCalcText("Головы: "+busaCalcNumber(busaCalcResult.heads)+" л АС",8,y+13,fg);
   busaCalcText("Хвосты: "+busaCalcNumber(busaCalcResult.tails)+" л АС",8,y+26,fg);
   busaCalcText("Выход: "+busaCalcNumber(busaCalcResult.product)+" л",8,y+39,accent);
  }
  busaCalcText(busaCalcEditing?"F1 очистить; 0 >2с запятая; ESC отмена":"ENT ввод; F1 сброс; ESC назад",6,bottom-4,fg);
 }
 ru.setFont(u8g2_font_6x13_t_cyrillic);
}
