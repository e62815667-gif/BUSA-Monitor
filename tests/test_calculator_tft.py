"""Execute actual TFT renderer/palette against native-size geometry spies.
No physical display or optical/image-quality test is implied.
"""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1];s=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
key=(root/'tests/test_calculator_keypad.py').read_text().split("code=r'''",1)[1].split('int currentPage=7',1)[0]
key=key.replace('String(int n):', 'String(double v,int n){char b[64];snprintf(b,sizeof(b),"%.*f",n,v);assign(b);}\n String(int n):')
land=(s/'BUSA_Landscape_V14.h').read_text();palette=land[land.index('static uint16_t V14_TEXT'):land.index('static uint32_t v14FlowLast')]
start=land.index('static void v14ApplyPalette(');end=land.index('static void v14Print(',start);palette+='\n'+land[start:end]
hq=sorted(set(re.findall(r'\bHQ_[A-Z_]+\b',palette)) - {'HQ_MONO_DARK'})
code='#include <cstdio>\n'+key+'\n#include <cstdint>\n#include <vector>\n'
code+='uint16_t '+','.join(hq)+';bool HQ_MONO_DARK;int tftThemeMode;\n'+palette
code+=r'''
struct Display{
 int w=320,h=240;uint16_t firstBg=0;int rects=0;
 int width(){return w;}int height(){return h;}
 void fillRect(int x,int y,int ww,int hh,uint16_t c){assert(x>=0&&y>=27&&x+ww<=w&&y+hh<=h-27);if(!rects++)firstBg=c;}
 void fillRoundRect(int x,int y,int ww,int hh,int,uint16_t c){fillRect(x,y,ww,hh,c);}
} display,*screen=&display;
const int u8g2_font_6x13_t_cyrillic=13;
struct Ru{void setFont(int f){assert(f==13);}}ru;
void ruText(const String& str,int x,int y,uint16_t){
 int chars=0;for(unsigned char c:str)if((c&0xc0)!=0x80)chars++;
 assert(x>=0&&x+chars*6<=screen->w);assert(y-11>=27&&y+2<=screen->h-27);
}
int currentPage=7,tftView=0;bool tftDirty=false;String keyMessage;
#include "BUSA_CalculatorUI.h"
#include "BUSA_CalculatorTft.h"
int main(){
 for(int portrait=0;portrait<2;portrait++)for(tftThemeMode=0;tftThemeMode<3;tftThemeMode++)for(int mode=0;mode<3;mode++){
  display.w=portrait?240:320;display.h=portrait?320:240;busaCalcMode=mode;
  for(tftView=0;tftView<2;tftView++)for(int state=0;state<4;state++){
   busaCalcComplete=state>=1;busaCalcResult.ok=state>=2;busaCalcEditing=state==3;
   busaCalcEdit="123456789012";busaCalcCursor=12;busaCalcField=busaCalcCounts[mode]-1;
   display.rects=0;busaCalcDraw();
   assert(display.firstBg==V14_PALE);
   assert(display.firstBg==(tftThemeMode==0?0xffff:tftThemeMode==1?0x0000:0x08c5));
  }
 }
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'tft.cpp';p.write_text(code);exe=Path(d)/'tft'
 subprocess.run(['g++','-std=c++11','-I'+str(s),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS actual TFT drawing: 144 menu/form/error/result/editor layouts, 320x240/240x320, all three real palettes; 6x13 text bounds. No hardware/optical test.')
