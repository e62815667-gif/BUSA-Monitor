"""Execute production chart and relay rendering at both native screen sizes."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1];fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE';source=(fw/'BUSA_Main.h').read_text()
prefix=(root/'tests/test_calculator_keypad.py').read_text().split("code=r'''",1)[1].split('int currentPage=7',1)[0]
code='#include <cstdint>\n#include <ctime>\n#include <cstdio>\n'+prefix+r'''
using std::min;int tftThemeMode=0,graphSubView=0,graphHoursIndex=0,tftView=0,clockUtcOffsetMinutes=300;
const uint8_t BUSA_GRAPH_HOURS[4]={1,6,12,24};
'''+re.search(r'static const char\* const BUSA_RELAY_LABELS\[4\]=.*?;',source).group()+r'''
#include "BUSA_ChartHistory.h"
BusaChart::History busaChartHistory;uint32_t fakeNow=0;
uint32_t millis(){return fakeNow;}bool deviceClockValid(){return true;}
bool relayOn[4]={false,true,false,true};bool busaSoundActive(uint8_t i){return relayOn[i];}
const int u8g2_font_5x7_t_cyrillic=5,u8g2_font_6x13_t_cyrillic=6;
struct Font{int size=6;void setFont(int v){size=v;}}ru;
struct Display{
 int w=320,h=240;int width(){return w;}int height(){return h;}
 void rect(int x,int y,int ww,int hh){assert(x>=0&&y>=0&&ww>=0&&hh>=0&&x+ww<=w&&y+hh<=h);}
 void fillRect(int x,int y,int ww,int hh,uint16_t){rect(x,y,ww,hh);}
 void fillRoundRect(int x,int y,int ww,int hh,int,uint16_t){rect(x,y,ww,hh);}
 void drawRoundRect(int x,int y,int ww,int hh,int,uint16_t){rect(x,y,ww,hh);}
 void drawFastHLine(int x,int y,int ww,uint16_t){rect(x,y,ww,1);}
 void drawFastVLine(int x,int y,int hh,uint16_t){rect(x,y,1,hh);}
 void drawLine(int a,int b,int c,int d,uint16_t){rect(a,b,1,1);rect(c,d,1,1);}
 void fillCircle(int x,int y,int r,uint16_t){rect(x-r,y-r,r*2+1,r*2+1);}
}display,*screen=&display;
void ruText(const String& s,int x,int y,uint16_t){int chars=0;for(unsigned char c:s)if((c&0xc0)!=0x80)++chars;assert(x>=0&&x+chars*ru.size<=screen->w&&y>=7&&y<screen->h);}
#include "BUSA_ChartTFT.h"
int main(){
 BusaChart::Row rows[1441];busaChartHistory.begin(rows,1441);float v[4]={24,78,96,NAN};
 for(int m=0;m<1440;m++){fakeNow=m*60000;v[0]+=0.01f;busaChartHistory.capture(fakeNow,v);}
 for(int portrait=0;portrait<2;portrait++)for(tftThemeMode=0;tftThemeMode<3;tftThemeMode++)for(graphSubView=0;graphSubView<3;graphSubView++)for(graphHoursIndex=0;graphHoursIndex<4;graphHoursIndex++)for(tftView=0;tftView<2;tftView++){
  display.w=portrait?240:320;display.h=portrait?320:240;busaDrawChart(fakeNow);
  busaDrawRelayStrip(portrait?242:180);
 }
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'tft.cpp';p.write_text(code);exe=Path(d)/'tft'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(fw),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS production TFT chart/relay geometry: 144 orientation/theme/tab/range/focus combinations; no out-of-screen text or draw commands')
