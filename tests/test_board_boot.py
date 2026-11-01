"""Exercise retained crash markers and incremental I2C against host mocks."""
from pathlib import Path
import subprocess,tempfile
r=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
common=r'''
#include <string>
#include <cstdint>
#include <cassert>
#include <cstring>
class String:public std::string{public:using std::string::string;String(const std::string&s):std::string(s){};String(uint32_t n):std::string(std::to_string(n)){};};
struct Print{virtual ~Print()=default;virtual size_t write(uint8_t)=0;template<class...T>void printf(const char*,T...){};};
struct Port{int writes=0;bool host=false;size_t write(uint8_t){++writes;return 1;}int availableForWrite(){return 0;}operator bool()const{return host;}}Serial0,Serial;
#define RTC_DATA_ATTR
using esp_reset_reason_t=int;
enum{ESP_RST_POWERON,ESP_RST_PANIC,ESP_RST_INT_WDT,ESP_RST_TASK_WDT,ESP_RST_WDT,ESP_RST_SW};
int reason=ESP_RST_POWERON;int esp_reset_reason(){return reason;}
'''
boot=r'''
int main(){
 busaBootBegin();assert(!busaSkipTft && !busaSkipI2c);
 busaBootPhase(1);reason=ESP_RST_PANIC;busaBootBegin();assert(!busaSkipTft && !busaSkipI2c && busaBootMarker.failuresTft==1);
 busaBootPhase(1);reason=ESP_RST_PANIC;busaBootBegin();assert(busaSkipTft && !busaSkipI2c && busaBootMarker.failuresTft==2);
 busaBootPhase(2);reason=ESP_RST_TASK_WDT;busaBootBegin();assert(busaSkipTft && !busaSkipI2c && busaBootMarker.failuresI2c==1);
 busaBootPhase(2);reason=ESP_RST_TASK_WDT;busaBootBegin();assert(busaSkipTft && busaSkipI2c && busaBootMarker.failuresI2c==2);
 reason=ESP_RST_SW;busaSkipTft=busaSkipI2c=false;busaBootBegin();assert(!busaSkipTft && !busaSkipI2c);
 busaBootPhase(1);reason=ESP_RST_POWERON;busaBootBegin();assert(!busaSkipTft);
 busaLog.write(65);assert(Serial0.writes==1 && Serial.writes==0);
 Serial.host=true;busaLog.write(66);assert(Serial0.writes==2 && Serial.writes==0);
}
'''
i2cstub=r'''
bool i2cOnline=true;uint8_t i2cFound[24]={},i2cFoundCount=0;uint16_t i2cAckCount=0;
uint32_t i2cFullScanAt=0;constexpr int BUS_SDA=8,BUS_SCL=9,HIGH=1;
int calls=0;bool released=true;int digitalRead(int){return released?HIGH:0;}
bool elapsed(uint32_t a,uint32_t b,uint32_t c){return uint32_t(a-b)>=c;}
bool ack(uint8_t a){++calls;return a==0x20 || a==0x40;}
'''
i2ctest=r'''
int main(){
 for(int i=0;i<111;i++){scanI2cAddresses(100+100*i);assert(calls==i+1);assert(i2cFoundCount==0);}
 scanI2cAddresses(11200);assert(calls==112 && i2cFoundCount==2 && i2cAckCount==2);
 assert(i2cFound[0]==0x20 && i2cFound[1]==0x40);
 scanI2cAddresses(11201);assert(calls==112);
 released=false;scanI2cAddresses(80000);assert(calls==112);
 released=true;scanI2cAddresses(80001);assert(calls==113 && i2cFoundCount==2);
}
'''
main=(r/'BUSA_Main.h').read_text()
start=main.index('uint8_t i2cScanAddress=');end=main.index('\nint readPcaReg',start)
with tempfile.TemporaryDirectory() as d:
 for name,src in [('boot',common+'\n#include "BUSA_Boot.h"\n'+boot),('i2c',common+i2cstub+main[start:end]+i2ctest)]:
  p=Path(d)/(name+'.cpp');p.write_text(src);exe=Path(d)/name
  subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(r),str(p),'-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
print('PASS: crash recovery, no USB wait, one-address I2C scans, stuck-bus handling')
