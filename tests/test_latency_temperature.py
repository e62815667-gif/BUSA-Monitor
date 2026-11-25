from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[1]
main=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
def function(signature):
    a=main.index(signature);p=main.index('{',a)+1;depth=1
    while depth:
        depth+=(main[p]=='{')-(main[p]=='}');p+=1
    return main[a:p]
code=r'''
#include <cassert>
#include <cmath>
#include <cstdint>
using std::isfinite;
using DeviceAddress=uint8_t[8];const float DEVICE_DISCONNECTED_C=-127;
uint32_t clockNow=0; unsigned ops=0;
uint32_t millis(){return clockNow;}
struct Sensor {
 bool attached=true,wait=true;uint8_t rom=1;float value=20;int scans=0,requests=0;uint32_t requestedAt=0;
 void begin(){++ops;++scans;}
 bool getAddress(uint8_t*out,int){++ops;out[0]=rom;return attached;}
 void setResolution(uint8_t*,int n){++ops;assert(n==12);}
 void setWaitForConversion(bool b){wait=b;}
 void requestTemperatures(){++ops;assert(!wait);++requests;requestedAt=clockNow;}
 float getTempC(uint8_t*cached){++ops;assert(uint32_t(clockNow-requestedAt)>=825);return attached&&cached[0]==rom?value:DEVICE_DISCONNECTED_C;}
} devices[4];Sensor* sensors[4]={&devices[0],&devices[1],&devices[2],&devices[3]};
float temperature[4]={NAN,NAN,NAN,NAN},tempOffset[4]={};
bool tempPresent[4]={},tempInitialized[4]={},cachedTempValid[4]={},tempConverting=false;
uint32_t tempAt[4]={},tempCycle=0,tempRequestAt=0,tempDiscoverAt[4]={};DeviceAddress cachedTempRom[4]={};
'''+function('bool elapsed(')+function('void sampleTemperatures(')+function('bool hasTemp(int n,uint32_t now){')+r'''
void pump(uint32_t start,uint32_t end){for(uint32_t t=start;t<=end;++t){clockNow=t;ops=0;sampleTemperatures(t);assert(ops<=1);}}
int main(){
 devices[1].attached=false;tempOffset[2]=1.2;
 pump(100,150);assert(!hasTemp(0,150));pump(151,900);assert(!hasTemp(0,900));
 pump(901,1200);assert(hasTemp(0,1200)&&fabs(temperature[2]-21.2)<.001&&!hasTemp(1,1200));
 devices[0].attached=false;pump(1201,4200);assert(!hasTemp(0,4200)&&!tempPresent[0]);
 devices[0].attached=true;devices[0].rom=9;devices[0].value=44.5;devices[1].attached=true;devices[1].value=33;
 pump(4201,18000);assert(hasTemp(0,18000)&&fabs(temperature[0]-44.5)<.001&&cachedTempRom[0][0]==9);
 assert(hasTemp(1,18000)&&devices[0].scans==2&&devices[1].scans==2);
 devices[0].value=NAN;pump(18001,22000);assert(!hasTemp(0,22000));
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'test.cpp';p.write_text(code);exe=Path(d)/'test'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: real cooperative temperature function; one bus operation per invocation, 12 bit, conversion wait, missing/replaced sensors and invalid samples')
