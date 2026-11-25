"""Execute the production capture scheduler and history API binning under loop jitter."""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[1]
fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
def function(source, signature):
    a=source.index(signature); p=source.index('{',a)+1; depth=1
    while depth:
        if source[p]=='{': depth+=1
        elif source[p]=='}': depth-=1
        p+=1
    return source[a:p]
main=(fw/'BUSA_Main.h').read_text()
history=(fw/'BUSA_History.h').read_text()
code=r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <cstdio>
#include "BUSA_ChartHistory.h"
using String=std::string;
const int MALLOC_CAP_SPIRAM=1,MALLOC_CAP_8BIT=2;
void* heap_caps_malloc(size_t n,int){return std::malloc(n);}
float temperature[4]={20,21,22,23};bool tempPresent[4]={true,true,true,true};
bool hasTemp(int k,uint32_t){return tempPresent[k];}
bool flowReady=true,flowSeen=false,pcfDetected=true;float flowLpm=0;
struct Meter{double raw(){return 0;}} collectionMeter;
const int WL_CONNECTED=1;struct Wifi{int status(){return 1;}} WiFi;
bool elapsed(uint32_t now,uint32_t last,uint32_t span){return uint32_t(now-last)>=span;}
void v13Log(const String&){}
static float v13History[4][48];static uint8_t v13HistoryCount=0,v13HistoryHead=0;
static uint32_t v13LastCapture=0,v13LastCollectAt=0;
static bool v13EventBoot=false,v13OldWifi=false,v13OldPcf=false,v13OldTemps[4]={},v13OldFlow=false;
'''
# Only the unrelated Arduino String journal expression is adapted to the host.
scheduler=function(history,'static void v13Collect(').replace('String("T")+String(i+1)', 'String("T")+std::to_string(i+1)')
code+=main[main.index('static BusaChart::History busaChartHistory;'):main.index('static void busaCaptureChart(')]+function(main,'static void busaCaptureChart(')+scheduler+r'''
int main(){
 const uint32_t start=UINT32_MAX-45000u;
 const uint32_t jitter[]={0,50,900,0,90};
 for(unsigned i=0;i<=200;++i)v13Collect(start+i*2000+jitter[i%5]);
 int16_t bins[150];
 for(uint32_t phase:{50u,500u,1050u,1999u}){
  busaChartHistory.selectMs(start+400000+phase,300000,bins,150);
  for(int i=0;i<150;++i){assert(bins[i]>=0);assert(busaChartHistory.coolingValue(bins[i])==0);assert(busaChartHistory.value(bins[i],0)==20);}
 }
 tempPresent[0]=false;flowReady=false;v13Collect(start+408000);
 busaChartHistory.selectMs(start+408000,300000,bins,150);
 assert(bins[146]<0&&bins[147]<0&&bins[148]<0&&bins[149]>=0);
 assert(std::isnan(busaChartHistory.value(bins[149],0)));
 assert(std::isnan(busaChartHistory.coolingValue(bins[149])));
 assert(busaChartHistory.value(bins[149],1)==21);
 std::puts("PASS jitter and rollover: no artificial gaps, actual missing intervals and invalid data retained");
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'schedule.cpp';p.write_text(code);exe=Path(d)/'schedule'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(fw),str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
