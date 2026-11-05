"""Run production minute history through 24h, gaps, missing sensors and rollover."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
code=r'''
#include <cassert>
#include <cmath>
#include "BUSA_ChartHistory.h"
int main(){
 BusaChart::Row rows[1441];BusaChart::History history;int16_t bins[160];
 history.select(0,1,bins,60);for(int i=0;i<60;i++)assert(bins[i]==-1);
 history.begin(rows,1441);float a[4]={20,40,60,NAN};
 for(uint32_t m=0;m<=1500;m++){a[0]=20+m/1000.0f;history.capture(m*60000,a);}
 assert(history.size()==1441);
 for(int hours:{1,6,12,24}){
  int n=hours==1?60:160;history.select(1500*60000,hours,bins,n);
  for(int i=0;i<n;i++){assert(bins[i]>=0);assert(history.value(bins[i],0)>20);assert(std::isnan(history.value(bins[i],3)));}
 }
 // A missing sensor replaces its bucket with a gap, never zero or a stale point.
 float missing[4]={NAN,NAN,NAN,NAN};history.capture(1500*60000+5000,missing);
 history.select(1500*60000+5000,1,bins,60);assert(std::isnan(history.value(bins[59],0)));
 // A long sampling interruption remains empty in time rather than squeezed out.
 history.capture(1500*60000+1800000,a);history.select(1500*60000+1800000,1,bins,60);
 for(int i=31;i<59;i++)assert(bins[i]==-1);
 assert(bins[59]>=0);
 // Fine history retains genuine 2s samples across the full hour.
 BusaChart::Row fineRows[1801];BusaChart::History fine;fine.begin(fineRows,1801,0);
 for(uint32_t i=0;i<=1900;i++)fine.capture(i*2000,a);
 assert(fine.size()==1801);
 for(int minutes:{5,10,15,30,60}){
  const int count=minutes==5?150:160;fine.selectMs(3800000,(uint32_t)minutes*60000,bins,count);
  for(int i=0;i<count;i++){assert(bins[i]>=0);assert(isfinite(fine.value(bins[i],0)));}
 }
 BusaChart::History wrap;wrap.begin(rows,1441);uint32_t first=UINT32_MAX-10000;
 wrap.capture(first,a);wrap.capture(first+70000u,a);assert(wrap.size()==2);
 wrap.select(first+70000u,1,bins,60);assert(bins[59]>=0);
}
'''
# initializer_list is explicit for embedded-compatible standard C++11.
code='#include <initializer_list>\n'+code
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'history.cpp';p.write_text(code);exe=Path(d)/'history'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS production fine 2s/one-hour history and legacy minute buckets: all ranges, minute buckets, NaN/gaps, capacity and millis rollover')
