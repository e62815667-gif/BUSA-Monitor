"""Execute production GPIO1 measurement math without ESP32 hardware."""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[1]
fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
assert (fw/'BUSA_Collection.h').is_file(), 'GPIO1 measurement implementation missing'
code=r'''
#include <cassert>
#include <cmath>
#include "BUSA_Collection.h"
int main(){
 BusaCollection::Meter m, other;
 m.begin(100);assert(!m.seen()&&!m.calibrated()&&isnan(m.rate())&&isnan(m.total()));
 m.sample(2100,20);assert(m.seen()&&m.pulses()==20&&isnan(m.rate())&&isnan(m.total()));
 assert(!m.setCalibration(0)&&!m.setCalibration(-1)&&!m.setCalibration(NAN)&&!m.setCalibration(INFINITY));
 assert(m.setCalibration(10));assert(fabs(m.rate()-60)<.001&&fabs(m.total()-2)<.001);
 m.sample(2200,25);assert(m.pulses()==20);
 m.sample(4100,25);assert(m.pulses()==25&&fabs(m.rate()-15)<.001&&fabs(m.total()-2.5)<.001);
 m.sample(6100,25);assert(m.rate()==0&&fabs(m.total()-2.5)<.001);
 assert(!other.seen()&&isnan(other.total()));assert(m.setCalibration(5));assert(fabs(m.total()-5)<.001);
 m.resetTotal(30);assert(m.total()==0&&m.rate()==0&&m.calibration()==5);m.sample(8100,35);assert(fabs(m.total()-1)<.001&&m.pulses()==35);assert(!other.seen()&&isnan(other.total()));
 BusaCollection::Meter wrap;wrap.begin(UINT32_MAX-999);wrap.setCalibration(2);wrap.sample(1000,4);
 assert(fabs(wrap.rate()-60)<.001&&wrap.total()==2);
 BusaCollection::Meter idle;idle.begin(0);idle.setCalibration(10);idle.sample(2000,0);assert(!idle.seen()&&isnan(idle.rate())&&isnan(idle.total()));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';p.write_text(code);exe=Path(d)/'test'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(fw),str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('PASS independent measurement, uncalibrated null, calibration, zero flow, millis rollover')
