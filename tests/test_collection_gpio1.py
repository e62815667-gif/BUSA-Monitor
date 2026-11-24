"""GPIO1 measurement contract: host-execute the production accounting code."""
from pathlib import Path
import subprocess, tempfile

root = Path(__file__).resolve().parents[1]
fw = root / 'firmware/BUSA_1M_WIFI_3v_GOOGLE'
main = (fw / 'BUSA_Main.h').read_text()
assert 'constexpr uint8_t COLLECTION_GPIO=1;' in main, 'GPIO1 input not implemented'
assert 'attachInterrupt(digitalPinToInterrupt(COLLECTION_GPIO),collectionISR,FALLING);' in main
assert 'attachInterrupt(digitalPinToInterrupt(FLOW_GPIO),flowISR,FALLING);' in main
assert 'sampleCollection(now);' in main
assert 'prefs.putFloat("colPulsesMl",value)' in main
assert 'prefs.getFloat("colPulsesMl",NAN)' in main
assert 's+=",\\"collection\\":"+collectionJSON();' in main
code = r'''
#include "BUSA_CollectionPulse.h"
#include <cassert>
#include <cmath>
int main(){
    BusaCollectionPulse a,b;
    a.begin(100,0); b.begin(100,0);
    assert(!a.seen() && !a.calibrated() && std::isnan(a.rate()) && std::isnan(a.volume()));
    assert(!a.calibrate(0) && !a.calibrate(-1) && !a.calibrate(NAN) && !a.calibrate(INFINITY));
    assert(a.calibrate(2) && b.calibrate(4));
    assert(a.calibrated() && !a.seen() && std::isnan(a.volume()));
    assert(!a.sample(2099,10));
    assert(a.sample(2100,10) && a.seen() && a.pulses()==10);
    assert(std::fabs(a.rate()-150)<1e-9 && std::fabs(a.volume()-5)<1e-9);
    assert(!b.seen() && b.pulses()==0 && std::isnan(b.rate()));
    assert(b.sample(2100,8) && std::fabs(b.volume()-2)<1e-9);
    assert(a.sample(4100,10) && a.rate()==0 && a.volume()==5);
    assert(!a.calibrate(NAN) && a.calibrated() && a.volume()==5);
    BusaCollectionPulse raw; raw.begin(0,0); raw.sample(2000,12);
    assert(raw.seen() && raw.pulses()==12 && std::isnan(raw.rate()) && std::isnan(raw.volume()));
    assert(raw.calibrate(3) && raw.volume()==4 && raw.rate()==120);
    BusaCollectionPulse wrap; wrap.begin(UINT32_MAX-999,UINT32_MAX-2);
    assert(wrap.calibrate(1)); assert(wrap.sample(1000,2));
    assert(wrap.pulses()==5 && wrap.rate()==150 && wrap.volume()==5);
    BusaCollectionPulse large;large.begin(0,0);large.calibrate(1);
    large.sample(2000,UINT32_MAX);large.sample(4000,3);
    assert(large.pulses()==uint64_t(UINT32_MAX)+4);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    src = Path(tmp) / 'test.cpp'; exe = Path(tmp) / 'test'
    src.write_text(code)
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-I'+str(fw),str(src),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS GPIO1 independent accounting, calibration, no invented readings, counter/time wrap')
