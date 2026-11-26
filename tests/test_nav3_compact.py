"""Exercise the actual ArduinoJson filter on a representative status frame."""
from pathlib import Path
import subprocess,sys,tempfile
root=Path(__file__).resolve().parents[1]
main=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
ui=(root/'firmware/BUSA_Display_N4R8/BUSA_TouchUI.h').read_text()
start=main.index('    JsonDocument filter, compact;',main.index('String busaUartStatusJSON(){'))
end=main.index('    if(!filter.overflowed()',start)
f=main[start:end]
assert 'set(clock_,clock.substring(' in ui
assert 'connection_=' not in ui
assert 'set(live_[i],!ok||v.isNull()?' not in ui
assert 'consumeKeys(now);refresh();' in ui
code=r'''
#include <ArduinoJson.h>
#include <cassert>
#include <iostream>
int main(){
''' + f + r'''
const char* sample=R"JSON({"device":"BUSA-1M-001","firmware":"IR1","uptime_ms":123,"display_ota_supported":true,"tft_theme_mode":2,"clock":{"datetime":"26.11.2026 20:10:11"},"temperatures":[{"c":12.5,"gpio":16},{"c":21.5},{"c":33},{"c":44}],"sound_alerts":{"channels":[{"sound_on":false},{"sound_on":true},{"sound_on":false},{"sound_on":true}]},"display_references":[{"target":30,"notice":29},{},{},{}],"calibration_offsets":[0,1,2,3],"water":{"lpm":1.2,"total_l_estimate":4.5,"pulses_per_litre":4710},"collection":{"ml_per_min":null,"total_ml":null,"pulses_per_ml":null},"station":{"connected":true,"ssid":"test"},"network_indicators":{"internet":true},"google":{"configured":true,"state":4},"modules":{"pcf8575":true},"keypad":{"message":"ready","expected":""},"performance":{"i2c_errors":0,"loop_worst_ms":100,"psram_total_bytes":8388608,"unneeded":"long"},"diagnostics":{"water_flow":{"status":"OK"},"esp32":{"free_heap_bytes":1234},"wifi":{"sta_connected":true},"unneeded":"long"},"unneeded":"long"})JSON";
assert(!filter.overflowed());assert(!deserializeJson(compact,sample,DeserializationOption::Filter(filter)));
assert(compact["temperatures"].size()==4);assert(compact["temperatures"][3]["c"]==44);
assert(compact["sound_alerts"]["channels"].size()==4);assert(compact["sound_alerts"]["channels"][3]["sound_on"]==true);
assert(compact["display_references"].size()==4&&compact["calibration_offsets"].size()==4);
assert(compact["water"]["pulses_per_litre"]==4710);assert(compact["diagnostics"]["water_flow"]["status"]=="OK");
assert(compact["unneeded"].isNull()&&compact["performance"]["unneeded"].isNull());
assert(compact["collection"]["ml_per_min"].isNull());
assert(measureJson(compact)<strlen(sample));
std::cout<<"PASS compact status retains four sensors, four sound channels, configuration and nulls\n";
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'test.cpp';p.write_text(code);exe=Path(d)/'test'
    subprocess.run(['g++','-std=c++11','-I'+sys.argv[1],str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
