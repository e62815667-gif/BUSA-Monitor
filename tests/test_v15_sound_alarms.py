"""Execute the shipped C++ sound annunciator hysteresis with a desktop compiler."""
from pathlib import Path
import subprocess, tempfile
src=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE").resolve()
body=r'''
#include "BUSA_V15_AlarmLogic.h"
#include <cassert>
#include <cstring>
int main(){
  using namespace BusaAlarm;
  Channel c[4];
  for(int i=0;i<4;i++){assert(!c[i].on);assert(c[i].reason==Disabled);}
  // Heating direction: wait, alarm on, hysteresis hold, off.
  assert(!evaluate(c[0],true,true,49.8f,50.0f,47.0f));
  assert(evaluate(c[0],true,true,50.0f,50.0f,47.0f));
  assert(evaluate(c[0],true,true,48.0f,50.0f,47.0f));
  assert(!evaluate(c[0],true,true,47.0f,50.0f,47.0f));
  // Cooling direction: trigger below the lower turn-on threshold.
  assert(!evaluate(c[1],true,true,25.0f,20.0f,24.0f));
  assert(evaluate(c[1],true,true,20.0f,20.0f,24.0f));
  assert(evaluate(c[1],true,true,22.0f,20.0f,24.0f));
  assert(!evaluate(c[1],true,true,24.0f,20.0f,24.0f));
  // Sound must stop if sensor loses verifiability or value.
  assert(evaluate(c[2],true,true,80.0f,75.0f,72.0f));
  assert(!evaluate(c[2],true,false,81.0f,75.0f,72.0f));
  assert(c[2].reason==SensorInvalid);
  assert(!evaluate(c[2],true,true,NAN,75.0f,72.0f));
  assert(!evaluate(c[2],true,true,60.0f,75.0f,72.0f));
  // Disabling, missing thresholds and inadequate separation fail silent.
  assert(!evaluate(c[3],false,true,100.0f,80.0f,70.0f));
  assert(!evaluate(c[3],true,true,100.0f,NAN,70.0f));
  assert(c[3].reason==MissingThreshold);
  assert(!validPair(50.0f,50.0f));
  assert(!validPair(50.0f,49.7f));
  assert(validPair(50.0f,49.5f));
  assert(!validPair(-1.0f,30.0f));
  assert(!validPair(126.0f,30.0f));
  // Independent channels: toggling channel 0 leaves channel 1 unchanged.
  assert(evaluate(c[1],true,true,20.0f,20.0f,24.0f));
  assert(!evaluate(c[0],false,true,80.0f,50.0f,47.0f));
  assert(c[1].on);
  assert(strcmp(reasonText(c[1].reason),"SOUNDING")==0);
  return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
  path=Path(d)/"test.cpp"
  path.write_text(body)
  exe=Path(d)/"test"
  subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror","-I"+str(src),str(path),"-o",str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
main=(src/"BUSA_Main.h").read_text()
runtime=(src/"BUSA_V15_Runtime.h").read_text()
driver=(src/"BUSA_V15_Alarm.h").read_text()
status=(src/"BUSA_V15_Status.h").read_text()
assert "15v1-N8R2-FOUR-SOUND-ALARMS" in main
assert 'apPassword="123456789";' in runtime
assert 'WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str()' in runtime
assert "alarmInitPins();" in runtime and "alarmLoadSettings();" in runtime
assert 'server.on("/api/alarm/config",HTTP_POST' in runtime
assert 'prefs.putBool(key.c_str(),requested[i])' in runtime
assert 'server.arg("confirm")!="SOUND_ALARMS_ONLY"' in runtime
assert "alarmPoll(now)" in runtime and "alarmOutputsOff()" in runtime
assert "sound_alarms" in status and "alarmStatusJSON(now)" in status
assert "id=\"alarmSave\"" in main and "alarmEnable0" in main
assert 'digitalWrite(' not in driver
assert 'pinMode(KPIN[i],INPUT);' in driver
assert 'HardwareLocked' in driver
assert 'prefs.getBool(' not in driver
assert "KPIN[4]={7,6,5,4}" in (src/"BUSA_V15_Board.h").read_text()
print("PASS V15.1: C++ hot/cold threshold hysteresis, 4 independent sound channels, stale sensor off, hysteresis retained but physical GPIO driver locked")
