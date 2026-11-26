"""Execute audited production code with fault injection; no physical outputs/network."""
from pathlib import Path
import subprocess,tempfile,json,re
root=Path(__file__).resolve().parents[1];fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE';main=(fw/'BUSA_Main.h').read_text()
def function(source,signature):
 a=source.index(signature);p=source.index('{',a)+1;depth=1
 while depth:
  if source[p]=='{':depth+=1
  elif source[p]=='}':depth-=1
  p+=1
 return source[a:p]
def run(code,name):
 with tempfile.TemporaryDirectory() as d:
  p=Path(d)/(name+'.cpp');p.write_text(code);exe=Path(d)/name
  subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Wno-unused-function','-pthread','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I'+str(fw),str(p),'-o',str(exe)],check=True)
  return subprocess.check_output([str(exe)],text=True)
common=r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <map>
#include <vector>
using std::isfinite;
class String:public std::string {public:using std::string::string;String(const std::string&s):std::string(s){}String(int n):std::string(std::to_string(n)){}unsigned length()const{return size();}};
String operator+(const String&s,int i){return String(std::string(s)+std::to_string(i));}
'''
raw=run(common+function(main,'String quoteJson(')+r'''
int main(){
 String s;for(int c=0;c<32;c++)s+=(char)c;
 s+="Куб\t\"\\";std::cout<<quoteJson(s);
}
''','json')
assert json.loads(raw)==''.join(chr(c) for c in range(32))+'Куб\t"\\'
# Execute the exact delay predicates from loop with a handler-set timestamp.
predicates=[]
for at in ['otaRestartAt','googleRestartAt','tftRebootAt']:
 expression=re.search(r'elapsed\([^\n;]*?,'+at+r',\d+\)',main).group()
 predicates.append(expression)
run(common+function(main,'bool elapsed(')+r'''
uint32_t clockNow=0,otaRestartAt=0,googleRestartAt=0,tftRebootAt=0;
uint32_t millis(){return clockNow;}
bool ready(){return '''+'||'.join(predicates)+r''';}
int main(){
 for(uint32_t start:{1001u,UINT32_MAX-1000u}){
  otaRestartAt=googleRestartAt=tftRebootAt=start;
  clockNow=start;assert(!ready());clockNow=start+2999;assert(!ready());
  clockNow=start+4000;assert(ready());
 }
}
''','restart')
# Actual four-bus discovery/conversion function, simulated disconnect/replacement.
run(common+r'''
using DeviceAddress=uint8_t[8];const float DEVICE_DISCONNECTED_C=-127;
uint32_t nowMs=0;uint32_t millis(){return nowMs;}
struct Sensor {
 bool attached=true;uint8_t rom=1;float value=20;int scans=0,requests=0;
 void begin(){++scans;}bool getAddress(uint8_t*out,int){out[0]=rom;return attached;}
 void setResolution(uint8_t*,int){}void setWaitForConversion(bool){}
 void requestTemperatures(){++requests;}float getTempC(uint8_t*cached){return attached&&cached[0]==rom?value:DEVICE_DISCONNECTED_C;}
} devices[4];Sensor* sensors[4]={&devices[0],&devices[1],&devices[2],&devices[3]};
float temperature[4]={NAN,NAN,NAN,NAN},tempOffset[4]={};
bool tempPresent[4]={},tempInitialized[4]={},cachedTempValid[4]={},tempConverting=false;
uint32_t tempAt[4]={},tempCycle=0,tempRequestAt=0,tempDiscoverAt[4]={};DeviceAddress cachedTempRom[4]={};
'''+function(main,'bool elapsed(')+function(main,'void sampleTemperatures(')+function(main,'bool hasTemp(int n,uint32_t now){')+r'''
int main(){
 devices[1].attached=false;tempOffset[2]=1.2;
 auto pump=[](uint32_t start,uint32_t end){for(uint32_t t=start;t<=end;++t){nowMs=t;sampleTemperatures(t);}};
 pump(100,924);assert(!hasTemp(0,924));pump(925,1200);
 assert(hasTemp(0,1200)&&fabs(temperature[2]-21.2)<.001&&!hasTemp(1,1200));
 devices[0].attached=false;pump(1201,4200);
 assert(!hasTemp(0,4200)&&!tempPresent[0]);
 devices[0].attached=true;devices[0].rom=9;devices[0].value=44.5;
 devices[1].attached=true;devices[1].value=33;
 pump(4201,18000);
 assert(hasTemp(0,18000)&&fabs(temperature[0]-44.5)<.001&&cachedTempRom[0][0]==9);
 assert(hasTemp(1,18000)&&devices[0].scans==2&&devices[1].scans==2);
 devices[0].value=NAN;
 pump(18001,22000);
 assert(!hasTemp(0,22000));
}
''','sensors')
# Real mailbox, ownership and immutable payload under concurrent producer/consumer.
run(common+r'''
#include "BUSA_SnapshotMailbox.h"
struct Frame {static std::atomic<int> live;unsigned seq;std::string json;Frame(unsigned n):seq(n),json(128,char(n%26+'a')){++live;}~Frame(){--live;}};
std::atomic<int> Frame::live{0};
int main(){
 {
  BusaSnapshotMailbox<Frame> box;assert(!box.needsSnapshot());box.request();assert(box.needsSnapshot());assert(!box.needsSnapshot());
  box.publish(new Frame(1));box.publish(new Frame(2));auto* f=box.take();assert(f&&f->seq==2);delete f;assert(!box.take());
  std::atomic<bool> done{false};
  std::thread writer([&]{for(unsigned i=0;i<50000;i++)box.publish(new Frame(i));done=true;});
  while(!done){auto* value=box.take();if(value){assert(value->json==std::string(128,char(value->seq%26+'a')));delete value;}}
  writer.join();delete box.take();
 }
 assert(Frame::live==0);
}
''','mailbox')
google=(fw/'BUSA_Google.h').read_text();worker=google[google.index('void googleWorker('):google.index('void addGoogleRoutes()')]
assert 'statusJSON(' not in worker and 'snapshot->json' in worker
assert 'busaGoogleSnapshotPoll();' in main[main.index('void loop(){'):]
# Actual sound state machine: hysteresis, T4 priority and return to T3, exclusive 30s sounds, expiry, rollover.
run(common+r'''
#include "BUSA_SoundAlarmLogic.h"
int main(){
 BusaSound::Channel c;
 assert(!BusaSound::evaluate(c,true,true,79,80,78));
 assert(BusaSound::evaluate(c,true,true,80,80,78));
 assert(BusaSound::evaluate(c,true,true,79,80,78));
 assert(!BusaSound::evaluate(c,true,true,78,80,78));
 assert(BusaSound::evaluate(c,true,true,28,28,30));
 assert(!BusaSound::evaluate(c,true,false,NAN,28,30));
 assert(!BusaSound::evaluate(c,false,true,27,28,30));
 for(uint32_t start:{100u,UINT32_MAX-1000u}){
  BusaSound::Routed r;bool en[4]={true,true,true,true};
  auto tick=[&](float t3,float t4,uint32_t at){BusaSound::evaluateRouted(r,en,true,t3,96,94,true,t4,85,80,at);};
  auto expect=[&](bool k1,bool k2,bool k3,bool k4){
    assert(r.k[0]==k1&&r.k[1]==k2&&r.k[2]==k3&&r.k[3]==k4);
    assert(!(r.k[1]&&r.k[2]));
  };
  tick(96,70,start);expect(true,true,false,false);
  tick(95,70,start+1);expect(true,true,false,false);assert(!r.t3OnEvent);
  tick(94,70,start+100);expect(false,false,true,false);
  tick(94,70,start+30099);expect(false,false,true,false);
  tick(94,70,start+30100);expect(false,false,false,false);
  tick(96,70,start+30200);expect(true,true,false,false);
  tick(96,85,start+30300);expect(false,true,false,true);assert(r.t4OnEvent);
  tick(94,85,start+30400);expect(false,true,false,true);assert(r.t3OffEvent);
  tick(96,85,start+30500);expect(false,true,false,true);assert(r.t3OnEvent);
  tick(96,85,start+60299);expect(false,true,false,true);
  tick(96,85,start+60300);expect(false,false,false,true);
  // T3 already active: T4 OFF must start a fresh K2 pulse, not K3.
  tick(96,80,start+60400);expect(true,true,false,false);assert(r.t4OffEvent&&!r.t3OnEvent);
  tick(95,80,start+90399);expect(true,true,false,false);
  tick(95,80,start+90400);expect(true,false,false,false);
  // Simultaneous T3 OFF + T4 ON: emergency wins.
  tick(94,85,start+90500);expect(false,true,false,true);
  // T3 still inactive: T4 OFF gives K3 for exactly 30 seconds.
  tick(94,80,start+90600);expect(false,false,true,false);
  tick(94,80,start+120599);expect(false,false,true,false);
  tick(94,80,start+120600);expect(false,false,false,false);
  // T3 ON interrupts K3 and starts K2. T4 ON interrupts normal signals.
  tick(96,70,start+120700);expect(true,true,false,false);
  tick(94,70,start+120800);expect(false,false,true,false);
  tick(94,85,start+120900);expect(false,true,false,true);
  // Simultaneous T3 ON + T4 OFF: return to T3, restart K2.
  tick(96,80,start+121000);expect(true,true,false,false);
  tick(96,80,start+150999);expect(true,true,false,false);
  tick(96,80,start+151000);expect(true,false,false,false);
  // T3 remains active in hysteresis band after emergency.
  tick(95,85,start+151100);expect(false,true,false,true);
  tick(95,80,start+151200);expect(true,true,false,false);
  tick(94,80,start+151300);expect(false,false,true,false);
  en[0]=en[1]=en[2]=en[3]=false;tick(96,90,start+151400);
  for(bool on:r.k)assert(!on);
  BusaSound::Routed invalid;BusaSound::evaluateRouted(invalid,en,false,NAN,96,94,false,NAN,85,80,start);
  assert(!invalid.t3OffEvent&&!invalid.t4OffEvent);
  // Owner's exact 30/29 C scenario, with T3 crossing during emergency.
  BusaSound::Routed example;bool enabled[4]={true,true,true,true};
  BusaSound::evaluateRouted(example,enabled,true,28,30,29,true,31,30,29,start);
  BusaSound::evaluateRouted(example,enabled,true,31,30,29,true,31,30,29,start+100);
  assert(!example.k[0]&&example.k[1]&&!example.k[2]&&example.k[3]);
  BusaSound::evaluateRouted(example,enabled,true,31,30,29,true,29,30,29,start+200);
  assert(example.k[0]&&example.k[1]&&!example.k[2]&&!example.k[3]);
  assert(BusaSound::remainingMs(example.k2,start+200,BusaSound::ROUTED_PULSE_MS)==30000);
  BusaSound::evaluateRouted(example,enabled,true,31,30,29,true,29,30,29,start+30200);
  assert(example.k[0]&&!example.k[1]&&!example.k[2]&&!example.k[3]);
  BusaSound::Routed lost;bool all[4]={true,true,true,true};
  BusaSound::evaluateRouted(lost,all,true,96,96,94,true,85,85,80,start);
  assert(!lost.k[0]&&lost.k[1]&&!lost.k[2]&&lost.k[3]);
  BusaSound::evaluateRouted(lost,all,true,96,96,94,false,NAN,85,80,start+1);
  assert(!lost.t4OffEvent&&!lost.k[2]&&!lost.k[0]);
 }
}
''','sound')
# NVS failure at every write, including the final command ID; previous flags restored.
run(common+r'''
using portMUX_TYPE=int;const int portMUX_INITIALIZER_UNLOCKED=0;
void portENTER_CRITICAL(int*){}void portEXIT_CRITICAL(int*){}
struct Preferences {
 std::map<std::string,float> f;std::map<std::string,bool>b;std::map<std::string,String>s;
 int writeAt=0,failAt=0,reads=0,nanReadAt=0;
 bool fail(){return ++writeAt==failAt;}
 float getFloat(const char*k,float fallback){if(++reads==nanReadAt)return NAN;return f.count(k)?f[k]:fallback;}
 bool getBool(const char*k,bool fallback){return b.count(k)?b[k]:fallback;}
 String getString(const char*k,const char*fallback){return s.count(k)?s[k]:String(fallback);}
 size_t putFloat(const char*k,float v){if(fail())return 0;f[k]=v;return sizeof(float);}
 size_t putBool(const char*k,bool v){if(fail())return 0;b[k]=v;return 1;}
 size_t putString(const char*k,const String&v){if(fail())return 0;s[k]=v;return v.length();}
 bool remove(const char*k){if(fail())return false;f.erase(k);b.erase(k);s.erase(k);return true;}
}prefs;
namespace BusaSound {struct Channel{};}
float buzzerOnC[2],buzzerOffC[2],soundOnC[4],soundOffC[4],refTemp[4],noticeTemp[4];
bool buzzerEnabled[2],soundEnabled[4],tftDirty=false;BusaSound::Channel buzzerTempChannels[2];uint32_t lastRender=9;
int outputsOff=0,resets=0;void soundOutputsOff(){++outputsOff;}void soundResetState(){++resets;}
String quoteJson(const String&s){return s;}
#include "BUSA_RemoteAlarm.h"
int main(){
 const char* onKeys[4]={"buzOn0","buzOn1","sndT3On","sndT4On"};
 const char* offKeys[4]={"buzOff0","buzOff1","sndT3Off","sndT4Off"};
 const char* enableKeys[6]={"buzE0","buzE1","sndRoleEnable0","sndRoleEnable1","sndRoleEnable2","sndRoleEnable3"};
 for(int failure=0;failure<=16;failure++){
  prefs=Preferences();for(int i=0;i<4;i++){prefs.f[onKeys[i]]=30;prefs.f[offKeys[i]]=29;refTemp[i]=30;noticeTemp[i]=29;}
  for(auto key:enableKeys)prefs.b[key]=false;
  prefs.s["remoteAlarmId"]="aaaaaaaaaaaaaaaaaaaaaaaa";
  auto floats=prefs.f;auto flags=prefs.b;auto strings=prefs.s;
  prefs.failAt=failure==16?0:failure;prefs.nanReadAt=failure==16?9:0;
  busaRemoteInboxReady=false;busaRemoteAckState=0;outputsOff=resets=0;
  BusaRemoteAlarmCommand cmd;strcpy(cmd.id,"bbbbbbbbbbbbbbbbbbbbbbbb");
  for(int i=0;i<4;i++){cmd.on[i]=80+i;cmd.off[i]=78+i;}
  busaRemoteEnqueue(cmd);busaRemoteAlarmApply();
  if(failure){assert(busaRemoteAckState==2&&prefs.f==floats&&prefs.b==flags&&prefs.s==strings&&refTemp[0]==30&&outputsOff==0);}
  else {assert(busaRemoteAckState==1&&refTemp[0]==80&&outputsOff==1&&resets==1);for(auto key:enableKeys)assert(prefs.b[key]);}
 }
}
''','remote_nvs')
print('PASS audit: all JSON control bytes/UTF-8 roundtrip; real restart predicates including rollover; DS18B20 hot replacement/missing/invalid conversion; 50k concurrent immutable snapshots without leaks; sound hysteresis/T4 priority/return to T3/exclusive 30s expiry/rollover; NVS failures at 15 writes and NaN readback roll back enable flags/thresholds/command ID')
