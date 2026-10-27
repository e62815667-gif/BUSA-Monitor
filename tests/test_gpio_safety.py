"""Execute the actual locked alarm driver and HTTP handler; GPIO writes are forbidden."""
from pathlib import Path
import subprocess, tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
runtime=(root/'BUSA_V15_Runtime.h').read_text()
marker='server.on("/api/alarm/config",HTTP_POST,[]{'
handler=runtime.split(marker,1)[1].split('\n  });',1)[0]
stub=r'''
#include <string>
#include <map>
#include <vector>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
using std::isfinite;
class String:public std::string{public:using std::string::string;
 String(const std::string&s):std::string(s){}String(int n):std::string(std::to_string(n)){}
 String(float n,int){char b[32];snprintf(b,sizeof b,"%.1f",n);assign(b);}};
constexpr int INPUT=1; // Deliberately no OUTPUT, HIGH, LOW or digitalWrite API.
std::vector<int> released;
void pinMode(int pin,int mode){assert(mode==INPUT);assert(pin==7||pin==6||pin==5||pin==4);released.push_back(pin);}
#include "BUSA_V15_Board.h"
float refTemp[4]={50,50,50,50},noticeTemp[4]={45,45,45,45};
bool hasTemp(int,uint32_t){return true;}
uint32_t millis(){return 100;}
bool local=true,otaActive=false,otaRestart=false;
bool apClient(){return local;}
struct Prefs{int reads=0,writes=0;bool fail=false;bool getBool(const char*,bool){++reads;return true;}
 bool putBool(const char*,bool value){assert(!value);++writes;return !fail;}}prefs;
struct Server{std::map<std::string,String> args;int code=0;
 String arg(const String& k){return args[k];}void send(int c,const char*,const String&){code=c;}}server;
#include "BUSA_V15_Alarm.h"
'''
tests=r'''
int main(){
 static_assert(!RELAY_PHYSICAL_OUTPUTS&&!LED_PHYSICAL_OUTPUTS,"outputs locked");
 alarmInitPins();assert((released==std::vector<int>{7,6,5,4}));assert(!alarmGpioInitialized);
 for(int i=0;i<4;i++){alarmEnabled[i]=true;alarmChannels[i].on=true;}
 alarmLoadSettings();assert(prefs.reads==0);
 for(int flags=0;flags<4;flags++){
  otaActive=flags&1;otaRestart=flags&2;
  for(int i=0;i<4;i++){alarmEnabled[i]=true;alarmChannels[i].on=true;}
  alarmPoll(100);for(int i=0;i<4;i++){assert(!alarmEnabled[i]&&!alarmChannels[i].on);assert(alarmChannels[i].reason==BusaAlarm::HardwareLocked);}
 }
 assert(released.size()==4);otaActive=otaRestart=false;
 server.args["confirm"]="SOUND_ALARMS_ONLY";
 for(int i=0;i<4;i++)server.args[std::string("enable")+std::to_string(i)]="0";
 for(int i=0;i<4;i++){
  server.args[std::string("enable")+std::to_string(i)]="1";int before=prefs.writes;
  configRoute();assert(server.code==423&&prefs.writes==before);
  server.args[std::string("enable")+std::to_string(i)]="0";
 }
 configRoute();assert(server.code==200&&prefs.writes==4);
 prefs.fail=true;configRoute();assert(server.code==500);prefs.fail=false;
 local=false;configRoute();assert(server.code==403);local=true;
 otaActive=true;configRoute();assert(server.code==403);otaActive=false;
 auto json=alarmStatusJSON(100);assert(json.find("\"driver_enabled\":false")!=std::string::npos);
 assert(json.find("HARDWARE_LOCKED")!=std::string::npos);
 assert(released.size()==4);assert(prefs.reads==0);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'gpio.cpp';p.write_text(stub+'\nvoid configRoute(){'+handler+'\n}\n'+tests)
 exe=Path(d)/'gpio';subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(root),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
setup=runtime.split('void setup(){',1)[1]
assert setup.lstrip().startswith('alarmInitPins();')
assert 'server.on("/expert",HTTP_GET' in runtime
print('PASS physical GPIO lock: INPUT/no pull only, no digitalWrite API, old NVS ignored, OTA/reset states, all four HTTP enable requests rejected, NVS failure, AP restriction')
