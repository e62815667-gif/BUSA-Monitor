"""Execute production scanner and reconnect supervisor with radio/event mocks. No ESP32."""
from pathlib import Path
import subprocess,tempfile,re,json,hashlib
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
def function(src,sig):
 start=src.index(sig);p=src.index('{',start)+1;depth=1
 while depth:
  if src[p]=='{':depth+=1
  elif src[p]=='}':depth-=1
  p+=1
 return src[start:p]
keypad=(sketch/'BUSA_KeypadTFT_UI.h').read_text()
prefix=r'''
#include <string>
#include <vector>
#include <atomic>
#include <cstdint>
#include <cassert>
#include <cstdio>
class String:public std::string{public:using std::string::string;using std::string::operator=;
 String(const std::string& s):std::string(s){}String(int n):std::string(std::to_string(n)){}String(unsigned n):std::string(std::to_string(n)){}bool isEmpty()const{return empty();}};
uint32_t clockMs=100;uint32_t millis(){return clockMs;}bool elapsed(uint32_t n,uint32_t t,uint32_t d){return uint32_t(n-t)>=d;}
const int ESP_OK=0,WIFI_AP_STA=3,WL_CONNECTED=3,WIFI_SCAN_TYPE_ACTIVE=0,WIFI_SCAN_TYPE_PASSIVE=1;
const int ARDUINO_EVENT_WIFI_SCAN_DONE=42;
struct ScanEvent{uint32_t status;uint8_t number;};struct arduino_event_t{int event_id;struct {ScanEvent wifi_scan_done;}event_info;};
using EventFn=void(*)(arduino_event_t*);
struct wifi_scan_config_t{uint8_t *ssid,*bssid,channel;bool show_hidden;int scan_type;struct{struct{uint32_t min,max;}active;uint32_t passive;}scan_time;uint8_t home_chan_dwell_time;uint32_t reserved;};
using esp_err_t=int;int startRc=0,startCalls=0,stopCalls=0;wifi_scan_config_t lastConfig;
int esp_wifi_scan_start(wifi_scan_config_t* c,bool block){assert(!block);lastConfig=*c;++startCalls;return startRc;}
int esp_wifi_scan_stop(){++stopCalls;return ESP_OK;}
struct Radio{int modeValue=3,connected=3,disconnects=0,begins=0,modeChanges=0,scanCount=0;EventFn observer=nullptr;std::vector<String> records;String associated="saved";std::vector<String> targets;
 int onEvent(EventFn fn,int id){assert(id==42);observer=fn;return 1;}
 int status(){return connected;}int getMode(){return modeValue;}bool mode(int n){modeValue=n;++modeChanges;return true;}
 void setAutoReconnect(bool){}void setSleep(bool){}
 void disconnect(bool radioOff,bool erase){assert(!radioOff&&!erase);++disconnects;connected=0;}
 void scanDelete(){records.clear();scanCount=0;}int scanComplete(){return scanCount;}
 String SSID(int n){return records.at(n);}String SSID(){return associated;}int RSSI(int){return -40;}
 void begin(const char* ssid,const char*){++begins;targets.push_back(ssid);associated=ssid;}
}WiFi;
enum {WIFIUI_STATUS,WIFIUI_SCANNING,WIFIUI_NETWORKS,WIFIUI_PASSWORD,WIFIUI_CONNECTING};int wifiUiStage=WIFIUI_STATUS;
bool otaActive=false,busaWifiScanRunning=false,wifiUiConnectPending=false,wifiUiDisablePending=false,wifiUiStaDisabled=false,wifiUiResumeStation=false,wifiUiOwnsScan=false,tftDirty=false;
uint32_t busaWifiScanStartedAt=0,busaWifiScanLastAt=0,wifiUiAssociationDueAt=0,wifiUiConnectAt=0,wifiUiScanAt=0,lastStationCheck=0;
int currentPage=5,approvedSettingsItem=1,tftView=1,wifiUiCount=0,wifiUiMenuIndex=0,wifiUiIndex=0,wifiUiScanAttempts=0,wifiUiLastScanCode=-99,googleInternet=0;
String wifiUiNetworks[48],wifiUiFeedback,stationSsid="saved",stationPass="password",wifiUiLastConnectError;int wifiUiSignals[48];
uint32_t wifiUiRetryLastAt=0,wifiUiRetryInterval=5000;bool wifiUiRetryArmed=false;
String quoteJson(const String& s){return "\""+s+"\"";}
#include "BUSA_WifiScan.h"
'''
code=prefix+'\n'+function(keypad,'void wifiUiResetReconnectBackoff()')+'\n'+function(keypad,'void wifiUiApplyStationDisconnect()')+'\n'+function(keypad,'void wifiUiPoll()')+r'''
void done(int status,std::vector<String> names,int advertised=-1){WiFi.records=names;WiFi.scanCount=names.size();arduino_event_t ev={};ev.event_id=42;ev.event_info.wifi_scan_done.status=status;ev.event_info.wifi_scan_done.number=advertised<0?names.size():advertised;WiFi.observer(&ev);}
void poll(uint32_t t){clockMs=t;busaWifiScanPoll(t);}
void start(uint32_t t){clockMs=t;assert(busaWifiRequestSharedScan(false,true));poll(t+1000);}
int main(){
 busaWifiInitScanner();assert(busaScanObserverReady);
 start(100);assert(startCalls==1&&WiFi.disconnects==0&&WiFi.modeChanges==0);
 assert(lastConfig.ssid==nullptr&&lastConfig.bssid==nullptr&&lastConfig.channel==0&&lastConfig.show_hidden&&lastConfig.reserved==0);
 assert(lastConfig.scan_time.active.min==120&&lastConfig.scan_time.active.max==300&&lastConfig.home_chan_dwell_time==60);
 done(0,{"Тестовая сеть","Тестовая сеть","router"});poll(2000);assert(wifiUiCount==2&&wifiUiNetworks[0]=="Тестовая сеть");assert(wifiUiLastScanCode==2&&!busaWifiScanRunning);
 // Empty retry NEVER disconnects an already connected STA.
 start(3000);done(0,{});poll(5000);assert(busaSharedScanState==BUSA_SCAN_REQUESTED);poll(5900);assert(lastConfig.scan_type==WIFI_SCAN_TYPE_PASSIVE&&lastConfig.scan_time.passive==450);
 done(0,{});poll(7000);assert(busaSharedScanState==BUSA_SCAN_EMPTY&&WiFi.disconnects==0&&WiFi.begins==0&&wifiUiLastScanCode==0);
 // Native error code survives, not mapped to an empty list.
 startRc=0x3006;start(8000);poll(10900);assert(busaSharedScanState==BUSA_SCAN_ERROR&&busaSharedScanError=="scan_start_failed");assert(busaSharedScanStartResult==0x3006);startRc=0;
 // Nonzero completion status / lost collector records are errors, not no_networks.
 start(12000);done(1,{});poll(14000);poll(14900);done(1,{});poll(15000);assert(busaSharedScanState==BUSA_SCAN_ERROR&&busaSharedScanError=="scan_driver_failed");
 start(16000);done(0,{},2);poll(18000);poll(18900);done(0,{},2);poll(19000);assert(busaSharedScanError=="scan_results_unavailable");
 start(20000);poll(31000);assert(busaSharedScanState==BUSA_SCAN_STOPPING&&stopCalls==1);poll(32000);assert(!busaWifiScanRunning&&busaSharedScanError=="scan_timeout");
 done(0,{"late"});assert(!busaScanEventReady); // late completion ignored while disarmed
 // Explicit STA-off still permits discovery, never clears the saved off intent.
 wifiUiStaDisabled=true;WiFi.connected=0;start(33000);done(0,{"Тестовая сеть"});poll(35000);assert(wifiUiCount==1&&wifiUiStaDisabled);
 int calls=WiFi.begins;wifiUiPoll();clockMs+=60000;wifiUiPoll();assert(WiFi.begins==calls&&!wifiUiConnectPending);
 // Reconnect only CURRENT target after scan/loss, with delay and bounded retry.
 wifiUiStaDisabled=false;wifiUiStage=WIFIUI_STATUS;stationSsid="current";wifiUiResetReconnectBackoff();wifiUiPoll();clockMs+=4999;wifiUiPoll();assert(!wifiUiConnectPending);++clockMs;wifiUiPoll();assert(wifiUiConnectPending);
 ++clockMs;wifiUiPoll();clockMs+=550;wifiUiPoll();assert(WiFi.begins==calls+1&&WiFi.targets.back()=="current");
 // Switching targets cannot resurrect the old iPhone.
 stationSsid="new iPhone";wifiUiResetReconnectBackoff();wifiUiConnectPending=true;wifiUiAssociationDueAt=0;wifiUiConnectAt=0;++clockMs;wifiUiPoll();clockMs+=550;wifiUiPoll();assert(WiFi.targets.back()=="new iPhone");
 WiFi.connected=3;wifiUiPoll();assert(wifiUiConnectAt==0&&wifiUiRetryInterval==5000);
 wifiUiStaDisabled=true;wifiUiDisablePending=true;wifiUiPoll();calls=WiFi.begins;clockMs+=100000;wifiUiPoll();assert(WiFi.begins==calls);
 // Rollover and OTA gating.
 WiFi.connected=0;wifiUiStaDisabled=true;start(UINT32_MAX-400);assert(busaSharedScanState==BUSA_SCAN_RUNNING);done(0,{"rollover"});poll(1800);assert(wifiUiCount==1);
 otaActive=true;assert(!busaWifiRequestSharedScan(false,true));otaActive=false;start(3000);otaActive=true;poll(4100);assert(busaSharedScanState==BUSA_SCAN_STOPPING);done(1,{});poll(4200);assert(!busaWifiScanRunning&&busaSharedScanError=="scan_cancelled_ota");
 puts("PASS production scan/supervisor: deterministic IDF config, active/passive, Cyrillic SSID/dedup, no connected-STA drop, native errors, timeout/OTA, manual-off, reconnect/current target and rollover");
}
'''
with tempfile.TemporaryDirectory() as d:
 (Path(d)/'esp_wifi.h').write_text('')
 p=Path(d)/'wifi.cpp';p.write_text(code);exe=Path(d)/'wifi'
 subprocess.run(['g++','-std=c++11','-I'+str(d),'-I'+str(sketch),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)

for sig,digest in json.loads((root/'tests/fixtures/wifi1_keypad_preserved.json').read_text()).items():
 text=function(keypad,sig)
 if sig.startswith('void wifiUiSaveConnection('):text=text.replace('  wifiUiResetReconnectBackoff();\n','')
 assert hashlib.sha256(text.encode()).hexdigest()==digest,sig
print('PASS keypad editor/mapping/manual-off/NVS commit paths preserved; Save only resets retry timer')
