from pathlib import Path
import subprocess, tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
source=(root/'BUSA_WifiRecovery.h').read_text()
assert 'esp_wifi_scan_start(&config,true)' in source
assert 'WiFi.scanNetworks(' not in source and 'WiFi.scanComplete(' not in source
assert 'WiFi.disconnect(false,false)' in source
stub=r'''
#include <string>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <new>
#include <algorithm>
class String : public std::string {
public:
 using std::string::string;
 String(const std::string&s):std::string(s){}
 String(int x):std::string(std::to_string(x)){}
 String(unsigned x):std::string(std::to_string(x)){}
 bool isEmpty()const{return empty();}
};
using WiFiEvent_t=int;
struct WiFiEventInfo_t{struct{uint8_t reason;}wifi_sta_disconnected;};
constexpr int ARDUINO_EVENT_WIFI_STA_DISCONNECTED=5;
constexpr int WL_CONNECTED=3;
constexpr int WIFI_SCAN_TYPE_ACTIVE=1;
using esp_err_t=int;
constexpr int ESP_OK=0;
struct wifi_scan_config_t{
 bool show_hidden=false;int scan_type=0;
 struct {struct{int min=0,max=0;}active;}scan_time;
};
struct wifi_ap_record_t{uint8_t ssid[33]={0};int8_t rssi=0;};
unsigned tick=0;unsigned millis(){return tick;}
bool elapsed(uint32_t now,uint32_t then,uint32_t delta){return uint32_t(now-then)>=delta;}
String quoteJson(const String& s){return String("\"")+s+"\"";}
struct FakeWifi {
 int state=0,connects=0,disconnects=0,staEnable=0,reconnects=0;bool autoReconnect=false;
 int status(){return state;}
 void disconnect(bool off,bool erase){assert(!off&&!erase);disconnects++;state=0;}
 void begin(const char*,const char*){connects++;}
 bool enableSTA(bool value){assert(value);staEnable++;return true;}
 bool setAutoReconnect(bool value){autoReconnect=value;return true;}
 bool reconnect(){reconnects++;return true;}
} WiFi;
bool otaActive=false,otaRestart=false;
uint32_t lastStationCheck=0;
String stationSsid="iPhone 16",stationPass="testpass",nearbyNames[5];
uint8_t nearbyCount=0;
int fakeScanCode=0,fakeRecordCode=0,starts=0,clears=0;
unsigned detected=2;
esp_err_t esp_wifi_scan_start(const wifi_scan_config_t*c,bool block){
 assert(block);assert(c->scan_type==WIFI_SCAN_TYPE_ACTIVE);
 assert(c->scan_time.active.min==100&&c->scan_time.active.max==160);
 starts++;return fakeScanCode;
}
esp_err_t esp_wifi_scan_get_ap_num(uint16_t*count){*count=(uint16_t)detected;return 0;}
esp_err_t esp_wifi_scan_get_ap_records(uint16_t*count,wifi_ap_record_t*out){
 if(fakeRecordCode)return fakeRecordCode;
 for(unsigned i=0;i<*count;i++){
  const char*name=i==0?"iPhone 16":"Home 2G";
  std::strcpy(reinterpret_cast<char*>(out[i].ssid),name);
  out[i].rssi=(int8_t)(-43-(int)i);
 }
 return 0;
}
esp_err_t esp_wifi_clear_ap_list(){clears++;return 0;}
'''
tests=r'''
void check(uint32_t t){tick=t;wifiRecoveryTick(t);}
int main(){
 // Missing credentials must not prevent discovery and must not fake a connect.
 stationSsid="";stationPass="";
 wifiRequestScan(1000);
 assert(!WiFi.autoReconnect);
 check(1649);assert(starts==0);
 check(1650);assert(starts==1);
 assert(!wifiScanBusy() && wifiScanSuccesses==1 && !wifiConnectPending);
 assert(wifiScanJSON().find("iPhone 16")!=std::string::npos);
 assert(wifiScanJSON().find("Home 2G")!=std::string::npos);
 assert(WiFi.connects==0);
 assert(wifiRecoveryJSON().find("scan_start_error")!=std::string::npos);
 // No APs is successful discovery, not a false driver failure.
 detected=0;wifiRequestScan(2000);check(2650);
 assert(!wifiScanBusy()&&wifiScanSuccesses==2);
 assert(wifiScanJSON().find("\"networks\":[]")!=std::string::npos);
 // Transient error reports exact IDF code, retries with a bounded pause.
 detected=1;fakeScanCode=0x3006;
 wifiRequestScan(3000);check(3650);
 assert(wifiScanBusy()&&wifiScanTries==1&&wifiLastScanErr==0x3006);
 check(4299);assert(starts==3);
 fakeScanCode=0;check(4300);
 assert(!wifiScanBusy()&&wifiScanTries==2&&wifiScanSuccesses==3);
 // Complete repeated failure is visible to the browser and diagnosable.
 fakeScanCode=0x3006;wifiRequestScan(5000);
 check(5650);check(6300);check(6950);
 assert(!wifiScanBusy()&&wifiScanTries==3);
 assert(wifiScanJSON().find("scan_driver_start_failed")!=std::string::npos);
 assert(wifiRecoveryJSON().find("12294")!=std::string::npos);
 // Results error gets its own code and never leaves stale list.
 fakeScanCode=0;fakeRecordCode=0x1002;
 wifiRequestScan(8000);check(8650);
 check(9300);check(9950);
 assert(!wifiScanBusy() && wifiScanJSON().find("scan_records_read_failed")!=std::string::npos);
 fakeRecordCode=0;
 // Previously saved credentials reconnect after a successful scan.
 stationSsid="iPhone 16";stationPass="secret123";
 wifiRequestScan(12000);check(12650);
 assert(wifiConnectPending);
 check(13149);assert(WiFi.connects==0);
 check(13150);assert(WiFi.connects==1);
 // Disconnected event reason and OTA pause remain functional.
 wifiRecoveryEvent(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{{201}});
 assert(wifiDisconnectReason==201&&wifiDisconnectCount==1);
 wifiScheduleConnect(14000);otaActive=true;
 check(14500);assert(WiFi.connects==1);
 otaActive=false;check(14500);assert(WiFi.connects==2);
 // V14.9 reconnect policy: retry saved station without a fresh disconnect.
 wifiConnectPending=false;lastStationCheck=1000;
 check(20999);assert(WiFi.reconnects==0);
 check(21000);assert(WiFi.reconnects==1);
 // Clock wrap around must not block reconnection forever.
 wifiScheduleConnect(UINT32_MAX-100);
 check(398);assert(WiFi.connects==2);
 check(399);assert(WiFi.connects==3);
 uint8_t hdr[36]={};hdr[0]=0xe9;hdr[1]=3;hdr[12]=9;
 hdr[32]=0x32;hdr[33]=0x54;hdr[34]=0xcd;hdr[35]=0xab;
 assert(busaAppHeaderValid(hdr,36)&&!busaAppHeaderValid(hdr,35));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp'
 p.write_text(stub+'\n#include "BUSA_WifiRecovery.h"\n#include "BUSA_AppHeader.h"\n'+tests)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(root),str(p),'-o',d+'/test'],check=True)
 subprocess.run([d+'/test'],check=True)
print('PASS V14.19: V14.9 STA reconnect and V14.18 IDF scan, empty credentials, retry, OTA pause, APP header')
