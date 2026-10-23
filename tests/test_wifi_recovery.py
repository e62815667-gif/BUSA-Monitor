from pathlib import Path
import subprocess,tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
stub=r'''
#include <string>
#include <cstdint>
#include <cstring>
#include <cassert>
class String:public std::string{
public:using std::string::string;String(const std::string&s):std::string(s){}
String(int v):std::string(std::to_string(v)){}
String(unsigned v):std::string(std::to_string(v)){}
bool isEmpty()const{return empty();}
};
const int WL_CONNECTED=3,WIFI_SCAN_RUNNING=-1,WIFI_SCAN_FAILED=-2;
const int ARDUINO_EVENT_WIFI_STA_DISCONNECTED=5;
using WiFiEvent_t=int;struct WiFiEventInfo_t{struct{uint8_t reason;}wifi_sta_disconnected;};
bool otaActive=false,otaRestart=false;uint32_t lastStationCheck=0,nowMs=0;
uint32_t millis(){return nowMs;}
bool elapsed(uint32_t a,uint32_t b,uint32_t d){return uint32_t(a-b)>=d;}
String quoteJson(const String&s){return "\""+s+"\"";}
String stationSsid="iPhone 16",stationPass="testpass",nearbyNames[5];uint8_t nearbyCount=0;
struct MockWifi{
int statusCode=0,startRc=WIFI_SCAN_RUNNING,complete=WIFI_SCAN_RUNNING;
int scans=0,deletes=0,disconnects=0,reconnects=0,begins=0,enabledSta=0;
bool autoreconnect=true;
int status(){return statusCode;}
void setAutoReconnect(bool v){autoreconnect=v;}
void disconnect(bool off,bool erase){assert(!off&&!erase);++disconnects;statusCode=0;}
void scanDelete(){++deletes;}
void enableSTA(bool yes){assert(yes);++enabledSta;}
int scanNetworks(bool async,bool hidden,bool passive,int time){
 assert(async&&!hidden&&!passive&&time==160);++scans;return startRc;
}
int scanComplete(){return complete;}
String SSID(int i){return i==0?"":(i==1?"iPhone 16":"iPhone 12");}
int RSSI(int){return -57;}
void begin(const char*,const char*){++begins;}
void reconnect(){++reconnects;}
}WiFi;
using esp_err_t=int;
const esp_err_t ESP_OK=0;
const int WIFI_SCAN_TYPE_ACTIVE=1;
struct wifi_scan_config_t {
bool show_hidden=false;int scan_type=0;
struct{struct{uint32_t min=0,max=0;}active;}scan_time;
};
struct wifi_ap_record_t{unsigned char ssid[33]={};int8_t rssi=-55;};
int idfStarts=0,idfStartRc=ESP_OK,idfCountRc=ESP_OK,idfGetRc=ESP_OK;
uint16_t idfApCount=2;int listClears=0;
esp_err_t esp_wifi_scan_start(const wifi_scan_config_t* cfg,bool block){
 assert(block && cfg->scan_type==WIFI_SCAN_TYPE_ACTIVE);
 ++idfStarts;return idfStartRc;
}
esp_err_t esp_wifi_scan_get_ap_num(uint16_t*n){
 if(idfCountRc!=ESP_OK)return idfCountRc;*n=idfApCount;return ESP_OK;
}
esp_err_t esp_wifi_scan_get_ap_records(uint16_t*used,wifi_ap_record_t*data){
 if(idfGetRc!=ESP_OK)return idfGetRc;
 uint16_t amount=*used; if(amount>idfApCount)amount=idfApCount;*used=amount;
 for(int i=0;i<amount;i++){
  const char*t=i==0?"":(i==1?"iPhone 16":"iPhone 12");
  strncpy((char*)data[i].ssid,t,32);
 }
 return ESP_OK;
}
esp_err_t esp_wifi_clear_ap_list(){++listClears;return ESP_OK;}
'''
tests=r'''
int main(){
 // Phase 1: successful Arduino scan after leaving STA reconnect.
 nowMs=1000;wifiRequestScan(nowMs);
 assert(wifiScanBusy()&&!WiFi.autoreconnect&&WiFi.disconnects==1);
 wifiRequestScan(nowMs);assert(WiFi.scans==0);
 wifiRecoveryTick(2199);assert(WiFi.scans==0);
 nowMs=2200;wifiRecoveryTick(nowMs);assert(WiFi.scans==1);
 WiFi.complete=3;nowMs=2250;wifiRecoveryTick(nowMs);
 assert(!wifiScanBusy()&&wifiScanSuccesses==1);
 assert(wifiScanJSON().find("iPhone 16")!=std::string::npos);
 assert(wifiScanJSON().find("[,")==std::string::npos);
 assert(wifiLastVisibleCount==2&&WiFi.autoreconnect);
 assert(wifiConnectPending);
 nowMs=3750;wifiRecoveryTick(nowMs);assert(WiFi.begins==1);
 // Phase 2: native -2 scan-start failures do not falsely show 0 networks.
 WiFi.startRc=WIFI_SCAN_FAILED;WiFi.complete=WIFI_SCAN_RUNNING;
 nowMs=10000;wifiRequestScan(nowMs);assert(!WiFi.autoreconnect);
 for(int i=1;i<=3;i++){
  nowMs=10000+i*1200;wifiRecoveryTick(nowMs);
 }
 assert(wifiScanUsedDriver&&wifiScanTries==3);
 assert(wifiLastScanErr==-2&&idfStarts==0);
 nowMs=14800;wifiRecoveryTick(nowMs);
 assert(idfStarts==1&&!wifiScanBusy());
 assert(wifiScanJSON().find("iPhone 16")!=std::string::npos);
 assert(wifiScanJSON().find("[,")==std::string::npos);
 assert(wifiScanSuccesses==2&&wifiScanFailures>=3);
 // Phase 3: driver-level errors are surfaced truthfully.
 idfStartRc=0x105;
 nowMs=20000;wifiRequestScan(nowMs);
 for(int i=1;i<=3;i++){nowMs=20000+i*1200;wifiRecoveryTick(nowMs);}
 nowMs=24800;wifiRecoveryTick(nowMs);
 assert(!wifiScanBusy());
 assert(wifiScanJSON().find("scan_driver_start_failed")!=std::string::npos);
 assert(wifiLastScanErr==0x105);
 // Phase 4: no scanning or reconnect while OTA is active.
 nowMs=30000;otaActive=true;
 wifiRequestScan(nowMs);assert(!wifiScanBusy());
 int starts=WiFi.scans;wifiRecoveryTick(50000);
 assert(WiFi.scans==starts);
 otaActive=false;
 // Clock wrapping and station retry.
 nowMs=UINT32_MAX-200;wifiScheduleConnect(nowMs);
 nowMs=1400;wifiRecoveryTick(nowMs);
 assert(WiFi.begins==2);
 wifiRecoveryEvent(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{{201}});
 assert(wifiDisconnectReason==201&&wifiDisconnectCount==1);
 uint8_t h[36]={};h[0]=0xe9;h[1]=3;h[12]=9;
 h[32]=0x32;h[33]=0x54;h[34]=0xcd;h[35]=0xab;
 assert(busaAppHeaderValid(h,36));assert(!busaAppHeaderValid(h,35));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp'
 p.write_text(stub+'\n#include "BUSA_WifiRecovery.h"\n#include "BUSA_AppHeader.h"\n'+tests)
 exe=Path(d)/'test'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(root),str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('PASS: iPhone 16 scan/reconnect exclusion; -2 retries; IDF fallback; OTA isolation; reset and board header')
