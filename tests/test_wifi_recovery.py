from pathlib import Path
import subprocess, tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
stub=r'''
#include <string>
#include <cassert>
#include <cstdint>
#include <cstring>
class String: public std::string {
public:
 using std::string::string;
 String(const std::string &s):std::string(s){}
 String(int v):std::string(std::to_string(v)){}
 String(unsigned v):std::string(std::to_string(v)){}
 bool isEmpty()const{return empty();}
};
const int WL_CONNECTED=3,WIFI_SCAN_RUNNING=-1,WIFI_SCAN_FAILED=-2;
const int ARDUINO_EVENT_WIFI_STA_DISCONNECTED=5;
using WiFiEvent_t=int;struct WiFiEventInfo_t{struct{uint8_t reason;}wifi_sta_disconnected;};
bool otaActive=false,otaRestart=false;uint32_t lastStationCheck=0,currentTime=0;
uint32_t millis(){return currentTime;}
String stationSsid="iPhone 16",stationPass="testpass",nearbyNames[5];uint8_t nearbyCount=0;
bool elapsed(uint32_t a,uint32_t b,uint32_t d){return uint32_t(a-b)>=d;}
String quoteJson(const String&s){return "\""+s+"\"";}
struct MockWiFi {
 int state=0,startResult=-1,complete=-1,starts=0,begins=0,disconnects=0,deletes=0;
 bool autoReconnect=true;
 void setAutoReconnect(bool v){autoReconnect=v;}
 int status(){return state;}
 void disconnect(bool off,bool erase){assert(!off&&!erase);++disconnects;state=0;}
 void scanDelete(){++deletes;}
 void enableSTA(bool v){assert(v);}
 int scanNetworks(bool a,bool h,bool p,int ms){assert(a&&!h&&!p&&ms==160);++starts;return startResult;}
 int scanComplete(){return complete;}
 String SSID(int i){return i==0?"":i==1?"iPhone 12":"Other";}
 int RSSI(int){return -51;}
 void begin(const char*,const char*){++begins;}
} WiFi;
using esp_err_t=int;
const esp_err_t ESP_OK=0;
const int WIFI_SCAN_TYPE_ACTIVE=1;
struct wifi_scan_config_t {
 bool show_hidden=false;
 int scan_type=0;
 struct { struct {uint32_t min=0,max=0;} active; } scan_time;
};
struct wifi_ap_record_t {
 unsigned char ssid[33]={};
 int8_t rssi=-53;
};
int idfStartRc=0,idfCountRc=0,idfRecordRc=0,idfCount=2,idfCalls=0,clears=0;
esp_err_t esp_wifi_scan_start(const wifi_scan_config_t* cfg,bool block){
 assert(block && cfg->scan_type==WIFI_SCAN_TYPE_ACTIVE);++idfCalls;return idfStartRc;
}
esp_err_t esp_wifi_scan_get_ap_num(uint16_t* n){
 if(idfCountRc!=ESP_OK)return idfCountRc;
 *n=(uint16_t)idfCount;return ESP_OK;
}
esp_err_t esp_wifi_scan_get_ap_records(uint16_t* count,wifi_ap_record_t* out){
 if(idfRecordRc!=ESP_OK)return idfRecordRc;
 int size=*count;*count=(uint16_t)(size<idfCount?size:idfCount);
 for(int i=0;i<*count;i++){
  const char* txt=(i==0)?"":(i==1)?"iPhone 12":"Other";
  std::strncpy((char*)out[i].ssid,txt,32);
  out[i].rssi=-50;
 }
 return ESP_OK;
}
esp_err_t esp_wifi_clear_ap_list(){++clears;return ESP_OK;}
'''
tests=r'''
int main(){
 wifiScheduleConnect(0);assert(!WiFi.autoReconnect);
 wifiRecoveryTick(499);assert(WiFi.begins==0);
 wifiRecoveryTick(500);assert(WiFi.begins==1&&WiFi.autoReconnect);
 WiFi.state=WL_CONNECTED;
 wifiRequestScan(20000);assert(!WiFi.autoReconnect && wifiScanBusy());
 wifiRequestScan(20001); // Repeated button taps must not restart scanning.
 wifiRecoveryTick(20649);assert(WiFi.starts==0);
 wifiRecoveryTick(20650);assert(WiFi.starts==1);
 WiFi.complete=2;wifiRecoveryTick(20700);
 assert(!wifiScanBusy()&&WiFi.autoReconnect);
 assert(wifiScanJSON().find("iPhone 12")!=std::string::npos);
 assert(wifiScanJSON().find("[,")==std::string::npos); // Blank SSIDs cannot break JSON.
 assert(wifiLastVisibleCount==1 && wifiScanSuccesses==1);
 assert(!wifiLastUsedFallback);
 // Failure in primary Arduino scan recovers using IDF blocking enumeration.
 WiFi.startResult=WIFI_SCAN_FAILED;
 wifiRequestScan(30000);wifiRecoveryTick(30650);
 assert(wifiLastUsedFallback&&idfCalls==0);
 currentTime=31300;wifiRecoveryTick(31300);
 assert(idfCalls==1&&!wifiScanBusy());
 assert(wifiScanJSON().find("iPhone 12")!=std::string::npos);
 assert(wifiScanJSON().find("[,")==std::string::npos);
 assert(wifiScanSuccesses==2&&wifiScanFailures==1);
 // Empty Arduino discovery also falls back before declaring zero networks.
 WiFi.startResult=WIFI_SCAN_RUNNING;WiFi.complete=0;idfCount=2;
 wifiRequestScan(40000);wifiRecoveryTick(40650);wifiRecoveryTick(40700);
 assert(wifiLastUsedFallback);
 currentTime=41350;wifiRecoveryTick(41350);
 assert(idfCalls==2&&wifiScanSuccesses==3);
 // Both engines may fail: report an explicit error, never invent SSIDs.
 WiFi.startResult=WIFI_SCAN_FAILED;idfStartRc=0x105;
 wifiRequestScan(50000);wifiRecoveryTick(50650);
 currentTime=51300;wifiRecoveryTick(51300);
 assert(!wifiScanBusy()&&wifiScanJSON().find("driver_scan_start_failed")!=std::string::npos);
 assert(wifiScanFailures>=3);
 // The local AP scan pauses when OTA is active.
 wifiRequestScan(60000);otaActive=true;int before=WiFi.starts;
 wifiRecoveryTick(60650);assert(WiFi.starts==before);
 otaActive=false;wifiCancelScan();assert(!wifiScanBusy());
 assert(WiFi.autoReconnect);
 // Automatic reconnect, clock wrap and disconnect evidence.
 WiFi.state=0;wifiScheduleConnect(UINT32_MAX-100);assert(!WiFi.autoReconnect);
 wifiRecoveryTick(400);assert(WiFi.autoReconnect);
 wifiRecoveryEvent(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{{201}});
 assert(wifiDisconnectReason==201&&wifiDisconnectCount==1);
 uint8_t hdr[36]={};hdr[0]=0xe9;hdr[1]=3;hdr[12]=9;
 hdr[32]=0x32;hdr[33]=0x54;hdr[34]=0xcd;hdr[35]=0xab;
 assert(busaAppHeaderValid(hdr,36));assert(!busaAppHeaderValid(hdr,35));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'wifi_scan_test.cpp'
 p.write_text(stub+'\n#include "BUSA_WifiRecovery.h"\n#include "BUSA_AppHeader.h"\n'+tests)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(root),
                 str(p),'-o',str(Path(d)/'wifi_scan_test')],check=True)
 subprocess.run([str(Path(d)/'wifi_scan_test')],check=True)
print('PASS: Arduino scan, hidden SSID JSON, IDF fallback, failure diagnostics, reconnect, OTA exclusion, ESP32-S3 target')
