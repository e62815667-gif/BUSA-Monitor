from pathlib import Path
import subprocess,tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
stub=r'''
#include <string>
#include <cassert>
#include <cstdint>
#include <cstring>
class String:public std::string{
public:using std::string::string;String(const std::string&s):std::string(s){};
String(int v):std::string(std::to_string(v)){} String(unsigned v):std::string(std::to_string(v)){}
bool isEmpty()const{return empty();}
};
const int WL_CONNECTED=3,WIFI_SCAN_RUNNING=-1,WIFI_SCAN_FAILED=-2;
const int ARDUINO_EVENT_WIFI_STA_DISCONNECTED=5;
using WiFiEvent_t=int;struct WiFiEventInfo_t{struct{uint8_t reason;}wifi_sta_disconnected;};
bool otaActive=false,otaRestart=false;uint32_t lastStationCheck=0;
String stationSsid="iPhone 16",stationPass="testpass",nearbyNames[5];uint8_t nearbyCount=0;
bool elapsed(uint32_t a,uint32_t b,uint32_t d){return uint32_t(a-b)>=d;}
String quoteJson(const String&s){return "\""+s+"\"";}
struct MockWiFi{
int state=0,startResult=-1,complete=-1,starts=0,begins=0,disconnects=0;bool autoReconnect=true;
void setAutoReconnect(bool value){autoReconnect=value;}
int status(){return state;}void disconnect(bool off,bool erase){assert(!off&&!erase);++disconnects;state=0;}
void scanDelete(){}int scanNetworks(bool a,bool h,bool p,int ms){assert(a&&!h&&!p&&ms==120);++starts;return startResult;}
int scanComplete(){return complete;}String SSID(int){return "iPhone 12";}int RSSI(int){return -51;}
void begin(const char*,const char*){++begins;}
}WiFi;
int stops=0;void esp_wifi_scan_stop(){++stops;}
'''
tests=r'''
int main(){
 wifiScheduleConnect(0);wifiRecoveryTick(499);assert(WiFi.begins==0);
 wifiRecoveryTick(500);assert(WiFi.begins==1);
 wifiRequestScan(20000);assert(!WiFi.autoReconnect);
 wifiRequestScan(20001);wifiRecoveryTick(20499);assert(WiFi.starts==0);
 wifiRecoveryTick(20500);assert(WiFi.starts==1&&WiFi.begins==1);
 wifiRecoveryTick(21000);assert(WiFi.begins==1);
 WiFi.complete=1;wifiRecoveryTick(22000);assert(!wifiScanBusy());assert(WiFi.autoReconnect);
 assert(wifiScanJSON().find("iPhone 12")!=std::string::npos);
 auto cached=wifiScanJSON();assert(cached==wifiScanJSON());
 wifiRecoveryTick(22499);assert(WiFi.begins==1);wifiRecoveryTick(22500);assert(WiFi.begins==2);
 WiFi.startResult=-2;wifiRequestScan(30000);
 wifiRecoveryTick(30500);wifiRecoveryTick(31000);wifiRecoveryTick(31500);
 assert(!wifiScanBusy()&&WiFi.autoReconnect &&wifiScanJSON().find("scan_start_failed")!=std::string::npos);
 WiFi.startResult=-1;WiFi.complete=-1;wifiRequestScan(40000);wifiRecoveryTick(40500);
 wifiRecoveryTick(55000);assert(!wifiScanBusy()&&stops==1);
 wifiRecoveryTick(55500);assert(WiFi.begins==3);
 WiFi.state=WL_CONNECTED;int dc=WiFi.disconnects;wifiRequestScan(60000);assert(WiFi.disconnects==dc);
 wifiRecoveryTick(60500);WiFi.complete=0;wifiRecoveryTick(61000);assert(!wifiConnectPending);
 otaActive=true;int b=WiFi.begins;WiFi.state=0;wifiRecoveryTick(90000);assert(WiFi.begins==b);
 otaActive=false;wifiRecoveryTick(90001);wifiRecoveryTick(90501);assert(WiFi.begins==b+1);
 wifiCancelScan();assert(WiFi.autoReconnect);
 wifiScheduleConnect(UINT32_MAX-100);assert(!WiFi.autoReconnect);
 wifiRecoveryTick(400);assert(WiFi.begins==b+2&&WiFi.autoReconnect);
 wifiRecoveryEvent(5,{{201}});assert(wifiDisconnectReason==201&&wifiDisconnectCount==1);
 uint8_t hdr[36]={};hdr[0]=0xe9;hdr[1]=3;hdr[12]=9;hdr[32]=0x32;hdr[33]=0x54;hdr[34]=0xcd;hdr[35]=0xab;
 assert(busaAppHeaderValid(hdr,36));assert(!busaAppHeaderValid(hdr,35));
 hdr[32]=0;assert(!busaAppHeaderValid(hdr,36));hdr[32]=0x32;hdr[12]=0;assert(!busaAppHeaderValid(hdr,36));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';p.write_text(stub+'\n#include "BUSA_WifiRecovery.h"\n#include "BUSA_AppHeader.h"\n'+tests)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(root),str(p),'-o',d+'/test'],check=True)
 subprocess.run([d+'/test'],check=True)
print('PASS: real WiFi scheduler: scan/connect exclusion, idempotence, cached results, retry exhaustion, timeout, reconnect, OTA pause, clock wrap; APP target/header checks')
