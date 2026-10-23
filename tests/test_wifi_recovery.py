from pathlib import Path
import subprocess, tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
stub=r'''
#include <string>
#include <cassert>
#include <cstdint>
class String:public std::string{
public:using std::string::string;
 String(const std::string& s):std::string(s){}
 String(int v):std::string(std::to_string(v)){}
 String(unsigned v):std::string(std::to_string(v)){}
 bool isEmpty()const{return empty();}
};
const int WL_CONNECTED=3,WIFI_SCAN_RUNNING=-1,WIFI_SCAN_FAILED=-2;
const int ARDUINO_EVENT_WIFI_STA_DISCONNECTED=5;
using WiFiEvent_t=int; struct WiFiEventInfo_t {struct{uint8_t reason;}wifi_sta_disconnected;};
bool otaActive=false,otaRestart=false;uint32_t lastStationCheck=0,nowMs=0;
uint32_t millis(){return nowMs;}
String stationSsid="iPhone 16",stationPass="testpass",nearbyNames[5];uint8_t nearbyCount=0;
bool elapsed(uint32_t a,uint32_t b,uint32_t d){return uint32_t(a-b)>=d;}
String quoteJson(const String&s){return "\""+s+"\"";}
struct MockWiFi{
 int state=WL_CONNECTED,scanStarted=0,scanDeletes=0,scanCompletes=0;
 int begins=0,reconnects=0,disconnects=0,scanReturn=WIFI_SCAN_RUNNING,scanState=WIFI_SCAN_RUNNING;
 bool autoReconnect=false;
 void setAutoReconnect(bool v){autoReconnect=v;}
 int status(){return state;}
 void disconnect(bool a,bool b){assert(!a&&!b);disconnects++;state=0;}
 int scanNetworks(bool async,bool hidden){assert(async&&!hidden);scanStarted++;return scanReturn;}
 int scanComplete(){scanCompletes++;return scanState;}
 void scanDelete(){scanDeletes++;}
 String SSID(int n){return n==0?"":n==1?"iPhone 12":"iPhone 16";}
 int RSSI(int){return -52;}
 void begin(const char*,const char*){begins++;}
 void reconnect(){reconnects++;}
}WiFi;
'''
test=r'''
int main(){
 nowMs=1000;wifiRequestScan(1000);
 assert(WiFi.scanStarted==1 && wifiScanBusy());
 wifiRequestScan(1001);assert(WiFi.scanStarted==1);
 assert(wifiScanJSON().find("scanning")!=std::string::npos);
 nowMs=2000;WiFi.scanState=3;
 String res=wifiScanJSON();
 assert(!wifiScanBusy() && wifiScanSuccesses==1);
 assert(res.find("iPhone 12")!=std::string::npos);
 assert(res.find("[,")==std::string::npos); // V14.14 hidden SSID edge case
 assert(wifiLastApCount==3&&wifiLastVisibleCount==2);
 assert(WiFi.scanDeletes>=2);
 // Start failure is reported instead of eternal loading.
 WiFi.scanReturn=WIFI_SCAN_FAILED;
 wifiRequestScan(3000);
 assert(!wifiScanBusy() && wifiScanFailures==1);
 assert(wifiScanJSON().find("scan_start_failed")!=std::string::npos);
 // Timeout returns explicit error and lets user retry.
 WiFi.scanReturn=WIFI_SCAN_RUNNING;WiFi.scanState=WIFI_SCAN_RUNNING;
 wifiRequestScan(4000);nowMs=24001;
 assert(wifiScanJSON().find("scan_timeout")!=std::string::npos);
 assert(!wifiScanBusy()&&wifiScanFailures==2);
 // Reconnect only when not scanning or OTA-busy.
 WiFi.state=0;lastStationCheck=nowMs;wifiScheduleConnect(nowMs);
 wifiRequestScan(25000);
 int old=WiFi.begins;
 wifiRecoveryTick(26000);
 assert(WiFi.begins==old);
 wifiCancelScan();wifiScheduleConnect(26000);
 wifiRecoveryTick(26500);
 assert(WiFi.begins==old+1&&WiFi.autoReconnect);
 WiFi.state=0;lastStationCheck=26500;
 wifiRecoveryTick(46500);
 assert(WiFi.reconnects==1);
 otaActive=true;wifiRecoveryTick(66500);
 assert(WiFi.reconnects==1);
 otaActive=false;
 wifiRecoveryEvent(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{{201}});
 assert(wifiDisconnectReason==201&&wifiDisconnectCount==1);
 uint8_t hdr[36]={};
 hdr[0]=0xe9;hdr[1]=3;hdr[12]=9;
 hdr[32]=0x32;hdr[33]=0x54;hdr[34]=0xcd;hdr[35]=0xab;
 assert(busaAppHeaderValid(hdr,36));
 assert(!busaAppHeaderValid(hdr,35));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'wifi_test.cpp'
 p.write_text(stub+'\n#include "BUSA_WifiRecovery.h"\n#include "BUSA_AppHeader.h"\n'+test)
 exe=Path(d)/'wifi_test'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(root),
                 str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('PASS: V14.14 scan/complete behavior, blank SSIDs, startup failure, timeout, OTA gating, STA reconnect and APP header')
