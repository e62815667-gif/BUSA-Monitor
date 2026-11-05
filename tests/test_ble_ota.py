"""Execute shipping BLE callbacks/recovery, OTA boot policy and credential sync.
All SDK and NVS calls are mocks. No ESP32, phone, cloud or Flash is contacted.
"""
from pathlib import Path
import os,re,subprocess,tempfile

ROOT=Path(__file__).resolve().parents[1]
SKETCH=ROOT/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
MAIN=(SKETCH/'BUSA_Main.h').read_text()
RAIN=(SKETCH/'BUSA_RainMaker.h').read_text()


def function(source, signature):
    start=source.index(signature);first=source.index('{',start);depth=1;pos=first+1
    while depth:
        if source[pos]=='{':depth+=1
        elif source[pos]=='}':depth-=1
        pos+=1
    return source[start:pos]


def run_cpp(code, name):
    with tempfile.TemporaryDirectory() as directory:
        source=Path(directory)/(name+'.cpp');binary=Path(directory)/name
        source.write_text(code)
        subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(SKETCH),str(source),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)


COMMON=r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <map>
#include <atomic>
class String:public std::string{public:using std::string::string;String(const std::string&s):std::string(s){}bool isEmpty()const{return empty();}};
using esp_err_t=int;const int ESP_OK=0,ESP_FAIL=-1,ESP_ERR_NOT_FOUND=1,ESP_ERR_INVALID_STATE=2,ESP_ERR_NOT_SUPPORTED=3;
'''

run_cpp(COMMON+r'''
#include "BUSA_BleBootRecovery.h"
int main(){
 BusaBleBootRecovery s;auto a=s.tick(100,100,true,false,false);assert(!a.restart&&!a.stop);
 // Even after a successful credential event, waiting until PROV_END is mandatory.
 a=s.tick(9000,100,false,false,false);assert(!a.restart&&!a.stop);
 a=s.tick(20000,100,false,true,false);assert(!a.restart);
 assert(!s.tick(23999,100,false,true,false).restart);
 assert(s.tick(24000,100,false,true,false).restart);
 BusaBleBootRecovery noadv;
 assert(!noadv.tick(29999,0,false,false,false).stop);
 assert(noadv.tick(30000,0,false,false,false).stop);
 assert(!noadv.tick(30001,0,false,false,false).stop);
 assert(!noadv.tick(45000,0,false,false,false).restart&&noadv.stopTimeout);
 assert(noadv.tick(49000,0,false,false,false).restart);
 BusaBleBootRecovery timeout;timeout.tick(1,0,true,false,false);
 assert(timeout.tick(180000,0,false,false,false).stop);
 timeout.tick(181000,0,false,true,false);assert(timeout.tick(185000,0,false,true,false).restart);
 BusaBleBootRecovery failed;assert(!failed.tick(0,0,false,false,true).restart);
 assert(failed.tick(4000,0,false,false,true).restart);
 BusaBleBootRecovery wrap;uint32_t begin=UINT32_MAX-1000;
 assert(!wrap.tick(begin,begin,true,false,false).restart);
 wrap.tick(begin+2000,begin,false,true,false);
 assert(wrap.tick(begin+6000,begin,false,true,false).restart);
}
''','recovery')

run_cpp(COMMON+r'''
struct esp_partition_t{const char*label;uint32_t address;};
enum esp_ota_img_states_t{ESP_OTA_IMG_UNDEFINED,ESP_OTA_IMG_PENDING_VERIFY,ESP_OTA_IMG_VALID,ESP_OTA_IMG_INVALID};
esp_partition_t current={"ota_0",0x10000},next={"ota_1",0x310000};
const esp_partition_t* running=&current;const esp_partition_t* boot=&next;
esp_ota_img_states_t imageState=ESP_OTA_IMG_PENDING_VERIFY;
int queryRc=ESP_OK,markRc=ESP_OK,marks=0;
const esp_partition_t* esp_ota_get_running_partition(){return running;}
const esp_partition_t* esp_ota_get_boot_partition(){return boot;}
int esp_ota_get_state_partition(const esp_partition_t*,esp_ota_img_states_t* out){*out=imageState;return queryRc;}
int esp_ota_mark_app_valid_cancel_rollback(){++marks;if(markRc==ESP_OK)imageState=ESP_OTA_IMG_VALID;return markRc;}
// Deliberately NO esp_ota_set_boot_partition API: compiling a reselection is an error.
struct SerialMock{void printf(const char*,...){}}Serial0;
int busaOtaConfirmResult=ESP_ERR_NOT_FOUND;
String busaOtaRunning,busaOtaBoot;
#include "BUSA_OtaBoot.h"
int main(){
 assert(busaOtaPendingVerification());busaConfirmRunningImage(false);
 assert(marks==0&&busaOtaPendingVerification()&&busaOtaConfirmResult==ESP_ERR_INVALID_STATE);
 busaConfirmRunningImage(true);assert(marks==1&&!busaOtaPendingVerification());
 assert(busaOtaConfirmResult==ESP_OK&&boot==&next);
 busaConfirmRunningImage(true);assert(marks==1&&boot==&next);
 imageState=ESP_OTA_IMG_PENDING_VERIFY;markRc=ESP_FAIL;busaConfirmRunningImage(true);
 assert(marks==2&&busaOtaPendingVerification()&&busaOtaConfirmResult==ESP_FAIL&&boot==&next);
 queryRc=ESP_ERR_NOT_FOUND;busaConfirmRunningImage(true);assert(marks==2&&boot==&next);
 queryRc=ESP_FAIL;busaConfirmRunningImage(true);assert(marks==2&&busaOtaConfirmResult==ESP_FAIL);
 running=nullptr;busaConfirmRunningImage(true);assert(busaOtaConfirmResult==ESP_ERR_NOT_FOUND&&marks==2);
}
''','ota_boot')

callback_stub=COMMON+r'''
enum {ARDUINO_EVENT_PROV_START,ARDUINO_EVENT_PROV_INIT,ARDUINO_EVENT_PROV_CRED_SUCCESS,ARDUINO_EVENT_PROV_CRED_FAIL,ARDUINO_EVENT_PROV_END};
struct arduino_event_t{int event_id;};
std::atomic<bool> busaRmBleAdvertising{false},busaRmBleEnded{false},busaRmBleCredentialsAccepted{false},busaRmBleCredentialsRejected{false},busaRmBleStopRequested{false};
int stops=0,autoStopDelay=0,begins=0;uint32_t busaRmBleSessionAt=0;
bool busaRmStarted=true,busaRmBleSessionStarted=false,busaRmBleSessionRequested=true,busaRmBleTimeoutPending=false;
String busaRmBleFeedback;char busaRmPoP[16]="test-only",busaRmService[24]="PROV_BUSA_TEST";
uint32_t millis(){return 101;}
void wifi_prov_mgr_stop_provisioning(){++stops;}
void wifi_prov_mgr_disable_auto_stop(int delay){autoStopDelay=delay;}
const int WIFI_PROV_SCHEME_BLE=1,WIFI_PROV_SCHEME_HANDLER_FREE_BTDM=2,WIFI_PROV_SECURITY_1=3;
struct ProvMock{void beginProvision(int scheme,int handler,int security,const char*pop,const char*name,void*key,void*uuid,bool reset){
 assert(scheme==1&&handler==2&&security==3&&pop==busaRmPoP&&name==busaRmService&&key==nullptr&&uuid==nullptr&&reset);++begins;
}}WiFiProv;
'''
run_cpp(callback_stub+'\n'+function(RAIN,'static void busaRmStopProvisioning()')+'\n'+function(RAIN,'static void busaRmProvisionEvent(')+'\n'+function(RAIN,'static void busaRainMakerProvision()')+r'''
int main(){
 busaRmStarted=false;busaRainMakerProvision();assert(begins==0);
 busaRmStarted=true;busaRainMakerProvision();assert(begins==1&&busaRmBleSessionStarted&&busaRmBleSessionAt==101);
 arduino_event_t e={ARDUINO_EVENT_PROV_INIT};busaRmProvisionEvent(&e);assert(autoStopDelay==10000);
 e.event_id=ARDUINO_EVENT_PROV_START;busaRmProvisionEvent(&e);assert(busaRmBleAdvertising);
 e.event_id=ARDUINO_EVENT_PROV_CRED_FAIL;busaRmProvisionEvent(&e);assert(busaRmBleCredentialsRejected&&!busaRmBleCredentialsAccepted);
 e.event_id=ARDUINO_EVENT_PROV_CRED_SUCCESS;busaRmProvisionEvent(&e);
 assert(busaRmBleCredentialsAccepted&&!busaRmBleCredentialsRejected&&stops==1);
 busaRmProvisionEvent(&e);busaRmStopProvisioning();assert(stops==1);
 e.event_id=ARDUINO_EVENT_PROV_END;busaRmProvisionEvent(&e);assert(busaRmBleEnded&&!busaRmBleAdvertising);
}
''','official_callbacks')

credential_stub=COMMON+r'''
const int WIFI_IF_STA=1;
struct wifi_config_t{struct{uint8_t ssid[32];uint8_t password[64];}sta;}config={};
int configRc=ESP_OK;
int esp_wifi_get_config(int type,wifi_config_t*out){assert(type==WIFI_IF_STA);*out=config;return configRc;}
struct Prefs{
 std::map<std::string,String> data;String failKey;int writes=0;
 String getString(const String&key,const char* fallback){return data.count(key)?data[key]:String(fallback);}
 int putString(const String&key,const String&value){++writes;if(key==failKey){failKey="";return 0;}data[key]=value;return value.length();}
}prefs;
'''
run_cpp(credential_stub+'\n'+function(MAIN,'static bool busaSaveProvisionedStation()')+r'''
int main(){
 prefs.data["ssid"]="previous";prefs.data["pass"]="previous-test";prefs.data["staOff"]="1";
 memcpy(config.sta.ssid,"new-test-network",16);memcpy(config.sta.password,"test-only-password",18);
 assert(busaSaveProvisionedStation());assert(prefs.data["ssid"]=="new-test-network");assert(prefs.data["staOff"]=="1");
 prefs.failKey="pass";memset(config.sta.password,0,sizeof(config.sta.password));memcpy(config.sta.password,"different-test",14);
 assert(!busaSaveProvisionedStation());assert(prefs.data["pass"]=="test-only-password");
 memset(config.sta.ssid,'s',32);assert(busaSaveProvisionedStation());assert(prefs.data["ssid"].length()==32);
 int before=prefs.writes;configRc=ESP_FAIL;assert(!busaSaveProvisionedStation()&&prefs.writes==before);
 configRc=ESP_OK;memset(config.sta.password,0,64);assert(!busaSaveProvisionedStation()&&prefs.writes==before);
 memset(config.sta.password,'p',64);assert(!busaSaveProvisionedStation()&&prefs.writes==before);
}
''','credential_sync')

# Integration: these requests survive a pending-image boot, but cannot reboot it early.
assert 'busaRmPairDeferred=busaNvsReady&&prefs.getBool("rmPairNext",false)&&busaOtaPendingVerification();' in MAIN
assert 'if(busaNvsReady&&prefs.getBool("rmPairNext",false)&&!busaRmPairDeferred)' in MAIN
assert 'if(busaRmStartRequested&&!busaOtaPendingVerification())' in MAIN
assert 'if(busaRmPairDeferred&&!busaOtaPendingVerification())busaRmStartRequested=true;' in MAIN
assert 'const bool paired=busaRmBleCredentialsAccepted&&busaRmBleEnded;' in MAIN
assert 'esp_ota_set_boot_partition(' not in MAIN
assert 'PROV_BUSA_%04X' in MAIN and 'PROV_BUSA_%04X' in RAIN
assert 'RMaker.enableOTA(' not in RAIN
assert 'millis()-busaRmBleSessionAt>=8000' not in MAIN
assert 'PoP %s' not in RAIN

# Legacy build entry points must be repeatable validators, never hidden code changes.
before={p:p.read_bytes() for p in SKETCH.iterdir() if p.is_file()}
for _ in range(2):
    for script in ['ble_cold_start_patch.py','ble_official_provisioning.py','busa_ble_ota_stability.py']:
        subprocess.run(['python3','ci/'+script],cwd=ROOT,check=True,env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1'},stdout=subprocess.DEVNULL)
assert before=={p:p.read_bytes() for p in SKETCH.iterdir() if p.is_file()}
print('PASS real FIX3 helpers: official BLE events, stop-once, END wait, startup/cleanup timeouts, rollover, OTA pending/valid/error states, no boot reselection, credential sync/rollback, source-preserving build validators')
