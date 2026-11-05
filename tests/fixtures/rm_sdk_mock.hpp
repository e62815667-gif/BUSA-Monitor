#include <string>
#include <map>
#include <vector>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <atomic>
#include <algorithm>
using std::isfinite;
class String:public std::string{public:using std::string::string;String(const std::string&s):std::string(s){}
 String(int x):std::string(std::to_string(x)){}String(unsigned x):std::string(std::to_string(x)){}
 String(float x,int){char b[32];snprintf(b,sizeof b,"%.2f",x);assign(b);}};
using esp_err_t=int;const int ESP_OK=0,ESP_FAIL=-1,ESP_ERR_NOT_FOUND=1,ESP_ERR_INVALID_STATE=2;
enum{RMAKER_VAL_TYPE_INVALID,RMAKER_VAL_TYPE_BOOLEAN,RMAKER_VAL_TYPE_INTEGER,RMAKER_VAL_TYPE_FLOAT,RMAKER_VAL_TYPE_STRING};
struct esp_rmaker_param_val_t{int type=0;union V{bool b;int i;float f;const char* s;V():i(0){}}val;};
esp_rmaker_param_val_t esp_rmaker_float(float x){esp_rmaker_param_val_t v;v.type=3;v.val.f=x;return v;}
esp_rmaker_param_val_t esp_rmaker_int(int x){esp_rmaker_param_val_t v;v.type=2;v.val.i=x;return v;}
esp_rmaker_param_val_t esp_rmaker_bool(bool x){esp_rmaker_param_val_t v;v.type=1;v.val.b=x;return v;}
esp_rmaker_param_val_t esp_rmaker_str(const char* s){esp_rmaker_param_val_t v;v.type=RMAKER_VAL_TYPE_STRING;v.val.s=s;return v;}
struct esp_rmaker_param_t{std::string name,owned,ui;int flags=0;esp_rmaker_param_val_t value;bool dirty=false;int updates=0;};
std::vector<esp_rmaker_param_t*> allParams;
const int PROP_FLAG_READ=1,PROP_FLAG_WRITE=2;
const char* ESP_RMAKER_UI_TOGGLE="esp.ui.toggle";const char* ESP_RMAKER_UI_TEXT="esp.ui.text";
using param_val_t=esp_rmaker_param_val_t;struct write_ctx_t{};
class Param{public:std::string name,ui;param_val_t val;int flags;
 Param(const char* n,const char*,param_val_t v,uint8_t f):name(n),val(v),flags(f){}
 const void* getParamHandle(){return this;}const char* getParamName(){return name.c_str();}
 int addUIType(const char* t){ui=t;return ESP_OK;}
};
param_val_t value(float x){return esp_rmaker_float(x);}param_val_t value(bool x){return esp_rmaker_bool(x);}param_val_t value(const char* x){return esp_rmaker_str(x);}
struct Device;using WriteCallback=void(*)(Device*,Param*,param_val_t,void*,write_ctx_t*);
struct Device{std::map<std::string,esp_rmaker_param_t> params;std::string deviceName;WriteCallback callback=nullptr;
 Device(const char* name="",const char* =nullptr):deviceName(name){}
 const char* getDeviceName(){return deviceName.c_str();}
 int addParam(Param p){add(p.name.c_str(),p.val);auto* h=getParamByName(p.name.c_str());h->flags=p.flags;h->ui=p.ui;return ESP_OK;}
 void addCb(WriteCallback fn){callback=fn;}

 void add(const char* name,esp_rmaker_param_val_t v){auto& p=params[name];p.name=name;p.value=v;allParams.push_back(&p);}
 esp_rmaker_param_t* getParamByName(const char* name){auto it=params.find(name);return it==params.end()?nullptr:&it->second;}
 int updateAndReportParam(const char* n,bool v);int updateAndReportParam(const char* n,int v);int updateAndReportParam(const char* n,float v);
};
struct Node{std::vector<Device*> devices;int addDevice(Device& d){devices.push_back(&d);return ESP_OK;}};
struct FakeSerial{template<class... T>void printf(const char*,T...) {}}Serial0;
using TemperatureSensor=Device;
Device tempDevices[4],water,health,screen,keys;
TemperatureSensor* busaRmTemps[4]={&tempDevices[0],&tempDevices[1],&tempDevices[2],&tempDevices[3]};
Device *busaRmWater=&water,*busaRmHealth=&health,*busaRmScreen=&screen,*busaRmKeys=&keys;
const char *RM_ON="ВКЛ °C",*RM_OFF="ВЫКЛ °C",*RM_CAL="Калибровка °C",*RM_SET="Пороги заданы",*RM_CLEAR="Удалить пороги",*RM_BUZ="Зуммер";
const char *RM_K1="Звук К1",*RM_K2="Звук К2",*RM_K3="Звук К3",*RM_K4="Звук К4",*RM_THEME="Тема (0/1/2)",*RM_SCREEN="Экран включен",*RM_PAGE="Раздел (0-6)",*RM_KEYLEARN="Обучить 20 клавиш",*RM_KEYSTOP="Отмена обучения";
#define ESP_RMAKER_DEF_TEMPERATURE_NAME "Temperature"
uint32_t nowMs=1;uint32_t millis(){return nowMs;}bool elapsed(uint32_t a,uint32_t b,uint32_t d){return uint32_t(a-b)>=d;}
float temperature[4]={24,25,24.81f,27},refTemp[4]={50,50,50,50},noticeTemp[4]={45,45,45,45},tempOffset[4]={};
bool valid[4]={true,true,true,true},buzzerEnabled[2]={},soundEnabled[4]={},googleEnabled=true,buzzerSoundOn=false;
struct Sound{bool on=false;}soundChannels[4];
float flowLpm=1.5f;float totalWaterLitres(){return 2.5f;}bool hasTemp(int n,uint32_t){return valid[n];}
namespace BusaSound{bool valid(float on,float off){return isfinite(on)&&isfinite(off)&&on!=off;}}
bool tftDisplayOff=false,keypadReady=true,keypadLearning=false,pcfDetected=true,otaActive=false,busaWifiScanRunning=false,busaGoogleNetworkBusy=false,busaRmStarted=true;
int tftThemeMode=1,currentPage=0,learnStep=0;
struct Wifi{int status(){return 3;}}WiFi;const int WL_CONNECTED=3;
using TaskHandle_t=void*;TaskHandle_t currentTask=(void*)1;TaskHandle_t xTaskGetCurrentTaskHandle(){return currentTask;}
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
using esp_event_base_t=const char*;const char* RMAKER_COMMON_EVENT="common";const int ESP_EVENT_ANY_ID=-1;
enum{RMAKER_MQTT_EVENT_CONNECTED,RMAKER_MQTT_EVENT_DISCONNECTED,RMAKER_MQTT_EVENT_PUBLISHED,RMAKER_MQTT_EVENT_MSG_DELETED};
using EventFn=void(*)(void*,esp_event_base_t,int32_t,void*);EventFn events=nullptr;
int esp_event_handler_register(esp_event_base_t,int,EventFn fn,void*){events=fn;return ESP_OK;}
using esp_rmaker_mqtt_publish_t=int(*)(const char*,void*,size_t,uint8_t,int*);
struct esp_rmaker_mqtt_config_t{esp_rmaker_mqtt_publish_t publish=nullptr;};esp_rmaker_mqtt_config_t config;
bool connected=true,backendFail=false,autoAck=true,silentDrop=false,queueFail=false;int tokens=100,nextId=1,wireCalls=0,drops=0;uint32_t refillAt=0;
std::map<const esp_rmaker_param_t*,esp_rmaker_param_val_t> cloud,payload;
std::map<const esp_rmaker_param_t*,std::string> cloudStrings;
int backend(const char*,void*,size_t,uint8_t,int* id){++wireCalls;if(backendFail)return ESP_FAIL;
 *id=nextId++;for(auto v:payload){cloud[v.first]=v.second;if(v.second.type==RMAKER_VAL_TYPE_STRING){cloudStrings[v.first]=v.second.val.s;cloud[v.first].val.s=cloudStrings[v.first].c_str();}}
 if(autoAck&&events){int number=*id;events(nullptr,RMAKER_COMMON_EVENT,RMAKER_MQTT_EVENT_PUBLISHED,&number);}return ESP_OK;
}
int esp_rmaker_mqtt_glue_setup(esp_rmaker_mqtt_config_t* c){c->publish=backend;return ESP_OK;}
int esp_rmaker_mqtt_setup(esp_rmaker_mqtt_config_t c){config=c;return ESP_OK;}
bool esp_rmaker_is_mqtt_connected(){return connected;}
bool esp_rmaker_mqtt_is_budget_available(){return tokens>0;}
void advance(uint32_t at){uint32_t delta=uint32_t(at-refillAt);tokens=std::min(1024,tokens+int(delta/5000));refillAt+=delta/5000*5000;nowMs=at;}
int esp_rmaker_param_update(const esp_rmaker_param_t* p,esp_rmaker_param_val_t v){if(!p||p->value.type!=v.type)return ESP_FAIL;auto* m=const_cast<esp_rmaker_param_t*>(p);m->value=v;if(v.type==RMAKER_VAL_TYPE_STRING){m->owned=v.val.s;m->value.val.s=m->owned.c_str();}m->dirty=true;++m->updates;return ESP_OK;}
int esp_rmaker_param_update_and_report(const esp_rmaker_param_t* p,esp_rmaker_param_val_t v){
 if(esp_rmaker_param_update(p,v)!=ESP_OK)return ESP_FAIL;
 payload.clear();for(auto* param:allParams)if(param->dirty){payload[param]=param->value;param->dirty=false;}
 // Matches the pinned SDK: dirty flags cleared and MQTT error not propagated.
 if(silentDrop){++drops;return ESP_OK;}
 if(!connected||tokens<=0){++drops;return ESP_OK;}
 int id=-1;int rc=config.publish("node/test/params/local",nullptr,0,1,&id);if(rc==ESP_OK)--tokens;else ++drops;
 return ESP_OK;
}
int Device::updateAndReportParam(const char*n,bool v){return esp_rmaker_param_update_and_report(getParamByName(n),esp_rmaker_bool(v));}
int Device::updateAndReportParam(const char*n,int v){return esp_rmaker_param_update_and_report(getParamByName(n),esp_rmaker_int(v));}
int Device::updateAndReportParam(const char*n,float v){return esp_rmaker_param_update_and_report(getParamByName(n),esp_rmaker_float(v));}
using WorkFn=void(*)(void*);WorkFn queued=nullptr;
int esp_rmaker_work_queue_add_task(WorkFn fn,void*){if(queueFail)return ESP_FAIL;assert(!queued);queued=fn;return ESP_OK;}
void runWorker(){if(queued){auto fn=queued;queued=nullptr;currentTask=(void*)2;fn(nullptr);currentTask=(void*)1;}}
void initialize(){
 config.publish=backend;
 for(int i=0;i<4;i++){
  tempDevices[i].add(ESP_RMAKER_DEF_TEMPERATURE_NAME,esp_rmaker_float(0));tempDevices[i].add("Датчик подключен",esp_rmaker_bool(false));
  for(auto name:{RM_ON,RM_OFF,RM_CAL})tempDevices[i].add(name,esp_rmaker_float(0));
  for(auto name:{RM_SET,RM_CLEAR})tempDevices[i].add(name,esp_rmaker_bool(false));
  if(i<2)tempDevices[i].add(RM_BUZ,esp_rmaker_bool(false));
  if(i==2)for(auto name:{RM_K1,RM_K2,RM_K3})tempDevices[i].add(name,esp_rmaker_bool(false));
  if(i==3)tempDevices[i].add(RM_K4,esp_rmaker_bool(false));
 }
 for(auto name:{"Расход л/мин","Объем л"})water.add(name,esp_rmaker_float(0));water.add("Калиброван",esp_rmaker_bool(false));
 health.add("Работает секунд",esp_rmaker_int(0));for(auto name:{"Google настроен","Звук К1","Звук К2","Звук К3","Звук К4","Зуммер активен"})health.add(name,esp_rmaker_bool(false));
 screen.add(RM_THEME,esp_rmaker_int(0));screen.add(RM_PAGE,esp_rmaker_int(0));screen.add(RM_SCREEN,esp_rmaker_bool(false));
 for(auto name:{"PCF8575 подключен","Раскладка сохранена","Обучение идет",RM_KEYLEARN,RM_KEYSTOP})keys.add(name,esp_rmaker_bool(false));keys.add("Следующая клавиша",esp_rmaker_int(0));
}

#include "BUSA_RmCalcState.h"
