#pragma once
#include <esp_rmaker_mqtt.h>
#include <esp_rmaker_common_events.h>
#include <esp_rmaker_work_queue.h>
#include <freertos/task.h>
#include "BUSA_RmPublishTracker.h"

static constexpr uint32_t BUSA_RM_REPORT_MS=10000U;
// Round only the RainMaker presentation value; keep sensor/local/Google precision.
static float busaRmTemperatureForCloud(float measured){
  const float rounded=roundf(measured*10.0f)/10.0f;
  return rounded==0.0f?0.0f:rounded;
}
struct BusaRmFrame {
  uint32_t seq=0,at=0,revision=0;int uptime=0;
  float temperatures[4]={},on[4]={},off[4]={},cal[4]={},flow=0,total=0;
  bool valid[4]={},set[4]={},buzzer[2]={},soundEnabled[4]={},soundOn[4]={};
  bool screen=false,keyReady=false,keyLearning=false,pcf=false,google=false,buzzerOn=false;
  int theme=0,page=0,keyStep=0;
};
static BusaRmFrame busaRmFrame;
static std::atomic<bool> busaRmFrameBusy{false},busaRmSnapshotPaused{false};
static std::atomic<uint32_t> busaRmSettingsRevision{0},busaRmSyncEpoch{1};
static std::atomic<uint32_t> busaRmQueueFailures{0},busaRmWriteQueueDrops{0};
static std::atomic<uint32_t> busaRmModelFailures{0},busaRmReportCalls{0},busaRmReportApiOk{0};
static std::atomic<uint32_t> busaRmReportNoTransport{0},busaRmTransportFailures{0},busaRmStaleFrames{0};
static std::atomic<uint32_t> busaRmBudgetDeferrals{0};
static std::atomic<int> busaRmLastReportRc{ESP_ERR_NOT_FOUND},busaRmLastTransportRc{ESP_ERR_NOT_FOUND};
static esp_err_t busaRmTelemetryInitRc=ESP_ERR_NOT_FOUND;
static esp_rmaker_mqtt_publish_t busaRmOriginalPublish=nullptr;
static portMUX_TYPE busaRmTrackerMux=portMUX_INITIALIZER_UNLOCKED;
static BusaRmPublishTracker busaRmTracker;
static std::atomic<TaskHandle_t> busaRmReportingTask{nullptr};
static bool busaRmInReport=false; // Read only on the reporting task, behind task-identity check.
static BusaRmPublishTracker busaRmTrackerCopy(){
  portENTER_CRITICAL(&busaRmTrackerMux);auto copy=busaRmTracker;portEXIT_CRITICAL(&busaRmTrackerMux);return copy;
}
static void busaRmMqttEvent(void*,esp_event_base_t,int32_t event,void* data){
  portENTER_CRITICAL(&busaRmTrackerMux);
  if(event==RMAKER_MQTT_EVENT_CONNECTED)busaRmTracker.connected();
  else if(event==RMAKER_MQTT_EVENT_DISCONNECTED)busaRmTracker.disconnected();
  else if(event==RMAKER_MQTT_EVENT_PUBLISHED&&data)busaRmTracker.published(*(int*)data,millis());
  else if(event==RMAKER_MQTT_EVENT_MSG_DELETED&&data)busaRmTracker.messageDeleted(*(int*)data);
  portEXIT_CRITICAL(&busaRmTrackerMux);
  if(event==RMAKER_MQTT_EVENT_CONNECTED||event==RMAKER_MQTT_EVENT_DISCONNECTED||event==RMAKER_MQTT_EVENT_MSG_DELETED)++busaRmSyncEpoch;
}
static esp_err_t busaRmObservePublish(const char* topic,void* data,size_t size,uint8_t qos,int* messageId){
  const uint32_t started=millis();int id=-1;
  // Identical official transport and arguments, only requesting the optional message ID.
  const esp_err_t rc=busaRmOriginalPublish(topic,data,size,qos,&id);
  if(messageId)*messageId=id;
  const char* suffix="/params/local";
  const bool params=topic&&strlen(topic)>=strlen(suffix)&&strcmp(topic+strlen(topic)-strlen(suffix),suffix)==0;
  if(xTaskGetCurrentTaskHandle()==busaRmReportingTask.load()&&busaRmInReport&&params){
    busaRmLastTransportRc=rc;
    if(rc==ESP_OK&&qos==1&&id>0){
      portENTER_CRITICAL(&busaRmTrackerMux);
      busaRmTracker.submitted(id,started,millis(),busaRmFrame.seq,(uint32_t)busaRmFrame.uptime);
      portEXIT_CRITICAL(&busaRmTrackerMux);
    }else{++busaRmTransportFailures;}
  }
  return rc;
}
static void busaRmTelemetryInit(){
  esp_rmaker_mqtt_config_t config={};
  busaRmTelemetryInitRc=esp_rmaker_mqtt_glue_setup(&config);
  if(busaRmTelemetryInitRc!=ESP_OK)return;
  if(!config.publish){busaRmTelemetryInitRc=ESP_ERR_INVALID_STATE;return;}
  busaRmOriginalPublish=config.publish;config.publish=busaRmObservePublish;
  busaRmTelemetryInitRc=esp_event_handler_register(RMAKER_COMMON_EVENT,ESP_EVENT_ANY_ID,busaRmMqttEvent,nullptr);
  if(busaRmTelemetryInitRc!=ESP_OK)return;
  busaRmTelemetryInitRc=esp_rmaker_mqtt_setup(config);
}
struct BusaRmCachedParam {const esp_rmaker_param_t* handle=nullptr;esp_rmaker_param_val_t value={};bool valid=false;};
static BusaRmCachedParam busaRmCache[64];
static void busaRmInvalidateCache(){for(auto& p:busaRmCache)p.valid=false;}
static bool busaRmSameValue(const esp_rmaker_param_val_t& a,const esp_rmaker_param_val_t& b){
  if(a.type!=b.type)return false;
  if(a.type==RMAKER_VAL_TYPE_BOOLEAN)return a.val.b==b.val.b;
  if(a.type==RMAKER_VAL_TYPE_INTEGER)return a.val.i==b.val.i;
  if(a.type==RMAKER_VAL_TYPE_FLOAT)return a.val.f==b.val.f;
  return false;
}
static bool busaRmStage(Device* device,const char* name,esp_rmaker_param_val_t value,bool force=false){
  const auto* handle=device?device->getParamByName(name):nullptr;
  if(!handle){++busaRmModelFailures;return false;}
  if(value.type==RMAKER_VAL_TYPE_FLOAT&&!isfinite(value.val.f)){++busaRmModelFailures;return false;}
  BusaRmCachedParam* entry=nullptr;
  for(auto& p:busaRmCache)if(p.handle==handle){entry=&p;break;}
  if(!entry)for(auto& p:busaRmCache)if(!p.handle){entry=&p;p.handle=handle;break;}
  if(!entry){++busaRmModelFailures;return false;}
  if(!force&&entry->valid&&busaRmSameValue(entry->value,value))return true;
  const esp_err_t rc=esp_rmaker_param_update(handle,value);
  if(rc!=ESP_OK){entry->valid=false;++busaRmModelFailures;return false;}
  entry->value=value;entry->valid=true;return true;
}
static void busaRmReportFrame(void*){
  // Only this worker mutates the parameter cache. Never read sensor/UI/NVS state here.
  const auto& f=busaRmFrame;static uint32_t lastEpoch=0,lastRevision=UINT32_MAX;
  if(busaRmSnapshotPaused.load()||!esp_rmaker_is_mqtt_connected()){
    busaRmFrameBusy=false;return;
  }
  if(uint32_t(millis()-f.at)>20000U){++busaRmStaleFrames;busaRmFrameBusy=false;return;}
  if(!esp_rmaker_mqtt_is_budget_available()){++busaRmBudgetDeferrals;busaRmFrameBusy=false;return;}
  const uint32_t epoch=busaRmSyncEpoch.load();
  if(epoch!=lastEpoch||f.revision!=lastRevision)busaRmInvalidateCache();
  bool ok=true;
  for(int i=0;i<4;i++){
    if(f.valid[i])ok=busaRmStage(busaRmTemps[i],ESP_RMAKER_DEF_TEMPERATURE_NAME,esp_rmaker_float(busaRmTemperatureForCloud(f.temperatures[i])),true)&&ok;
    ok=busaRmStage(busaRmTemps[i],"Датчик подключен",esp_rmaker_bool(f.valid[i]))&&ok;
    ok=busaRmStage(busaRmTemps[i],RM_ON,esp_rmaker_float(f.on[i]))&&ok;
    ok=busaRmStage(busaRmTemps[i],RM_OFF,esp_rmaker_float(f.off[i]))&&ok;
    ok=busaRmStage(busaRmTemps[i],RM_CAL,esp_rmaker_float(f.cal[i]))&&ok;
    ok=busaRmStage(busaRmTemps[i],RM_SET,esp_rmaker_bool(f.set[i]))&&ok;
    ok=busaRmStage(busaRmTemps[i],RM_CLEAR,esp_rmaker_bool(false))&&ok;
    if(i<2)ok=busaRmStage(busaRmTemps[i],RM_BUZ,esp_rmaker_bool(f.buzzer[i]))&&ok;
    if(i==2){
      ok=busaRmStage(busaRmTemps[i],RM_K1,esp_rmaker_bool(f.soundEnabled[0]))&&ok;
      ok=busaRmStage(busaRmTemps[i],RM_K2,esp_rmaker_bool(f.soundEnabled[1]))&&ok;
      ok=busaRmStage(busaRmTemps[i],RM_K3,esp_rmaker_bool(f.soundEnabled[2]))&&ok;
    }
    if(i==3)ok=busaRmStage(busaRmTemps[i],RM_K4,esp_rmaker_bool(f.soundEnabled[3]))&&ok;
  }
  if(isfinite(f.flow))ok=busaRmStage(busaRmWater,"Расход л/мин",esp_rmaker_float(f.flow))&&ok;
  if(isfinite(f.total))ok=busaRmStage(busaRmWater,"Объем л",esp_rmaker_float(f.total))&&ok;
  ok=busaRmStage(busaRmWater,"Калиброван",esp_rmaker_bool(false))&&ok;
  ok=busaRmStage(busaRmHealth,"Google настроен",esp_rmaker_bool(f.google))&&ok;
  const char* sounds[]={"Звук К1","Звук К2","Звук К3","Звук К4"};
  for(int i=0;i<4;i++)ok=busaRmStage(busaRmHealth,sounds[i],esp_rmaker_bool(f.soundOn[i]))&&ok;
  ok=busaRmStage(busaRmHealth,"Зуммер активен",esp_rmaker_bool(f.buzzerOn))&&ok;
  ok=busaRmStage(busaRmScreen,RM_THEME,esp_rmaker_int(f.theme))&&ok;
  ok=busaRmStage(busaRmScreen,RM_SCREEN,esp_rmaker_bool(f.screen))&&ok;
  ok=busaRmStage(busaRmScreen,RM_PAGE,esp_rmaker_int(f.page))&&ok;
  ok=busaRmStage(busaRmKeys,"PCF8575 подключен",esp_rmaker_bool(f.pcf))&&ok;
  ok=busaRmStage(busaRmKeys,"Раскладка сохранена",esp_rmaker_bool(f.keyReady))&&ok;
  ok=busaRmStage(busaRmKeys,"Обучение идет",esp_rmaker_bool(f.keyLearning))&&ok;
  ok=busaRmStage(busaRmKeys,"Следующая клавиша",esp_rmaker_int(f.keyStep))&&ok;
  ok=busaRmStage(busaRmKeys,RM_KEYLEARN,esp_rmaker_bool(false))&&ok;
  ok=busaRmStage(busaRmKeys,RM_KEYSTOP,esp_rmaker_bool(false))&&ok;
  const auto* heartbeat=busaRmHealth?busaRmHealth->getParamByName("Работает секунд"):nullptr;
  if(!ok||!heartbeat){busaRmInvalidateCache();busaRmFrameBusy=false;return;}
  // Official batched API: N local updates, ONE report for the entire node.
  ++busaRmReportCalls;busaRmLastTransportRc=ESP_ERR_NOT_FOUND;busaRmReportingTask=xTaskGetCurrentTaskHandle();busaRmInReport=true;
  const esp_err_t rc=esp_rmaker_param_update_and_report(heartbeat,esp_rmaker_int(f.uptime));
  busaRmInReport=false;busaRmLastReportRc=rc;
  if(rc==ESP_OK)++busaRmReportApiOk;
  const bool transported=busaRmTrackerCopy().lastEnqueuedSeq==f.seq;
  if(rc!=ESP_OK||!transported){
    // This SDK can return ESP_OK despite a dropped publish. Restage on retry.
    ++busaRmReportNoTransport;busaRmInvalidateCache();
  }else{lastEpoch=epoch;lastRevision=f.revision;}
  busaRmFrameBusy=false;
}
static void busaRmCaptureAndQueue(uint32_t now){
  static bool scheduled=false,wasConnected=false;static uint32_t lastAt=0,lastEpoch=0,nextSeq=0;
  const bool connected=busaRmStarted&&busaRmTelemetryInitRc==ESP_OK&&esp_rmaker_is_mqtt_connected();
  if(!connected){wasConnected=false;return;}
  if(!wasConnected){++busaRmSyncEpoch;wasConnected=true;scheduled=false;}
  bool expired=false;
  portENTER_CRITICAL(&busaRmTrackerMux);expired=busaRmTracker.expire(now);portEXIT_CRITICAL(&busaRmTrackerMux);
  if(expired)++busaRmSyncEpoch;
  const uint32_t epoch=busaRmSyncEpoch.load();
  if(scheduled&&epoch==lastEpoch&&uint32_t(now-lastAt)<BUSA_RM_REPORT_MS)return;
  if(busaRmSnapshotPaused.load()||busaRmFrameBusy.exchange(true))return;
  if(!esp_rmaker_mqtt_is_budget_available()){
    ++busaRmBudgetDeferrals;busaRmFrameBusy=false;scheduled=true;lastAt=now;lastEpoch=epoch;return;
  }
  auto& f=busaRmFrame;f.seq=++nextSeq;f.at=now;f.revision=busaRmSettingsRevision.load();f.uptime=(int)(now/1000U);
  for(int i=0;i<4;i++){
    f.valid[i]=hasTemp(i,now)&&isfinite(temperature[i]);f.temperatures[i]=temperature[i];
    f.on[i]=isfinite(refTemp[i])?refTemp[i]:0.0f;f.off[i]=isfinite(noticeTemp[i])?noticeTemp[i]:0.0f;
    f.cal[i]=tempOffset[i];f.set[i]=BusaSound::valid(refTemp[i],noticeTemp[i]);
    f.soundEnabled[i]=soundEnabled[i];f.soundOn[i]=soundChannels[i].on;
    if(i<2)f.buzzer[i]=buzzerEnabled[i];
  }
  f.flow=flowLpm;f.total=totalWaterLitres();f.google=googleEnabled;f.buzzerOn=buzzerSoundOn;
  f.theme=(int)tftThemeMode;f.screen=!tftDisplayOff;f.page=(int)currentPage;
  f.pcf=pcfDetected;f.keyReady=keypadReady;f.keyLearning=keypadLearning;
  f.keyStep=keypadLearning?(int)learnStep+1:0;
  const esp_err_t rc=esp_rmaker_work_queue_add_task(busaRmReportFrame,nullptr);
  scheduled=true;lastAt=now;lastEpoch=epoch;
  if(rc!=ESP_OK){++busaRmQueueFailures;busaRmFrameBusy=false;}
}
static String busaRainMakerTelemetryJSON(){
  const auto t=busaRmTrackerCopy();String s;s.reserve(1600);
  s="{\"interval_ms\":10000,\"mqtt_connected\":"+String(busaRmStarted&&esp_rmaker_is_mqtt_connected()?"true":"false");
  s+=",\"init_rc\":"+String((int)busaRmTelemetryInitRc)+",\"worker_pending\":"+String(busaRmFrameBusy.load()?"true":"false");
  s+=",\"snapshot_seq\":"+String(busaRmFrame.seq)+",\"snapshot_uptime_s\":"+String(busaRmFrame.uptime);
  s+=",\"snapshot_temperatures\":[";
  for(int i=0;i<4;i++){if(i)s+=",";s+=busaRmFrame.valid[i]?String(busaRmFrame.temperatures[i],2):String("null");}s+="]";
  s+=",\"report_calls\":"+String(busaRmReportCalls.load())+",\"report_api_ok\":"+String(busaRmReportApiOk.load());
  s+=",\"last_report_rc\":"+String(busaRmLastReportRc.load())+",\"last_transport_rc\":"+String(busaRmLastTransportRc.load());
  s+=",\"transport_failures\":"+String(busaRmTransportFailures.load())+",\"api_without_transport\":"+String(busaRmReportNoTransport.load());
  s+=",\"batch_enqueued\":"+String(t.enqueued)+",\"broker_confirmed_batches\":"+String(t.acked);
  s+=",\"last_enqueued_seq\":"+String(t.lastEnqueuedSeq)+",\"last_broker_ack_seq\":"+String(t.lastAckSeq);
  s+=",\"last_broker_ack_uptime_s\":"+String(t.lastAckUptime)+",\"last_broker_ack_ms\":"+String(t.lastAckAt);
  s+=",\"pending_acks\":"+String(t.outstanding())+",\"unconfirmed_batches\":"+String(t.unconfirmed);
  s+=",\"mqtt_connect_events\":"+String(t.connectedEvents)+",\"mqtt_disconnect_events\":"+String(t.disconnectedEvents);
  s+=",\"mqtt_published_events_all\":"+String(t.publishedEvents)+",\"mqtt_deleted_events_all\":"+String(t.deleted);
  s+=",\"queue_failures\":"+String(busaRmQueueFailures.load())+",\"write_queue_drops\":"+String(busaRmWriteQueueDrops.load());
  s+=",\"model_update_failures\":"+String(busaRmModelFailures.load())+",\"budget_deferrals\":"+String(busaRmBudgetDeferrals.load());
  s+=",\"stale_frames_dropped\":"+String(busaRmStaleFrames.load())+",\"cloud_app_delivery\":\"not_observed\"}";
  return s;
}
