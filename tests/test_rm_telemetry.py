"""Compile actual telemetry code against a deterministic SDK model; no ESP/cloud test."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
mock=(root/'tests/fixtures/rm_sdk_mock.hpp').read_text()
old=(root/'tests/fixtures/rm_fix3_poll.cpp').read_text()
new=(sketch/'BUSA_RainMaker.h').read_text().split('static void busaRainMakerPoll(')[-1]
new='static void busaRainMakerPoll('+new
baseline=r'''
uint32_t busaRmLastSent=0,busaRmLastSettingsSent=0;uint8_t busaRmSettingsCursor=0;
'''+old+r'''
int main(){initialize();for(uint32_t t=100;t<=600000;t+=100){advance(t);temperature[2]=float(t)/10000;busaRainMakerPoll(t);}
 assert(drops>500);auto* p=health.getParamByName("Работает секунд");
 assert(cloud.count(p));assert(cloud[p].val.i<400);
 printf("FIX3 model: dropped=%d last uptime=%d at 600s\n",drops,cloud[p].val.i);
}
'''
fixed=r'''
#include "BUSA_RainMakerTelemetry.h"
'''+new+r'''
void tick(uint32_t at){advance(at);int before=wireCalls;busaRainMakerPoll(at);assert(wireCalls==before);runWorker();}
int main(){initialize();busaRmTelemetryInit();assert(busaRmTelemetryInitRc==ESP_OK);
 events(nullptr,RMAKER_COMMON_EVENT,RMAKER_MQTT_EVENT_CONNECTED,nullptr);
 tick(10000);assert(wireCalls==1&&busaRmTrackerCopy().acked==1);
 auto* temp=tempDevices[2].getParamByName(ESP_RMAKER_DEF_TEMPERATURE_NAME);
 auto* setting=tempDevices[0].getParamByName(RM_ON);int updates=setting->updates;
 assert(fabs(cloud[temp].val.f-24.81f)<0.01f);
 temperature[2]=29.69f;tick(20000);assert(wireCalls==2&&fabs(cloud[temp].val.f-29.69f)<0.01f);
 assert(setting->updates==updates);
 busaGoogleNetworkBusy=true;
 for(uint32_t t=30000;t<=3600000;t+=10000)tick(t);
 assert(wireCalls==360&&drops==0&&busaRmTrackerCopy().acked==360);
 assert(cloud[health.getParamByName("Работает секунд")].val.i==3600);
 // SDK API falsely succeeds without reaching transport. Cache must be restaged.
 silentDrop=true;refTemp[0]=47.5f;tick(3610000);
 assert(busaRmReportNoTransport==1&&busaRmTrackerCopy().acked==360);
 silentDrop=false;tick(3620000);assert(cloud[setting].val.f==47.5f);
 backendFail=true;tick(3630000);assert(busaRmTransportFailures==1);backendFail=false;
 tick(3640000);assert(busaRmTrackerCopy().lastAckUptime==3640);
 queueFail=true;tick(3650000);assert(busaRmQueueFailures==1&&!busaRmFrameBusy);queueFail=false;
 connected=false;events(nullptr,RMAKER_COMMON_EVENT,RMAKER_MQTT_EVENT_DISCONNECTED,nullptr);
 int count=wireCalls;tick(3660000);assert(wireCalls==count);
 connected=true;events(nullptr,RMAKER_COMMON_EVENT,RMAKER_MQTT_EVENT_CONNECTED,nullptr);
 tick(3670000);assert(wireCalls==count+1);
 otaActive=true;tick(3680000);assert(wireCalls==count+1);otaActive=false;
 busaWifiScanRunning=true;tick(3690000);assert(wireCalls==count+1);busaWifiScanRunning=false;
 valid[2]=false;temperature[2]=NAN;tick(3700000);
 assert(!cloud[tempDevices[2].getParamByName("Датчик подключен")].val.b);
 assert(fabs(cloud[temp].val.f-29.69f)<0.01f); // no invented zero/NaN sample
 valid[2]=true;temperature[2]=31;
 advance(3710000);busaRainMakerPoll(nowMs);assert(queued);advance(3740000);runWorker();assert(busaRmStaleFrames==1);
 tokens=0;tick(3740001);assert(busaRmBudgetDeferrals>0);tokens=20;tick(3750001);
 assert(cloud[temp].val.f==31);
 // A foreign PUBACK never confirms this application's frame.
 autoAck=false;tick(3760001);auto a=busaRmTrackerCopy();int foreign=99999;
 events(nullptr,RMAKER_COMMON_EVENT,RMAKER_MQTT_EVENT_PUBLISHED,&foreign);assert(busaRmTrackerCopy().acked==a.acked);
 assert(busaRmTrackerCopy().outstanding()==1);
 puts(busaRainMakerTelemetryJSON().c_str());
}
'''
with tempfile.TemporaryDirectory() as d:
 d=Path(d);(d/'freertos').mkdir()
 for h in ['esp_rmaker_mqtt.h','esp_rmaker_common_events.h','esp_rmaker_work_queue.h','freertos/task.h']:(d/h).write_text('')
 for name,body in [('baseline',baseline),('fixed',fixed)]:
  p=d/(name+'.cpp');p.write_text(mock+'\n'+body);exe=d/name
  subprocess.run(['g++','-std=c++11','-I'+str(d),'-I'+str(sketch),str(p),'-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
print('PASS actual telemetry: 1h cadence/budget, batching, settings cache, real/invalid samples, API-vs-transport, queue, reconnect, OTA/scan pause, stale frame, foreign PUBACK')
