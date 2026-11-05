"""Execute native RainMaker schema/callback/math/batch code against SDK mocks."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
mock=(root/'tests/fixtures/rm_sdk_mock.hpp').read_text()
code=mock+r'''
struct BusaRmRequest{uint8_t target,action,role;float f;int i;bool b;};
void* busaRmQueue=(void*)1;std::vector<BusaRmRequest> requests;bool writeQueueFull=false;
const int pdPASS=1;
int xQueueSend(void*,const BusaRmRequest* q,int){if(writeQueueFull)return 0;requests.push_back(*q);return pdPASS;}
#include "BUSA_RainMakerTelemetry.h"
#include "BUSA_RainMakerCalc.h"
void deliver(int mode,const char* name,param_val_t value){Param param(name,nullptr,value,0);Device wrapper;wrapper.setDeviceHandle(busaRmCalcDevices[mode]->getDeviceHandle());assert(&wrapper!=busaRmCalcDevices[mode]);busaRmCalcWrite(&wrapper,&param,value,nullptr,nullptr);}
void drain(){for(const auto& q:requests){assert(q.target>=6&&q.target<9);busaRmCalcApply(q.target-6,q.action,q.role,q.f,q.b);}requests.clear();}
void input(int m,int field,float v){deliver(m,busaRmCalcInputNames[m][field],esp_rmaker_float(v));drain();}
void calculate(int m){deliver(m,busaRmCalcActionNames[m][0],esp_rmaker_bool(true));drain();}
void tick(uint32_t t){advance(t);int before=wireCalls;busaRmCaptureAndQueue(t);assert(wireCalls==before);runWorker();}
int main(){
 initialize();Node node;busaRmAddCalculators(node);assert(node.devices.size()==1);
 assert(busaRmCalcReady[0]&&busaRmCalcReady[1]&&busaRmCalcReady[2]);
 assert(node.devices[0]->params.size()==29);
 assert(busaRmCalcDevices[0]==busaRmCalcDevices[1]&&busaRmCalcDevices[1]==busaRmCalcDevices[2]);
 assert(node.devices[0]->deviceName=="Калькулятор");
 for(int m=0;m<3;m++){
  for(int i=0;i<busaRmCalcInputCounts[m];i++){auto* p=busaRmCalcDevices[m]->getParamByName(busaRmCalcInputNames[m][i]);assert(p&&p->value.type==RMAKER_VAL_TYPE_FLOAT&&(p->flags&PROP_FLAG_WRITE));}
  for(int i=0;i<busaRmCalcOutputCounts[m];i++){auto* p=busaRmCalcDevices[m]->getParamByName(busaRmCalcOutputNames[m][i]);assert(p&&p->value.type==RMAKER_VAL_TYPE_STRING&&!(p->flags&PROP_FLAG_WRITE));}
  assert(std::string(busaRmCalculators[m].outputs[0])=="—");
 }
 auto before=busaRmCalculators[0].revision;deliver(0,busaRmCalcInputNames[0][0],esp_rmaker_float(1));
 assert(requests.size()==1&&busaRmCalculators[0].revision==before);drain();
 input(0,1,96);input(0,2,40);calculate(0);
 assert(std::string(busaRmCalculators[0].outputs[0])=="1,47 л");assert(std::string(busaRmCalculators[0].outputs[1])=="2,40 л");
 input(1,0,30);input(1,1,40);calculate(1);assert(std::string(busaRmCalculators[1].outputs[0])=="35,98 % об.");
 for(int i=0;i<5;i++)input(2,i,(float[]){10,40,80,10,10}[i]);calculate(2);
 assert(std::string(busaRmCalculators[2].outputs[0])=="4,00 л АС");assert(std::string(busaRmCalculators[2].outputs[3])=="4,00 л");
 busaRmTelemetryInit();tick(10000);assert(wireCalls==1&&busaRmTrackerCopy().acked==1);
 auto* output=busaRmCalcDevices[0]->getParamByName("Долить воды");auto* field=busaRmCalcDevices[0]->getParamByName(busaRmCalcInputNames[0][0]);int updates=field->updates;
 assert(std::string(cloud[output].val.s)=="1,47 л");tick(20000);assert(wireCalls==2&&field->updates==updates);
 input(0,0,2);assert(std::string(busaRmCalculators[0].outputs[0])=="—");calculate(0);
 silentDrop=true;tick(30000);assert(busaRmReportNoTransport==1);assert(std::string(cloud[output].val.s)=="1,47 л");
 silentDrop=false;tick(40000);assert(std::string(cloud[output].val.s)=="2,94 л");
 // Late writes cannot mutate an already captured frame; next snapshot receives them.
 advance(50000);busaRmCaptureAndQueue(nowMs);input(0,0,3);calculate(0);runWorker();assert(std::string(cloud[output].val.s)=="2,94 л");
 tick(60000);assert(std::string(cloud[output].val.s)=="4,41 л");
 before=busaRmCalculators[0].revision;deliver(0,busaRmCalcInputNames[0][0],esp_rmaker_bool(true));assert(requests.empty()&&busaRmCalculators[0].revision==before);
 writeQueueFull=true;deliver(0,busaRmCalcActionNames[0][0],esp_rmaker_bool(true));assert(requests.empty()&&busaRmWriteQueueDrops==1);writeQueueFull=false;
 input(0,1,NAN);assert(std::string(busaRmCalculators[0].outputs[0])=="—");assert(busaRmCalculators[0].inputs[1]==96);
 input(2,3,90);input(2,4,20);calculate(2);assert(std::string(busaRmCalculators[2].outputs[0])=="—");
 deliver(0,busaRmCalcActionNames[0][1],esp_rmaker_bool(true));drain();assert(busaRmCalculators[0].inputs[0]==0&&std::string(busaRmCalculators[0].outputs[0])=="—");
 // One unavailable calculator model must never suppress temperature publication.
 busaRmCalcReady[1]=false;tick(70000);assert(cloud[health.getParamByName("Работает секунд")].val.i==70);
 for(uint32_t t=80000;t<=3600000;t+=10000)tick(t);assert(wireCalls==359&&drops==1);
 assert(busaRmTrackerCopy().lastAckUptime==3600);
}
'''
# ISO C++ does not accept a C compound literal; use a real fixed array.
code=code.replace('for(int i=0;i<5;i++)input(2,i,(float[]){10,40,80,10,10}[i]);','float fractionInputs[5]={10,40,80,10,10};for(int i=0;i<5;i++)input(2,i,fractionInputs[i]);')
with tempfile.TemporaryDirectory() as d:
 d=Path(d);(d/'freertos').mkdir()
 for h in ['esp_rmaker_mqtt.h','esp_rmaker_common_events.h','esp_rmaker_work_queue.h','freertos/task.h']:(d/h).write_text('')
 p=d/'native.cpp';p.write_text(code);exe=d/'native'
 subprocess.run(['g++','-std=c++11','-I'+str(d),'-I'+str(sketch),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS native RainMaker: one card / three independent calculations, queued typed writes, actual calculations, invalid/reset, immutable frames, one batch/10s, unchanged-input suppression, retry, calc-failure isolation; no real cloud tested')
