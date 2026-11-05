"""Execute shipping grouped-settings callback; guard exact original schema union."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1];s=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
src=(s/'BUSA_RainMaker.h').read_text()
def function(sig):
 start=src.index(sig);pos=src.index('{',start)+1;depth=1
 while depth:
  if src[pos]=='{':depth+=1
  elif src[pos]=='}':depth-=1
  pos+=1
 return src[start:pos]
code=(root/'tests/fixtures/rm_sdk_mock.hpp').read_text().replace('struct FakeSerial{','struct FakeSerial{void println(const char*){}')
code+='\n'+src[src.index('static const char * const busaRmTempNames'):src.index('static const char * const RM_ON')]+src[src.index('struct BusaRmRequest {'):src.index('static QueueHandle_t busaRmQueue')]
code+=r'''
int busaRmWriteQueueDrops=0;void* busaRmQueue=(void*)1;std::vector<BusaRmRequest> requests;const int pdPASS=1;
int xQueueSend(void*,const BusaRmRequest* q,int){requests.push_back(*q);return pdPASS;}
'''+function('static bool busaRmFindSensor(')+'\n'+function('static void busaRmWrite(')+r'''
void send(const char* name,param_val_t v,int target,int action){
 Device wrapper("Настройки");Param p(name,nullptr,v,0);size_t before=requests.size();
 busaRmWrite(&wrapper,&p,v,nullptr,nullptr);assert(requests.size()==before+1);
 assert(requests.back().target==target&&requests.back().action==action);
}
int main(){
 send(RM_THEME,esp_rmaker_int(2),4,5);assert(requests.back().i==2);
 send(RM_SCREEN,esp_rmaker_bool(false),4,6);assert(!requests.back().b);
 send(RM_PAGE,esp_rmaker_int(7),4,7);assert(requests.back().i==7);
 send(RM_KEYLEARN,esp_rmaker_bool(true),5,8);
 send(RM_KEYSTOP,esp_rmaker_bool(true),5,9);
 Device wrapper("Настройки");Param p("Работает секунд",nullptr,esp_rmaker_int(0),0);
 busaRmWrite(&wrapper,&p,esp_rmaker_int(1),nullptr,nullptr);assert(requests.size()==5);
 Device sensor("T1 Вода");Param threshold(RM_ON,nullptr,esp_rmaker_float(35.7),0);
 busaRmWrite(&sensor,&threshold,esp_rmaker_float(35.7),nullptr,nullptr);
 assert(requests.back().target==0&&requests.back().action==0);
 assert(fabs(requests.back().f-35.7)<1e-4);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'settings.cpp';p.write_text(code);exe=Path(d)/'settings'
 subprocess.run(['g++','-std=c++11','-I'+str(s),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
start=function('static void busaRainMakerStart()')
assert start.count('new Device("Настройки"')==1
assert 'busaRmKeys=busaRmScreen;' in start and 'busaRmHealth=busaRmScreen;' in start
assert 'node.addDevice(*busaRmScreen)' not in start and 'node.addDevice(*busaRmKeys)' not in start
assert start.count('node.addDevice(*busaRmHealth)')==1
print('PASS grouped Settings: one registered device; theme/screen/page/learn/cancel queued correctly; readonly status ignored; temperature thresholds preserved')
