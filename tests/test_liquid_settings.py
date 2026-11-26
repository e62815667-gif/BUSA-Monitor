"""Regression for separate liquid settings, using the real UART contract, HTTP handler and browser save code."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
main=(fw/'BUSA_Main.h').read_text()
ui=(root/'firmware/BUSA_Display_N4R8/BUSA_TouchUI.h').read_text()
def function(s, signature):
    a=s.index(signature); p=s.index('{',a)+1; depth=1
    while depth:
        if s[p]=='{': depth+=1
        elif s[p]=='}': depth-=1
        p+=1
    return s[a:p]
water=ui[ui.index('    void water(){'):ui.index('    void graphAxes(')]
assert 'Израсходовано' in water and 'fields_' not in water and 'Импульс' not in water and water.count('button(')==2
assert 'button(q,"Сбросить собрано",224)' in water and 'button(p,"Сбросить израсходовано",223)' in water
assert 'sub_==9' in ui and 'Route::collectionCoefficient' in ui and 'Route::waterCoefficient' in ui
assert 'collectionConfig_?222:201' in ui
html=main.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
monitor=html.split('<section class="page" data-section="2"',1)[1].split('</section>',1)[0]
assert 'Израсходовано' in monitor and 'Импульс' not in monitor and '<input' not in monitor
assert 'id="waterResetTotal"' in monitor and 'id="collectionResetTotal"' in monitor
settings=html.split('<div class="card settingCard" data-stab="9"',1)[1].split('<div class="settingCard" data-stab="6"',1)[0]
for key in ['waterPplInput','waterPplSave','waterPplPassport','collectionPplInput','collectionPplSave']:
    assert 'id="'+key+'"' in settings and html.count('id="'+key+'"')==1
assert 'data-liquid-panel="collection"' in settings and 'data-liquid-panel="water" hidden' in settings
assert 'case busa_uart::ApiRoute::collectionCoefficient:busaApi_collectionCoefficient(local);break;' in main
assert 'server.on("/api/collection/calibration",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_collectionCoefficient(request);});' in main
code=r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <algorithm>
#include "BUSA_Collection.h"
#include "BUSA_WaterFlowMath.h"
class String: public std::string{
 public:using std::string::string;String(const std::string& s):std::string(s){}
 void trim(){auto a=find_first_not_of(" \r\n\t");if(a==npos){clear();return;}*this=substr(a,find_last_not_of(" \r\n\t")-a+1);}
 void replace(char a,char b){std::replace(begin(),end(),a,b);}
 bool isEmpty()const{return empty();}
};
struct BusaLocalRequest{
 bool allowed=true;String confirm="YES",value="4.71",body;int status=0;
 bool local()const{return allowed;}
 String target="water";
 String arg(const char* k){return std::string(k)=="confirm"?confirm:std::string(k)=="target"?target:value;}
 void send(int c,const char*,const String& b){status=c;body=b;}
};
BusaCollection::Meter collectionMeter;
BusaWater::VolumeAccumulator waterVolume;
bool otaActive=false;
namespace busa_display_ota {bool busy(){return false;}}
int flowMux=0;
uint32_t pulseCounter=100,flowRejectedCounter=2,flowLastCount=90,flowLastRejected=1,flowCycle=10;
uint32_t millis(){return 2000;}
void portENTER_CRITICAL(int*){} void portEXIT_CRITICAL(int*){}
uint64_t collectionPulses(){return 40;}
void busaWebRecord(const String&){}
bool busaNvsReady=true;
struct Preferences{
 float factor=NAN,water=4710;bool fail=false;unsigned writes=0;
 float getFloat(const char* key,float fallback){assert(std::string(key)=="collPulseMl");return std::isfinite(factor)?factor:fallback;}
 size_t putFloat(const char* key,float f){assert(std::string(key)=="collPulseMl");if(fail)return 0;factor=f;++writes;return sizeof(float);}
}prefs;
'''
code+=function(main,'static String collectionNumber(')+'\n'+function(main,'static void loadCollectionCalibration(')+'\n'+function(main,'static bool saveCollectionCalibration(')+'\n'+function(main,'static void busaApi_collectionCoefficient(')+'\n'+function(main,'static void busaApi_resetVolume(')+r'''
int main(){
 BusaLocalRequest q;
 q.allowed=false;busaApi_collectionCoefficient(q);assert(q.status==403&&prefs.writes==0);
 q.allowed=true;q.confirm="NO";busaApi_collectionCoefficient(q);assert(q.status==403&&prefs.writes==0);
 q.confirm="YES";
 for(const char* invalid:{"","0","-1","nan","inf","1x"}){q.value=invalid;busaApi_collectionCoefficient(q);assert(q.status==400&&prefs.writes==0);}
 q.value="4,71";busaApi_collectionCoefficient(q);assert(q.status==200&&prefs.writes==1&&fabs(collectionMeter.calibration()-4.71)<0.00001&&prefs.water==4710);
 collectionMeter=BusaCollection::Meter();loadCollectionCalibration();assert(fabs(collectionMeter.calibration()-4.71)<0.00001);
 prefs.fail=true;q.value="5";busaApi_collectionCoefficient(q);assert(q.status==500&&prefs.writes==1&&fabs(collectionMeter.calibration()-4.71)<0.00001);
 prefs.fail=false;q.confirm="YES";q.allowed=true;
 collectionMeter.begin(0);collectionMeter.sample(2000,40);const double before=collectionMeter.total();
 waterVolume.record(4710,0,true,4710);assert(waterVolume.countedLitres==1);
 q.allowed=false;busaApi_resetVolume(q);assert(q.status==403&&waterVolume.countedLitres==1);
 q.allowed=true;q.confirm="NO";busaApi_resetVolume(q);assert(q.status==403&&waterVolume.countedLitres==1);
 q.confirm="YES";otaActive=true;busaApi_resetVolume(q);assert(q.status==409&&waterVolume.countedLitres==1);
 otaActive=false;q.target="bad";busaApi_resetVolume(q);assert(q.status==400&&waterVolume.countedLitres==1);
 q.target="water";busaApi_resetVolume(q);assert(q.status==200&&waterVolume.countedLitres==0&&collectionMeter.total()==before&&flowLastCount==100&&flowLastRejected==2&&flowCycle==2000);
 waterVolume.record(471,0,true,4710);q.target="collection";busaApi_resetVolume(q);assert(q.status==200&&collectionMeter.total()==0&&fabs(waterVolume.countedLitres-.1)<.0001&&fabs(collectionMeter.calibration()-4.71)<.0001);
 collectionMeter.sample(4000,45);assert(fabs(collectionMeter.total()-5/4.71)<.0001);
 std::puts("PASS collection configuration, reset isolation, confirmation, busy rejection, retained calibration and next pulses");
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'liquid.cpp';p.write_text(code);exe=Path(d)/'liquid'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(fw),str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
a=main.index('async function saveWaterPpl(')
b=main.index("get('tempCalSave').onclick",a)
js=r'''
const assert=require('node:assert/strict'), nodes={},calls=[];
const get=id=>nodes[id]??=( {value:'',textContent:'',disabled:false});
let waterPplInitialized=false,collectionPplInitialized=false,failed=false,approved=true;
const window={confirm:()=>approved};
const fetch=async (url,opts)=>{
 calls.push([url,Object.fromEntries(opts.body)]);
 if(failed)return {ok:false,text:async()=>'NVS'};
 return {ok:true,json:async()=>url.includes('/volume/reset')?{ok:true}:url.includes('/water/')?{pulses_per_litre:4710}:{pulses_per_ml:4.71}};
};
'''+main[a:b]+r'''
(async()=>{
 get('collectionPplInput').value='4.71';await get('collectionPplSave').onclick();
 assert.deepEqual(calls[0],['/api/collection/calibration',{confirm:'YES',pulses_per_ml:'4.71'}]);
 assert.equal(get('collectionPplInput').value,'4.71');
 get('waterPplInput').value='4710';await saveWaterPpl('4710');
 assert.deepEqual(calls[1],['/api/water/coefficient',{confirm:'YES',pulses_per_litre:'4710'}]);
 assert.equal(get('waterPplInput').value,'4710.0');
 get('collectionPplInput').value='0';await get('collectionPplSave').onclick();assert.equal(calls.length,2);
 failed=true;get('collectionPplInput').value='5';await get('collectionPplSave').onclick();
 assert.match(get('collectionPplResult').textContent,/NVS/);
 assert.equal(get('collectionPplSave').disabled,false);
 failed=false;approved=false;const n=calls.length;await get('waterResetTotal').onclick();assert.equal(calls.length,n);
 approved=true;await get('waterResetTotal').onclick();assert.deepEqual(calls[n],['/api/volume/reset',{confirm:'YES',target:'water'}]);
 await get('collectionResetTotal').onclick();assert.deepEqual(calls[n+1],['/api/volume/reset',{confirm:'YES',target:'collection'}]);assert.equal(get('collectionResetResult').textContent,'Объём сброшен');
 failed=true;await get('collectionResetTotal').onclick();assert.match(get('collectionResetResult').textContent,/NVS/);assert.equal(get('collectionResetTotal').disabled,false);
 console.log('PASS actual local browser independent reset/save, cancellation and error recovery');
})().catch(e=>{console.error(e);process.exit(1)});
'''
subprocess.run(['node','-e',js],check=True)
# Live rate is an ESP32 measurement, not a derivative of rounded cumulative volume.
rate_js=r'''
const assert=require('node:assert/strict'),nodes={};
const get=id=>nodes[id]??=({textContent:'',classList:{toggle(){}}});
const decimal1=new Intl.NumberFormat('ru-RU',{minimumFractionDigits:1,maximumFractionDigits:1});
let previousWater=null;
'''+main[main.index('function measured('):main.index('function renderHomeThresholds(')]+r'''
const water={credible:true,seen_pulses:true,calibrated:true,total_l_estimate:1.457,lpm:0.639};
const sample=(overrides={},uptime=10000)=>updateLiquidPanels({water:{...water,...overrides},uptime_ms:uptime});
const rate=(value)=>{assert.equal(get('coolingLph').textContent,value);assert.equal(get('homeCoolingRate').textContent,value+' л/ч');};
sample();rate('38,3');
sample({total_l_estimate:1.457},10001);rate('38,3');
sample({total_l_estimate:3.12,lpm:0.711},10002);rate('42,7');
sample({total_l_estimate:0},10003);rate('38,3');assert.equal(get('coolingTotal').textContent,'0,0');
sample({lpm:0},10004);rate('0,0');
for(const v of [null,undefined,-1,NaN,Infinity,'0.639']){sample({lpm:v});rate('—');}
updateLiquidPanels(null);rate('—');
sample({},1);rate('38,3');
console.log('PASS ESP32 rate on first sample, arbitrary polling intervals, reset, zero, invalid values and reconnect');
'''
subprocess.run(['node','-e',rate_js],check=True)
