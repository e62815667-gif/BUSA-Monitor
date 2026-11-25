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
assert 'Израсходовано' in water and 'fields_' not in water and 'Импульс' not in water and 'button(' not in water
assert 'sub_==9' in ui and 'Route::collectionCoefficient' in ui and 'Route::waterCoefficient' in ui
assert 'collectionConfig_?222:201' in ui
html=main.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
monitor=html.split('<section class="page" data-section="2"',1)[1].split('</section>',1)[0]
assert 'Израсходовано' in monitor and 'Импульс' not in monitor and '<input' not in monitor
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
class String: public std::string{
 public:using std::string::string;String(const std::string& s):std::string(s){}
 void trim(){auto a=find_first_not_of(" \r\n\t");if(a==npos){clear();return;}*this=substr(a,find_last_not_of(" \r\n\t")-a+1);}
 void replace(char a,char b){std::replace(begin(),end(),a,b);}
 bool isEmpty()const{return empty();}
};
struct BusaLocalRequest{
 bool allowed=true;String confirm="YES",value="4.71",body;int status=0;
 bool local()const{return allowed;}
 String arg(const char* k){return std::string(k)=="confirm"?confirm:value;}
 void send(int c,const char*,const String& b){status=c;body=b;}
};
BusaCollection::Meter collectionMeter;
bool busaNvsReady=true;
struct Preferences{
 float factor=NAN,water=4710;bool fail=false;unsigned writes=0;
 float getFloat(const char* key,float fallback){assert(std::string(key)=="collPulseMl");return std::isfinite(factor)?factor:fallback;}
 size_t putFloat(const char* key,float f){assert(std::string(key)=="collPulseMl");if(fail)return 0;factor=f;++writes;return sizeof(float);}
}prefs;
'''
code+=function(main,'static String collectionNumber(')+'\n'+function(main,'static void loadCollectionCalibration(')+'\n'+function(main,'static bool saveCollectionCalibration(')+'\n'+function(main,'static void busaApi_collectionCoefficient(')+r'''
int main(){
 BusaLocalRequest q;
 q.allowed=false;busaApi_collectionCoefficient(q);assert(q.status==403&&prefs.writes==0);
 q.allowed=true;q.confirm="NO";busaApi_collectionCoefficient(q);assert(q.status==403&&prefs.writes==0);
 q.confirm="YES";
 for(const char* invalid:{"","0","-1","nan","inf","1x"}){q.value=invalid;busaApi_collectionCoefficient(q);assert(q.status==400&&prefs.writes==0);}
 q.value="4,71";busaApi_collectionCoefficient(q);assert(q.status==200&&prefs.writes==1&&fabs(collectionMeter.calibration()-4.71)<0.00001&&prefs.water==4710);
 collectionMeter=BusaCollection::Meter();loadCollectionCalibration();assert(fabs(collectionMeter.calibration()-4.71)<0.00001);
 prefs.fail=true;q.value="5";busaApi_collectionCoefficient(q);assert(q.status==500&&prefs.writes==1&&fabs(collectionMeter.calibration()-4.71)<0.00001);
 std::puts("PASS collection handler, AP/confirm, invalid input, comma, NVS failure, reload and water isolation");
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
let waterPplInitialized=false,collectionPplInitialized=false,failed=false;
const fetch=async (url,opts)=>{
 calls.push([url,Object.fromEntries(opts.body)]);
 if(failed)return {ok:false,text:async()=>'NVS'};
 return {ok:true,json:async()=>url.includes('/water/')?{pulses_per_litre:4710}:{pulses_per_ml:4.71}};
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
 console.log('PASS real browser save functions, independent endpoints and error recovery');
})().catch(e=>{console.error(e);process.exit(1)});
'''
subprocess.run(['node','-e',js],check=True)
