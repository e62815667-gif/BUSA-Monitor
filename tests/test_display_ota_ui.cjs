// Execute the actual embedded update-tab script against a minimal DOM and API fixture.
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const path=require('path');
const source=fs.readFileSync(path.join(__dirname,'../firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h'),'utf8');
const script=[...source.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(x=>x[1]).find(x=>x.includes("const active=new Set(['receiving','ready','sending','waiting_boot'])"));
assert(script,'display OTA browser script missing');
for(const id of ['otaMainTab','otaScreenTab','otaMainPanel','otaScreenPanel','displayFirmwareFile','displayManualUpdate','displayOtaProgress','displayUpdateResult','displayOtaVersion']){
 assert.equal((source.match(new RegExp('id="'+id+'"','g'))||[]).length,1,id);
}
const elements={},parent={hidden:false};
const el=id=>elements[id]||(elements[id]={hidden:id==='otaScreenPanel',disabled:false,files:[],textContent:'',value:0,classList:{toggle(){}},setAttribute(){},closest(){return parent;}});
let status={capable:true,state:'idle',firmware:'BUSA-DISPLAY-N4R8-OTA1',size:0,confirmed:0};
let tick,requests=[],confirm=true;
class FormDataStub{append(){}}
class Xhr{
 constructor(){this.upload={};}
 open(method,url){requests.push([method,url]);}
 setRequestHeader(k,v){assert.equal(k,'X-Busa-Token');assert.equal(v,'token');}
 send(){status.state='ready';status.size=60000;this.status=200;this.onload();}
}
const context={document:{getElementById:el},setInterval(fn,ms){assert.equal(ms,2000);tick=fn;},
 fetch:async(url,options)=>{requests.push([options?.method||'GET',url]);return{ok:true,json:async()=>url.endsWith('/prepare')?{token:'token'}:status};},
 confirm:()=>confirm,FormData:FormDataStub,XMLHttpRequest:Xhr,URLSearchParams,Set,Math,String,Error};
vm.runInNewContext(script,context);
const flush=()=>new Promise(setImmediate);
(async()=>{
 el('otaScreenTab').onclick();await flush();
 assert.equal(el('otaMainPanel').hidden,true);assert.equal(el('otaScreenPanel').hidden,false);
 assert.equal(el('displayManualUpdate').disabled,false);
 await el('displayManualUpdate').onclick();assert.match(el('displayUpdateResult').textContent,/BIN/);
 el('displayFirmwareFile').files=[{name:'DISPLAY_OTA.bin',size:60000}];
 await el('displayManualUpdate').onclick();await flush();
 assert(requests.some(x=>x[1]==='/api/display-ota/prepare'));
 assert(requests.some(x=>x[1]==='/api/display-ota/upload'));
 assert.equal(el('displayManualUpdate').disabled,true);
 status.state='waiting_boot';status.confirmed=60000;await tick();assert.match(el('displayUpdateResult').textContent,/Проверка запуска/);
 assert.equal(el('displayOtaProgress').value,100);
 status.state='success';await tick();assert.match(el('displayUpdateResult').textContent,/успешно обновлён/);
 status.state='error';await tick();assert.match(el('displayUpdateResult').textContent,/не подтверждено/);
 el('otaMainTab').onclick();assert.equal(el('otaScreenPanel').hidden,true);assert.equal(el('otaMainPanel').hidden,false);
 console.log('PASS actual update-tab JS: separate target, upload route, progress, boot confirmation, error and navigation');
})().catch(e=>{console.error(e);process.exitCode=1;});
