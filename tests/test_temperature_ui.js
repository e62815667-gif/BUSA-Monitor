const fs=require('fs'),vm=require('vm'),assert=require('assert');
const src=fs.readFileSync('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h','utf8');
const a=src.indexOf('const temperatureErrors='),b=src.indexOf('let bindingFingerprint=',a);
const context={};vm.createContext(context);vm.runInContext(src.slice(a,b),context);
for(const t of [{c:25,verified:false},{c:null,verified:true},{c:'25',verified:true},{c:NaN,verified:true},{c:126,verified:true}]){
 const v=context.temperatureView({...t,gpio:17,error:'MULTIPLE_ROM'});assert(!v.valid&&v.value===null&&v.text==='Н/Д');
}
for(const c of [-55,-12.5,0,85,125])assert(context.temperatureView({c,verified:true,gpio:16}).valid);
assert(context.temperatureView({c:null,verified:false,gpio:18,error:'NO_PRESENCE'}).detail.includes('Нет ответа'));
const fields={};for(let i=0;i<4;i++){fields['t'+i]={textContent:'26',classList:{add(){}}};fields['ts'+i]={textContent:'OK'};}
fields.lastTUpdate={};fields.temperatures={};fields.temperatureDetails={};fields.temperatureInventory={};fields.dataLed={classList:{remove(){}}};
const lost=src.slice(src.indexOf('function temperatureConnectionLost(){'),src.indexOf("for(let i=0;i<4;i++)get('bind'",src.indexOf('function temperatureConnectionLost(){')));
context.get=id=>fields[id];vm.runInContext(lost,context);context.temperatureConnectionLost();
assert.equal(fields.t0.textContent,'НЕТ СВЯЗИ');assert(fields.lastTUpdate.textContent.includes('потеряна'));
assert(src.includes("if(j.saved!==true)throw Error('Нет подтверждения сохранения')"));
const html=src.match(/R"HTMLBUSA\(([\s\S]*?)\)HTMLBUSA"/)[1];
const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(x=>x[1]);assert.equal(new Set(ids).size,ids.length,'duplicate DOM id');
for(const id of ['saveBindings','temperatureInventory','temperatureDetails','lastTUpdate',...Array.from({length:4},(_,i)=>'bind'+i)])assert(ids.includes(id),id);
console.log('PASS: actual app helpers: verified finite readings only, real 85 C, error messages, lost-link clearing, binding confirmation and DOM ids');

for(const error of ['LINE_LOW','NO_PRESENCE','MULTIPLE_ROM','STALE_READING','UNASSIGNED','ROM_ASSIGNED_MULTIPLE_ROLES']){
 const v=context.temperatureView({c:null,verified:false,gpio:17,error});assert(v.detail&&!v.valid);
}
assert(src.includes("new Option('Не назначен','none')"));
for(let i=0;i<4;i++){
 const on=html.match(new RegExp('<label>([^<]+)</label><input[^>]+id="ref'+i+'"'))[1];
 const off=html.match(new RegExp('<label>([^<]+)</label><input[^>]+id="nt'+i+'"'))[1];
 assert(on.includes('ВКЛ, °C')&&off.includes('ОТКЛ, °C'));
 assert(!/Звук|Сигнализация/i.test(on+off));
 assert(new RegExp('id="alarmEnable'+i+'"[^>]*disabled').test(html));
}
assert(/id="alarmSave"[^>]*disabled/.test(html));
console.log('PASS SAFE1 UI: independent ON/OFF labels, unassigned and electrical diagnostics, physical controls disabled');

const landscape=fs.readFileSync('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_LandscapeSafe.h','utf8');
assert(landscape.includes('ВКЛ, °C: ')&&landscape.includes('ОТКЛ, °C: '));
for(const state of ['LOW','NO DS','ROM?','STALE','UNSET'])assert(landscape.includes('"'+state+'"'));
assert(src.includes('#include "BUSA_V15_LandscapeSafe.h"'));
