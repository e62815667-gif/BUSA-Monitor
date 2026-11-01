const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('online/index.html','utf8');
function fn(name){const start=html.indexOf('function '+name+'(');assert(start>=0);let open=html.indexOf('{',start),depth=1,i=open+1;while(depth){if(html[i]==='{')depth++;if(html[i]==='}')depth--;i++;}return html.slice(start,i);}
let nodes={};const context={document:{getElementById:id=>nodes[id]??=( {textContent:''})},stampRussian:()=>'',Number,Date,cloudRelease:{version:'V15.3',build_flash_bytes:16777216},cloudCurrentTelemetry:{firmware:'БУСА 1М WiFi 15v3-N16R8-UNIFIED-UI-BOOTFIX1'},cloudCurrentAt:Date.now()};
vm.createContext(context);
vm.runInContext(fn('renderCloudCompatibility')+'\nrenderCloudCompatibility()',context);
assert.equal(nodes.cloudVersionRelation.textContent,'Установлена версия из каталога');
context.cloudRelease.version='V14.27';vm.runInContext('renderCloudCompatibility()',context);assert.equal(nodes.cloudVersionRelation.textContent,'Установленная версия новее каталога');
vm.runInContext(fn('verifiedTemperature'),context);
assert.equal(vm.runInContext('verifiedTemperature({verified:false,c:85})',context),null);
assert.equal(vm.runInContext('verifiedTemperature({verified:true,c:85})',context),85);
assert.equal(vm.runInContext('verifiedTemperature({verified:true,c:126})',context),null);
assert(html.includes('Звуковая сигнализация') && html.includes('15.3'));
const main=fs.readFileSync('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h','utf8');
for(const [name,text] of [['online',html],['local',main]])for(const m of text.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g))new vm.Script(m[1],{filename:name});
console.log('PASS: V15.3 catalogue comparison, verified temperatures including 85C, both JS bundles');
