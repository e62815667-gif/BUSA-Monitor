const {test}=require('node:test'),assert=require('node:assert/strict'),C=require('../../lab-core.js'),fs=require('node:fs');
const frame=()=>({device:'BUSA-1M-001',version:'v0.7.2-lab.1',mode:'MONITOR_ONLY',outputs_enabled:false,simulation:false,bridge_online:true,keypad_online:false,key_stream_id:'abc',temperatures:[25,26,27,28],channels:Array.from({length:4},()=>({valid:true,age_ms:100}))});
test('valid four real channels',()=>assert.equal(C.telemetry(frame()).temperatures[3],28));
for(const [name,change] of [
 ['demo',d=>d.simulation=true],['outputs',d=>d.outputs_enabled=true],
 ['stale',d=>d.channels[0].age_ms=3001],['bridge',d=>d.bridge_online=false],
 ['NaN',d=>d.temperatures[0]=NaN],['range',d=>d.temperatures[0]=126],
 ['invalid value leak',d=>d.channels[0].valid=false]])test('reject '+name,()=>{const d=frame();change(d);assert.throws(()=>C.telemetry(d));});
test('null missing sensor',()=>{const d=frame();d.channels[0].valid=false;d.temperatures[0]=null;assert.doesNotThrow(()=>C.telemetry(d));});
test('nullable settings and decimal comma',()=>{
 const s={revision:0,warning:[null,-55,0,125],automation:[null,null,null,null],volume_l:null,physical_outputs:false};
 assert.doesNotThrow(()=>C.settings(s));assert.equal(C.parseInput(' 24,5 ',-55,125),24.5);assert.equal(C.parseInput('',-55,125),null);
 s.warning[0]=true;assert.throws(()=>C.settings(s));assert.throws(()=>C.parseInput('abc',-55,125));
});
test('no physical actuator API/GPIO or peripheral writes in entrypoint',()=>{
 const s=fs.readFileSync('firmware/v0.7.2/main/LabMain.cpp','utf8');
 assert.match(s,/constexpr bool PHYSICAL_OUTPUTS=false/);
 assert.doesNotMatch(s,/digitalWrite|pinMode|analogWrite|ledcWrite|pcfWrite|\/api\/relay/);
 assert.match(s,/bool armed=false/);assert.match(s,/Wire.begin\(8,7,100000\)/);
});
test('no cached telemetry',()=>{const s=fs.readFileSync('sw.js','utf8');assert.match(s,/\/api\//);});

