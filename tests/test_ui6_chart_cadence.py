"""Run the production browser polling function/timer without a network or wall-time wait."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1];s=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
a=s.index('async function refreshGraphHistory(force=false){');b=s.index('async function update()',a);poll=s[a:b]
timer=next(l for l in s.splitlines() if l.startswith('setInterval(()=>{if(active===3'))
visible=next(l for l in s.splitlines() if l.startswith("document.addEventListener('visibilitychange'"))
js=r'''
const assert=require('node:assert/strict');
let now=0,active=3,chartMinutes=5,chartFetchToken=0,chartRequestController=null,busaGraphFetchedAt=0,busaGraphData={},calls=[],draws=0,hanging=false;
const document={hidden:false,addEventListener:(event,fn)=>document.listener=fn};
const state={textContent:''};const get=()=>state;const graphs=()=>draws++;
Date.now=()=>now;
let tick,interval;const setInterval=(fn,ms)=>{tick=fn;interval=ms;};const setTimeout=()=>1;const clearTimeout=()=>{};
function fetch(url,options){
 const minutes=Number(url.split('=')[1]);calls.push({at:now,minutes,signal:options.signal});
 if(hanging)return new Promise((resolve,reject)=>options.signal.addEventListener('abort',()=>reject(Error('aborted'))));
 return Promise.resolve({ok:true,json:async()=>({ok:true,minutes,temperature:[[],[],[],[]]})});
}
'''+re.search(r'const CHART_REFRESH_MS=\d+;',s).group()+'\n'+poll+'\n'+timer+'\n'+visible+r'''
async function flush(){await new Promise(setImmediate);}
(async()=>{
 assert.equal(interval,2000);
 for(const minutes of [5,10,15,30,60]){
  chartMinutes=minutes;
  for(let i=0;i<3;i++){now+=2000;const before=calls.length;tick();await flush();assert.equal(calls.length,before+1);assert.equal(calls.at(-1).minutes,minutes);assert.equal(busaGraphData.minutes,minutes);}
 }
 for(let i=1;i<15;i++)assert.equal(calls[i].at-calls[i-1].at,2000);
 document.hidden=true;let count=calls.length;now+=2000;tick();await flush();assert.equal(calls.length,count);
 document.hidden=false;document.listener();await flush();assert.equal(calls.length,count+1);
 active=0;count=calls.length;now+=2000;tick();await flush();assert.equal(calls.length,count);
 active=3;hanging=true;now+=2000;tick();await flush();const old=calls.at(-1);count=calls.length;
 now+=2000;tick();await flush();assert.equal(calls.length,count); // no overlapping ordinary poll
 hanging=false;chartMinutes=10;await refreshGraphHistory(true);await flush();assert.equal(old.signal.aborted,true);assert.equal(busaGraphData.minutes,10);
 console.log('PASS fixed 2s browser chart cadence across all ranges; status-independent, hidden-tab pause/resume, no overlapping poll, forced range-change cancellation');
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'cadence.js';p.write_text(js);subprocess.run(['node',str(p)],check=True)
