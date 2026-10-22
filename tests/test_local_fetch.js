const fs=require('fs'),vm=require('vm'),assert=require('assert');
const main=fs.readFileSync('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h','utf8');
const helper=main.slice(main.indexOf('async function localFetch('),main.indexOf('\n}',main.indexOf('async function localFetch('))+2);
async function run(stallBody){
 let aborted=false;
 const ctx={AbortController,Response,clearTimeout,setTimeout:(fn)=>setTimeout(fn,10),
  fetch:async(url,{signal})=>{
   const wait=new Promise((_,reject)=>signal.addEventListener('abort',()=>{aborted=true;reject(Error('aborted'))}));
   if(!stallBody)return wait;
   return {status:200,statusText:'OK',headers:{},arrayBuffer:()=>wait};
  }};
 vm.createContext(ctx);vm.runInContext(helper,ctx);
 await assert.rejects(ctx.localFetch('/api/status'),/aborted/);assert(aborted);
}
(async()=>{await run(false);await run(true);
 const ctx={AbortController,Response,setTimeout,clearTimeout,fetch:async()=>new Response('{"ok":true}',{status:200})};
 vm.createContext(ctx);vm.runInContext(helper,ctx);assert.deepEqual(await (await ctx.localFetch('/api/status')).json(),{ok:true});
 console.log('PASS: actual Safari HTTP wrapper: request timeout, response-body timeout, successful JSON');
})().catch(e=>{console.error(e);process.exit(1)});
