// Development-only HTTP mock. Never included in the firmware or published UI.
const {chromium}=require('playwright'),http=require('http'),fs=require('fs'),assert=require('node:assert/strict');
let fail=false,revision=0,posts=0,seq=0;
const settings=()=>({revision,warning:[null,null,null,null],automation:[null,null,null,null],volume_l:null,physical_outputs:false});
const caps={device:'BUSA-1M-001',version:'v0.7.2-lab.1',api_read_only:true,physical_outputs:false,temperature_channels:4,keyboard:'4x4 PCF8575',settings_write:true};
const server=http.createServer((req,res)=>{
 if(req.url.startsWith('/api/')){
  res.setHeader('Content-Type','application/json');if(fail){res.statusCode=503;res.end('{}');return;}
  if(req.url==='/api/capabilities')return res.end(JSON.stringify(caps));
  if(req.url==='/api/keys')return res.end(JSON.stringify({stream_id:'test',events:[]}));
  if(req.url==='/api/settings'){
   if(req.method==='POST'){let b='';req.on('data',c=>b+=c);req.on('end',()=>{
    assert.equal(req.headers['x-busa-token'],'a'.repeat(32));let s=JSON.parse(b);assert.equal(s.revision,revision);revision++;posts++;res.end(JSON.stringify({...s,revision}));});return;}
   return res.end(JSON.stringify(settings()));
  }
  return res.end(JSON.stringify({device:'BUSA-1M-001',version:caps.version,mode:'MONITOR_ONLY',outputs_enabled:false,simulation:false,bridge_online:true,keypad_online:false,key_stream_id:'test',sequence:++seq,temperatures:[21,22,23,24],channels:Array.from({length:4},()=>({valid:true,age_ms:10})),level_mode:'manual',volume_l:null}));
 }
 const files={'/':'index.html','/index.html':'index.html','/lab-app.js':'lab-app.js','/lab-core.js':'lab-core.js','/manifest.webmanifest':'manifest.webmanifest'};
 if(!files[req.url]){res.statusCode=404;res.end();return;}
 res.setHeader('Content-Type',req.url.endsWith('.js')?'text/javascript':'text/html');res.end(fs.readFileSync(files[req.url]));
});
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true}),page=await browser.newPage({viewport:{width:390,height:844}});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 try{
 await page.goto('http://127.0.0.1:'+server.address().port);
 await page.waitForFunction(()=>document.getElementById('badge').textContent==='Реальная телеметрия');
 await page.waitForFunction(()=>document.getElementById('settings-status').textContent.includes('ревизия'));
 assert.match(await page.locator('#summary').innerText(),/21.0/);
 await page.locator('[data-tab=settings]').click();await page.locator('#pair-token').fill('a'.repeat(32));
 await page.locator('#warning-0').fill('30,5');await page.locator('#volume').fill('50');
 await page.locator('#save-settings').click();await page.waitForFunction(()=>document.getElementById('settings-status').textContent.includes('Сохранено'));
 assert.equal(posts,1);
 fail=true;await page.waitForFunction(()=>document.getElementById('badge').textContent==='Нет связи');
 assert.equal(await page.locator('#save-settings').isDisabled(),true);
 assert.doesNotMatch(await page.locator('#summary').innerText(),/21.0/);
 await page.locator('[data-tab=connection]').click();await page.locator('#demo').click();
 await page.waitForTimeout(2200);assert.match(await page.locator('#badge').innerText(),/ДЕМО/);
 const count=await page.evaluate(()=>JSON.parse(localStorage.getItem('busa-real-history')).length);
 await page.waitForTimeout(1600);assert.equal(await page.evaluate(()=>JSON.parse(localStorage.getItem('busa-real-history')).length),count);
 fail=false;await page.locator('#connect').click();await page.waitForFunction(()=>document.getElementById('badge').textContent==='Реальная телеметрия');
 for(const tab of ['overview','temperatures','keys','automation','calculator','safety','history','settings'])await page.locator('[data-tab='+tab+']').click();
 assert.deepEqual(errors,[]);console.log('PASS: Chromium mobile viewport, real API mock, settings POST, disconnect, stale clear, demo isolation, reconnect, navigation. NOT iPhone/hardware test.');
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;server.close();});

