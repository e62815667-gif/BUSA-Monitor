const {_android}=require('playwright');
const assert=require('node:assert/strict');
const {execFileSync}=require('node:child_process');
const fs=require('node:fs');
const adb=(...args)=>execFileSync('adb',args);
const out='android/out/emulator/';
(async()=>{
 let device,page;
 async function attach(){
  for(let n=0;n<3;n++){
   device=(await _android.devices())[0];page=await(await device.webView({pkg:'ru.busa.monitor'})).page();
   try{await page.waitForFunction('window.busaCloudInstalled===true',null,{timeout:60000});return;}
   catch(e){if(!String(e).includes('has been closed')||n===2)throw e;await device.close();}
  }
 }
 async function restart(state){
  await fetch('http://127.0.0.1:18080/__test/cloud?state='+state);
  await device.close();adb('shell','am','force-stop','ru.busa.monitor');
  adb('shell','am','start','-W','-n','ru.busa.monitor/.MainActivity');await attach();
 }
 try{
  await attach();
  assert(adb('shell','settings','get','global','wifi_on').toString().trim()==='0','Wi-Fi is not disabled');
  await page.waitForFunction('connected&&!pollBusy',null,{timeout:60000});
  assert.equal(await page.locator('#t0').innerText(),'26,4°C');
  assert(await page.locator('#localLed').evaluate(e=>e.classList.contains('localOn')));
  assert(await page.locator('#googleLed').evaluate(e=>e.classList.contains('googleOn')));
  // A view key must never enter the HTML, JS storage or web-facing requests.
  assert(!(await page.content()).includes('a'.repeat(64)));
  assert(!(await page.evaluate('JSON.stringify(localStorage)')).includes('a'.repeat(64)));
  for(const [section,name] of [[0,'home'],[1,'temperatures'],[2,'water'],[3,'graphs'],[7,'calculator'],[5,'settings']]){
   await page.locator(`[data-page="${section}"]`).click();await page.waitForTimeout(100);
   assert(!await page.evaluate(()=>scrollY||document.documentElement.scrollWidth>innerWidth+1||document.documentElement.scrollHeight>innerHeight+1),name);
   fs.writeFileSync(out+'cloud-'+name+'.png',adb('exec-out','screencap','-p'));
  }
  await page.locator('[data-page="7"]').click();
  for(const [key,value] of [['v','1'],['from','96'],['to','40']])await page.locator('#calc-0-'+key).fill(value);
  assert((await page.locator('#calc-panel-0 [data-value="water"]').innerText()).includes('1,47'));
  assert.equal(await page.locator('#temperatureSave').isDisabled(),true);
  // A stale last row, an empty sheet and invalid authorization never become
  // green indicators or live sensor readings.
  for(const state of ['stale','empty','denied']){
   await restart(state);await page.waitForTimeout(3000);
   assert.equal(await page.locator('#t0').innerText(),'Н/Д',state);
   assert.equal(await page.locator('#localLed').evaluate(e=>e.classList.contains('localOn')),false,state);
   assert.equal(await page.locator('#googleLed').evaluate(e=>e.classList.contains('googleOn')),false,state);
   fs.writeFileSync(out+'cloud-'+state+'.png',adb('exec-out','screencap','-p'));
  }
  await restart('fresh');await page.waitForFunction('connected&&!pollBusy');
  assert.equal(await page.locator('#t0').innerText(),'26,4°C');
  const result='PASS Android mobile-only cloud mode: Wi-Fi disabled, real host cellular route, authenticated JSONP reader, six shared screens, calculator, view key kept out of WebView, stale/empty/unauthorized data remain N/D and disconnected, recovery; mocked Google endpoint, no physical ESP32 or user deployment\n';
  fs.writeFileSync(out+'CLOUD_RESULT.txt',result);process.stdout.write(result);
 }finally{if(device)await device.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
