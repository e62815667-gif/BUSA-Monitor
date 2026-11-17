const {_android}=require('playwright');
const assert=require('node:assert/strict');
const {execFileSync}=require('node:child_process');
const fs=require('node:fs');
const adb=(...args)=>execFileSync('adb',args);
const out='android/out/emulator/';
function nativeNodes(){
 adb('shell','uiautomator','dump','/sdcard/busa-cloud-ui.xml');
 return adb('shell','cat','/sdcard/busa-cloud-ui.xml').toString().match(/<node\b[^>]*>/g)||[];
}
function tapNode(node){
 assert(node,'Native control not found');const b=node.match(/bounds="\[(\d+),(\d+)\]\[(\d+),(\d+)\]"/);assert(b,'Native bounds missing');
 adb('shell','input','tap',String((+b[1]+ +b[3])/2),String((+b[2]+ +b[4])/2));
}
function fillNative(index,value){
 tapNode(nativeNodes().filter(n=>n.includes('class="android.widget.EditText"'))[index]);
 adb('shell','input','keyevent','123');adb('shell','input','keyevent',...Array(100).fill('67'));
 adb('shell','input','text',value);
}
function positive(){tapNode(nativeNodes().find(n=>n.includes('resource-id="android:id/button1"')));}
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
  // Configure the cloud through the real first-launch form, rather than
  // installing working credentials directly into private preferences.
  let controls;
  for(const until=Date.now()+10000;Date.now()<until;){
   controls=nativeNodes().filter(n=>n.includes('class="android.widget.EditText"'));
   if(controls.length===2)break;
  }
  assert.equal(controls.length,2,'No first-launch internet connection form');
  fillNative(0,'http://10.0.2.2:18080/exec');fillNative(1,'wrong');positive();
  assert.equal(nativeNodes().filter(n=>n.includes('class="android.widget.EditText"')).length,2,'Invalid key dismissed the form');
  const before=await(await fetch('http://127.0.0.1:18080/__test/gets')).json();
  assert(!before.some(p=>p.startsWith('/exec')),'Invalid local key was sent to the server');
  fillNative(1,'a'.repeat(64));positive();
  await page.waitForFunction('typeof connected!=="undefined"&&connected&&!pollBusy',null,{timeout:60000});
  assert.equal(await page.locator('#t0').innerText(),'26,4°C');
  assert(await page.locator('#localLed').evaluate(e=>e.classList.contains('localOn')));
  assert(await page.locator('#googleLed').evaluate(e=>e.classList.contains('googleOn')));
  // A view key must never enter the HTML, JS storage or web-facing requests.
  assert(!(await page.content()).includes('a'.repeat(64)));
  assert(!(await page.evaluate('JSON.stringify(localStorage)')).includes('a'.repeat(64)));
  await page.waitForFunction('document.querySelector("#homeCoolingRate")?.textContent==="120,0 л/ч"',null,{timeout:45000});
  for(let i=0;i<3;i++){await page.evaluate('update()');await page.waitForFunction('!pollBusy');}
  assert.equal(await page.locator('#homeCoolingRate').innerText(),'120,0 л/ч','Cached cloud sample lost measured cooling');
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
  for(const state of ['stale','empty','denied','malformed']){
   await restart(state);await page.waitForTimeout(3000);
   assert.equal(await page.locator('#t0').innerText(),'Н/Д',state);
   assert.equal(await page.locator('#localLed').evaluate(e=>e.classList.contains('localOn')),false,state);
   assert.equal(await page.locator('#googleLed').evaluate(e=>e.classList.contains('googleOn')),false,state);
   fs.writeFileSync(out+'cloud-'+state+'.png',adb('exec-out','screencap','-p'));
  }
  await restart('fresh');await page.waitForFunction('typeof connected!=="undefined"&&connected&&!pollBusy');
  assert.equal(await page.locator('#t0').innerText(),'26,4°C');
  await fetch('http://127.0.0.1:18080/__test/cloud?state=http_error');
  await page.waitForFunction('!connected&&!pollBusy',null,{timeout:60000});
  await fetch('http://127.0.0.1:18080/__test/cloud?state=fresh');
  await page.waitForFunction('connected&&!pollBusy',null,{timeout:60000});
  // Re-enter local mode via the address form with Wi-Fi already available.
  adb('shell','svc','wifi','enable');
  let wifi=false;
  for(const until=Date.now()+20000;Date.now()<until;){
   if(adb('shell','dumpsys','connectivity').toString().includes('Transports: WIFI')){wifi=true;break;}
  }
  assert(wifi,'Wi-Fi network did not become available');
  adb('shell','input','keyevent','4');
  tapNode(nativeNodes().find(n=>/text="Адрес БУСА"/i.test(n)));
  fillNative(0,'http://10.0.2.2:18080');positive();
  await page.waitForFunction('typeof connected!=="undefined"&&connected&&!pollBusy&&document.documentElement.dataset.fixtureDevicePage==="1"',null,{timeout:60000});
  assert.equal(await page.locator('#t0').innerText(),'26,4°C');
  assert.equal(await page.locator('#temperatureSave').isDisabled(),false);
  const result='PASS Android mobile-only cloud mode: real first-launch setup/invalid key, Wi-Fi disabled, cellular host route, authenticated JSONP reader, six screens/calculator, stable cached cooling, key kept out of WebView, stale/empty/unauthorized/malformed cleared, automatic network failure/recovery, cloud-to-local address switch with existing Wi-Fi; mocked Google, no physical ESP32 or user deployment\n';
  fs.writeFileSync(out+'CLOUD_RESULT.txt',result);process.stdout.write(result);
 }finally{if(device)await device.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
