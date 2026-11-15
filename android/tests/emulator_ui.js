/* Use Playwright's Android WebView driver: desktop CDP context APIs do not apply. */
const {_android} = require('playwright');
const assert = require('node:assert/strict');
const {execFileSync} = require('node:child_process');
const fs = require('node:fs');
const path = require('node:path');
const out = path.resolve('android/out/emulator');
const adb = (...args) => execFileSync('adb', args);
const back = () => adb('shell','input','keyevent','4');
(async () => {
 let device,page;
 const errors=[];
 // Android can relaunch the first Activity while finishing initial package
 // optimisation. Reattach only to a replaced WebView, never retry assertions.
 for(let attempt=0;attempt<3;attempt++) {
  const devices=await _android.devices();assert(devices.length,'No Android emulator');
  device=devices[0];
  const webview=await device.webView({pkg:'ru.busa.monitor'});page=await webview.page();
  try {
   await page.waitForFunction('typeof connected!=="undefined"&&document.documentElement.dataset.viewportReady==="1"',null,{timeout:60000});
   await page.waitForTimeout(3000); // Let initial Android resource/configuration updates settle.
   break;
  } catch(error) {
   if(!String(error).includes('has been closed')||attempt===2)throw error;
   console.log('Android recreated its initial Activity; reattaching to the new WebView.');
   await device.close();
  }
 }
 page.on('pageerror',e=>errors.push(String(e)));
 try {
  // Real native fallback at startup must explain the failure, retain truthful
  // values and recover when returning from Wi-Fi settings / another activity.
  let nativeUi=adb('shell','uiautomator','dump','/sdcard/busa-ui.xml').toString();
  nativeUi=adb('shell','cat','/sdcard/busa-ui.xml').toString();
  fs.writeFileSync(path.join(out,'native-startup.xml'),nativeUi);
  assert(nativeUi.includes('Нет связи с БУСА'));
  assert.equal(await page.evaluate('connected'),false);
  assert.equal(await page.locator('#t0').innerText(),'Н/Д');
  const cancel=nativeUi.match(/text="Открыть без связи"[^>]*bounds="\[(\d+),(\d+)\]\[(\d+),(\d+)\]"/i);
  assert(cancel,'No offline dialog action');
  adb('shell','input','tap',String((+cancel[1]+ +cancel[3])/2),String((+cancel[2]+ +cancel[4])/2));
  await fetch('http://127.0.0.1:18080/__test/online');
  adb('shell','input','keyevent','3');
  adb('shell','am','start','-W','-n','ru.busa.monitor/.MainActivity');
  await page.waitForFunction('connected&&!pollBusy&&document.documentElement.dataset.fixtureDevicePage==="1"',null,{timeout:60000});
  assert.equal(await page.evaluate('typeof AbortSignal.timeout'),'function');
  const gets=await(await fetch('http://127.0.0.1:18080/__test/gets')).json();
  assert.equal(gets[0],'/','A native API preflight must not gate loading the device page');
  assert.equal(await page.locator('#t0').innerText(),'26,4°C');
  assert(await page.locator('.approvedRigArt').evaluate(e=>e.complete&&e.naturalWidth>0));
  for (const [section,name] of [[0,'home'],[1,'temperatures'],[2,'water'],[3,'graphs'],[7,'calculator'],[5,'settings']]) {
   await page.locator(`[data-page="${section}"]`).click();await page.waitForTimeout(150);
   assert(!await page.evaluate(()=>scrollY||document.documentElement.scrollWidth>innerWidth+1||document.documentElement.scrollHeight>innerHeight+1),name);
   assert.equal(await page.locator('.vpPager').count(),0);
   fs.writeFileSync(path.join(out,name+'.png'),adb('exec-out','screencap','-p'));
  }
  assert.equal(await page.locator('#settingsMenu>.approvedMenuItem:visible').count(),9);
  await page.locator('[data-stab-btn="1"]').click();
  await page.waitForFunction('!wifiListBusy&&document.querySelector("#net").options.length===3');
  assert.deepEqual(await page.locator('#net option').allTextContents(),['Выберите сеть','BUSA router','Phone hotspot']);
  await page.locator('#net').selectOption('BUSA router');
  assert.equal(await page.locator('#net').inputValue(),'BUSA router');
  back();await page.waitForFunction('settingsActive===-1');
  await page.locator('[data-stab-btn="5"]').click();
  await page.locator('#firmwareFile').click();
  await page.waitForTimeout(2000);
  const activity=adb('shell','dumpsys','activity','activities').toString();
  assert(activity.includes('documentsui')||activity.includes('DocumentsActivity'),'No Android document picker');
  back();await page.waitForTimeout(500);
  back();await page.waitForFunction('settingsActive===-1');
  back();await page.waitForFunction('active===0');
  await page.locator('[data-page="7"]').click();
  for(const [key,value] of [['v','1'],['from','96'],['to','40']])await page.locator('#calc-0-'+key).fill(value);
  assert((await page.locator('#calc-panel-0 [data-value="water"]').innerText()).includes('1,47'));
  back();await page.waitForTimeout(300);
  if(await page.evaluate('active')!==0)back(); // Android may dismiss the keyboard first.
  await page.waitForFunction('active===0');
  await fetch('http://127.0.0.1:18080/__test/offline');
  for(let i=0;i<3;i++){await page.evaluate('update()');await page.waitForTimeout(100);}
  await page.waitForFunction('!connected&&!pollBusy');
  assert.equal(await page.locator('#t0').innerText(),'Н/Д');
  assert(!await page.locator('#rigArtStage').evaluate(e=>e.classList.contains('hasCooling')||e.classList.contains('hasCollection')));
  const posts=await(await fetch('http://127.0.0.1:18080/__test/posts')).json();
  assert(posts.every(x=>x==='/api/time/sync'),JSON.stringify(posts));
  assert.deepEqual(errors,[]);
  const result='PASS installed Android 15 APK: native offline startup/recovery, actual host network route (no adb reverse), slow 6-second device page, older AbortSignal compatibility, ESP32 Wi-Fi network list, six screens, document picker/cancel, native Back, calculator, connection loss and no outer scroll; HTTP fixtures, no physical ESP32\n';
  fs.writeFileSync(path.join(out,'RESULT.txt'),result);process.stdout.write(result);
 } finally {await device.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
