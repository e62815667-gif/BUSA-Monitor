"""Production UI7 local browser against explicit fixtures; no actual ESP32 used."""
from pathlib import Path
import re,subprocess,json,urllib.parse,time,shutil
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1];source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text();html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
for script in re.findall('<script>(.*?)</script>',html,re.S):subprocess.run(['node','--check'],input=script.encode(),check=True)
artifacts=root.parent/'ui7/preview';artifacts.mkdir(parents=True,exist_ok=True)
with sync_playwright() as pw:
 browser=pw.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox'])
 for width in [320,390,430,768]:
  for theme in ['system','dark','light']:
   page=browser.new_page(viewport={'width':width,'height':844},device_scale_factor=2,locale='ru-RU',timezone_id='Asia/Yekaterinburg');errors=[];page.on('pageerror',lambda e:errors.append(str(e)));state={'empty':False}
   def route(r):
    u=urllib.parse.urlsplit(r.request.url)
    if u.path=='/':r.fulfill(content_type='text/html',body=html)
    elif u.path=='/calculator.css':r.fulfill(content_type='text/css',body=(root/'calculator.css').read_text())
    elif u.path=='/calculator.js':r.fulfill(content_type='application/javascript',body=(root/'calculator.js').read_text())
    elif u.path=='/api/chart-history':
     minutes=int(urllib.parse.parse_qs(u.query)['minutes'][0]);n=min(minutes*30,160);series=[[None if state['empty'] or i<n-10 else 20+k*20+i/100 for i in range(n)] for k in range(4)];water=[None if state['empty'] or i<n-10 else 36+i/100 for i in range(n)]
     r.fulfill(content_type='application/json',body=json.dumps({'ok':True,'minutes':minutes,'temperature':series,'cooling_lph':water,'collection_sensor':False,'cooling_calibrated':False}))
    elif u.path=='/api/web-events':
     now=int(time.time());r.fulfill(content_type='application/json',body=json.dumps({'boot_id':7,'events':[{'id':2,'epoch':now-5,'kind':2,'text':'Авария: активен'},{'id':1,'epoch':now-10,'kind':0,'text':'Устройство включено'}]}))
    else:r.fulfill(status=503,body='{}')
   page.route('**/*',route);page.goto('http://busa.test/');page.wait_for_selector('#busaCalculator .calcPanel',state='attached');page.evaluate('(v)=>document.documentElement.dataset.theme=v',theme)
   assert page.locator('.appTabs button').evaluate_all('(a)=>a.map(e=>Number(e.dataset.page))')==[0,1,2,3,7,5]
   assert page.locator('.approvedRigArt').count()==1 and page.locator('.approvedRig svg').count()==0
   assert page.locator('.rigSheen').evaluate('(e)=>getComputedStyle(e).animationName')=='rigShine'
   assert page.locator('.approvedTemp strong[id]').evaluate_all('(a)=>a.map(e=>e.id)')==['t0','t2','t1','t3']
   if width==390 and theme=='system':page.screenshot(path=str(artifacts/'01-home.png'),full_page=True)
   page.locator('[data-page="1"]').click()
   assert page.locator('.approvedSensorLive').evaluate_all('(a)=>a.map(e=>e.id)')==['tempLive0','tempLive2','tempLive1','tempLive3']
   for sensor in page.locator('.approvedSensor').all():assert sensor.bounding_box()['height']<=90
   colors=page.locator('.approvedSensorLive').evaluate_all('(a)=>a.map(e=>getComputedStyle(e).color)');assert colors==['rgb(56, 189, 248)','rgb(244, 200, 74)','rgb(83, 212, 134)','rgb(255, 100, 115)'],colors
   page.locator('.sensorEdit summary').first.click();assert page.locator('#tempOn0').is_visible()
   if width==390 and theme=='system':page.locator('.sensorEdit summary').first.click();page.screenshot(path=str(artifacts/'02-temperatures.png'),full_page=True)
   page.locator('[data-page="2"]').click();assert page.locator('[data-section="2"] canvas,[data-section="2"] .approvedGraphPlaceholder').count()==0
   values=page.evaluate('''()=>[formatVolumeMl(0),formatVolumeMl(999.9),formatVolumeMl(1000),formatVolumeMl(1234),formatRunTime(7410000)]''');assert values==['0 мл','999 мл','1,00 л','1,23 л','02:03'],values
   page.evaluate('''()=>{updateLiquidPanels({uptime_ms:10000,collection:{sensor_present:true,ml_per_min:24,total_ml:1234,duration_ms:7410000},water:{seen_pulses:true,total_l_estimate:1.2,calibrated:false}});updateLiquidPanels({uptime_ms:12000,collection:{sensor_present:true,ml_per_min:24,total_ml:1234,duration_ms:7410000},water:{seen_pulses:true,total_l_estimate:1.22,calibrated:false}});}''')
   assert page.locator('#collectionMlMin').inner_text()=='24,00' and page.locator('#collectionLph').inner_text()=='1,44';assert page.locator('#collectionTotal').inner_text()=='1,23 л';assert page.locator('#coolingLph').inner_text()=='36,00'
   assert page.locator('#rigArtStage').evaluate('(e)=>e.classList.contains("hasCollection")&&e.classList.contains("hasCooling")')
   page.evaluate('updateLiquidPanels(null)');assert page.locator('#collectionMlMin').inner_text()=='—';assert page.locator('#coolingTotal').inner_text()=='—'
   if width==390 and theme=='system':page.screenshot(path=str(artifacts/'03-water-collection.png'),full_page=True)
   page.locator('[data-page="3"]').click();page.wait_for_function('busaGraphData.minutes===5')
   assert page.locator('[data-chart]').all_inner_texts()==['Температура','Отбор','Охлаждение']
   assert page.locator('#chartLegend span').all_inner_texts()==['Вода','Куб','Колонна','Авария']
   page.locator('[data-chart="2"]').click();assert 'калибровки' in page.locator('#chartState').inner_text()
   page.locator('[data-chart="1"]').click();assert page.locator('#chartState').inner_text()=='Нет датчика отбора'
   page.locator('[data-chart="0"]').click()
   if width==390 and theme=='system':page.screenshot(path=str(artifacts/'04-graphs-fixture.png'),full_page=True)
   page.locator('[data-page="5"]').click();page.locator('[data-stab-btn="8"]').click();page.wait_for_function("eventRecords.some(e=>e.id==='esp-7-2')")
   assert page.locator('[data-event]').count()==0
   assert page.evaluate("eventRecords.filter(e=>e.source==='device').map(e=>e.seq)")==[2,1]
   before=page.evaluate('eventRecords.length');page.evaluate("message('25.4 C','temperature')");assert page.evaluate('eventRecords.length')==before
   page.evaluate("message('Новое событие для теста')");assert 'Новое событие' in page.locator('#events .eventRow').first.inner_text()
   if width==390 and theme=='system':page.screenshot(path=str(artifacts/'06-journal-fixture.png'),full_page=True)
   page.locator('[data-page="7"]').click();page.locator('#calc-0-v').fill('1');page.locator('#calc-0-from').fill('96');page.locator('#calc-0-to').fill('40');page.locator('#calc-panel-0 button[type=submit]').click();assert '1,47' in page.locator('[data-value="water"]').inner_text()
   assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'),(width,theme)
   assert not errors,errors;page.close()
 browser.close()
print('PASS UI7 browser: 12 viewport/themes; restored apparatus/animation; compact ordered sensor colors; volume/rate/time formats; two panels/no graph; correct three chart tabs; reverse journal/no temperature readings; calculator/no overflow/JS error')
