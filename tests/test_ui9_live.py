"""Production local polling with explicit HTTP fixtures; no physical device or writes."""
from pathlib import Path
import json,urllib.parse,shutil
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1];s=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text();html=s.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
for forbidden in ['const temps=','const on=[30','72,0 л/ч','12,4 л','1,2 л','Status polling disabled ONLY','No clock writes in review mockup']:assert forbidden not in html,forbidden
fixture={'network_indicators':{'local_client':True,'internet':True},'station':{'connected':True,'ssid':'test','ip':'192.0.2.1'},'google':{'configured':True,'state':5,'http':200,'sent':1},'temperatures':[{'c':v} for v in [26.4,78.6,96.2,80]],'sound_alerts':{'channels':[{'channel':i+1,'on_c':a,'off_c':b,'sound_on':i==3} for i,(a,b) in enumerate([(96,94),(96,94),(96,94),(85,80)])],'buzzer':{'sensors':[{'on_c':30,'off_c':28},{'on_c':78,'off_c':76}],'sound_on':False}},'water':{'lpm':.6,'pulses':100,'seen_pulses':True,'total_l_estimate':1.2,'calibrated':False},'collection':{'sensor_present':False,'ml_per_min':None,'total_ml':None},'uptime_ms':10000,'display_sleep':False,'tft_theme_mode':1,'calibration_offsets':[0,0,0,0],'firmware':'BUSA UI9','ota':{'message':'test','active':False},'modules':{'pcf8575':True,'tft_attempted':True,'tft_profile':'ST7789','tft_rotation':0,'tft_boot_complete':True,'tft_profile_id':1}}
state={'offline':False}
with sync_playwright() as p:
 b=p.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox']);page=b.new_page(viewport={'width':390,'height':844},locale='ru-RU');errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 def route(r):
  u=urllib.parse.urlsplit(r.request.url)
  if u.path=='/':r.fulfill(content_type='text/html',body=html)
  elif u.path=='/calculator.css':r.fulfill(content_type='text/css',body=(root/'calculator.css').read_text())
  elif u.path=='/calculator.js':r.fulfill(content_type='application/javascript',body=(root/'calculator.js').read_text())
  elif u.path=='/api/status' and not state['offline']:r.fulfill(content_type='application/json',body=json.dumps(fixture))
  elif u.path=='/api/web-events':r.fulfill(content_type='application/json',body='{"boot_id":1,"events":[]}')
  else:r.fulfill(status=503,body='{}')
 page.route('**/*',route);page.goto('http://busa-live.test/');page.wait_for_function('connected&&!pollBusy&&statusFailures===0')
 assert page.locator('#t0').inner_text()=='26,4°C' and page.locator('#t2').inner_text()=='96,2 °C'
 for i,(a,z) in enumerate([('30,0','28,0'),('78,0','76,0'),('96,0','94,0'),('85,0','80,0')]):assert page.locator('#homeSet'+str(i)+' span').all_inner_texts()==['ВКЛ '+a+'°','ОТКЛ '+z+'°']
 assert page.locator('#homeCollectionTotal').inner_text()=='—' and page.locator('#collectionAvailability').inner_text()=='Нет датчика'
 assert page.locator('#relayBadge3').get_attribute('data-state')=='on'
 fixture['uptime_ms']=12000;fixture['water']['total_l_estimate']=1.22;fixture['water']['pulses']=120;fixture['temperatures'][0]['c']=27.1
 page.evaluate('update()');assert page.evaluate('statusFailures')==0
 assert page.locator('#t0').inner_text()=='27,1°C';assert page.locator('#homeCoolingRate').inner_text()=='36,0 л/ч';assert page.locator('#homeWaterUsed').inner_text()=='1,2 л'
 assert page.locator('.waterTrace').first.evaluate('(e)=>Number(getComputedStyle(e).opacity)')>0
 fixture['collection']={'sensor_present':True,'ml_per_min':24,'total_ml':1234,'duration_ms':7410000};fixture['uptime_ms']=14000;fixture['water']['total_l_estimate']=1.24
 page.evaluate('update()');assert page.evaluate('statusFailures')==0
 assert page.locator('#homeCollectionTotal').inner_text()=='1,2' and page.locator('#homeCollectionUnit').inner_text()=='л';assert page.locator('#homeCollectionRate').inner_text()=='24,0'
 assert page.locator('.productTrace').evaluate('(e)=>Number(getComputedStyle(e).opacity)')>0
 state['offline']=True
 for _ in range(3):page.evaluate('update()')
 assert not page.evaluate('connected');assert page.locator('#t0').inner_text()=='Н/Д';assert page.locator('#homeCoolingRate').inner_text()=='— л/ч';assert page.locator('#homeCollectionTotal').inner_text()=='—'
 assert page.locator('.waterTrace').first.evaluate('(e)=>getComputedStyle(e).opacity')=='0'
 assert page.locator('.boilerBoil i').first.evaluate('(e)=>getComputedStyle(e).animationName')=='busaBoil';assert page.locator('.collectionSurface').evaluate('(e)=>getComputedStyle(e).animationName')=='busaSurface'
 assert page.locator('#relayBadge3').get_attribute('data-state')=='unknown'
 assert not errors,errors;b.close()
print('PASS production HTTP polling: live temperatures, all threshold mappings, one decimal and units, real cooling estimate, optional collection, true relay state, disconnect clearing, passive versus decorative animations; no demo scripts')
