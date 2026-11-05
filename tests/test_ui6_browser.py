"""UI6 production browser markup/scripts against explicit offline fixtures."""
from pathlib import Path
import re,subprocess,json,urllib.parse
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1];source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text();html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
for script in re.findall('<script>(.*?)</script>',html,re.S):subprocess.run(['node','--check'],input=script.encode(),check=True)
artifacts=root.parent/'ui6/preview';artifacts.mkdir(parents=True,exist_ok=True)
with sync_playwright() as pw:
 browser=pw.chromium.launch(executable_path='/usr/bin/chromium',args=['--no-sandbox'])
 for width in [320,390,430,768]:
  for theme in ['system','dark','light']:
   page=browser.new_page(viewport={'width':width,'height':844},device_scale_factor=2,locale='ru-RU');errors=[];page.on('pageerror',lambda e:errors.append(str(e)));state={'empty':False,'requests':[]}
   def route(r):
    path=urllib.parse.urlsplit(r.request.url)
    if path.path=='/':r.fulfill(content_type='text/html',body=html)
    elif path.path=='/calculator.css':r.fulfill(content_type='text/css',body=(root/'calculator.css').read_text())
    elif path.path=='/calculator.js':r.fulfill(content_type='application/javascript',body=(root/'calculator.js').read_text())
    elif path.path=='/api/chart-history':
     hours=int(urllib.parse.parse_qs(path.query)['hours'][0]);state['requests'].append(hours);n=min(hours*60,160)
     series=[[None if state['empty'] or i< n-10 or i==n-3 else 20+k*20+i/100 for i in range(n)] for k in range(4)]
     r.fulfill(content_type='application/json',body=json.dumps({'ok':True,'hours':hours,'temperature':series,'collection_sensor':False,'power_sensor':False}))
    else:r.fulfill(status=503,body='{}')
   page.route('**/*',route);page.goto('http://busa.test/');page.wait_for_selector('#busaCalculator .calcPanel',state='attached');page.evaluate('(v)=>document.documentElement.dataset.theme=v',theme)
   assert page.locator('h1').inner_text()=='BUSA MONITOR'
   assert page.locator('.appTabs button').evaluate_all('(els)=>els.map(e=>Number(e.dataset.page))')==[0,1,2,3,7,5]
   assert not re.search(r'№\s*001|БУСА-1М|К1|K1',page.locator('body').inner_text())
   for button in page.locator('.appTabs button').all():
    box=button.bounding_box();assert box['x']>=0 and box['x']+box['width']<=width+1
   page.evaluate('updateRelayBadges([{sound_on:true},{sound_on:false},{sound_on:false},{sound_on:true}])')
   assert page.locator('#relayBadge0').get_attribute('data-state')=='on';assert page.locator('#relayBadge3').get_attribute('data-state')=='on'
   assert 'ТЭН откл.' in page.locator('#relayBadge0').inner_text();assert 'Кран закрыт' in page.locator('#relayBadge1').inner_text()
   if width==390 and theme=='system':page.screenshot(path=str(artifacts/'home.png'),full_page=True)
   page.evaluate('updateRelayBadges(null)');assert page.locator('#soundStatus0').inner_text()=='Нет связи'
   for tab in [1,2,3,7,5,0]:
    page.locator(f'[data-page="{tab}"]').click();assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
   page.locator('[data-page="3"]').click();page.wait_for_function('busaGraphData.hours===1')
   for hours in [6,12,24,1]:
    page.locator(f'[data-hours="{hours}"]').click();page.wait_for_function('(h)=>busaGraphData.hours===h',arg=hours);assert page.locator(f'[data-hours="{hours}"]').get_attribute('aria-pressed')=='true'
   assert page.locator('#chartState').inner_text()=='Данные с включения'
   if width==390 and theme=='system':page.screenshot(path=str(artifacts/'graphs-fixture.png'),full_page=True)
   page.locator('[data-chart="1"]').click();assert page.locator('#chartState').inner_text()=='Нет датчика отбора'
   page.locator('[data-chart="2"]').click();assert page.locator('#chartState').inner_text()=='Нет данных питания'
   state['empty']=True;page.locator('[data-chart="0"]').click();page.evaluate('refreshGraphHistory(true)');page.wait_for_function("document.getElementById('chartState').textContent==='Ожидание измерений'")
   page.locator('[data-page="5"]').click();page.locator('[data-stab-btn="8"]').click();assert page.locator('#events').is_visible();assert page.locator('#settingsMenu').is_hidden();page.locator('#settingsBack').click();assert page.locator('#settingsMenu').is_visible()
   page.locator('[data-stab-btn="6"]').click();assert page.locator('[data-stab="6"]').is_visible();page.locator('#settingsBack').click()
   page.locator('[data-page="7"]').click();page.locator('#calc-0-v').fill('1');page.locator('#calc-0-from').fill('96');page.locator('#calc-0-to').fill('40');page.locator('#calc-panel-0 button[type=submit]').click();assert '1,47' in page.locator('[data-value="water"]').inner_text()
   assert page.locator('#calc-panel-0 .calcNote').get_attribute('open') is None
   if width==390 and theme=='system':page.screenshot(path=str(artifacts/'calculator-fixture.png'),full_page=True)
   assert not errors,errors;assert page.evaluate('document.documentElement.scrollWidth<=innerWidth');page.close()
 browser.close()
print('PASS UI6 Chromium: 12 viewport/theme combinations; exactly six tabs; Events/Diagnostics inside Settings; role state/stale handling; all chart tabs/ranges/no-data; calculator; no overflow/JS errors. Preview values are fixtures.')
