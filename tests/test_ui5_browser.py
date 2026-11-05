from pathlib import Path
import re,json
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1]; source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text();html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
output=root.parent/'ui5/preview';output.mkdir(parents=True,exist_ok=True)
(output/'BUSA_UI5_preview.html').write_text(html)
for script in re.findall('<script>(.*?)</script>',html,re.S):
 import subprocess
 subprocess.run(['node','--check'],input=script.encode(),check=True)
with sync_playwright() as pw:
 browser=pw.chromium.launch(executable_path='/usr/bin/chromium',args=['--no-sandbox'])
 for width in [320,390,393,430,768]:
  for theme in ['system','dark','light']:
   page=browser.new_page(viewport={'width':width,'height':844},device_scale_factor=2,locale='ru-RU')
   errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
   def route(r):
    if r.request.url=='http://busa.test/':r.fulfill(content_type='text/html',body=html)
    elif r.request.url.startswith('http://busa.test/calculator.css'):r.fulfill(content_type='text/css',body=(root/'calculator.css').read_text())
    elif r.request.url.startswith('http://busa.test/calculator.js'):r.fulfill(content_type='application/javascript',body=(root/'calculator.js').read_text())
    else:r.fulfill(status=503,body='{}')
   page.route('**/*',route);page.goto('http://busa.test/');page.wait_for_timeout(150)
   page.evaluate('(t)=>document.documentElement.dataset.theme=t',theme)
   assert page.locator('h1').inner_text()=='BUSA MONITOR'
   assert page.locator('.approvedHomeTitle').count()==0
   assert not re.search(r'№\s*001|БУСА-1М',page.locator('body').inner_text())
   assert re.search(r'\d{2}:\d{2}$',page.locator('#clockDisplay').inner_text())
   assert not re.search(r'\d{2}:\d{2}:\d{2}',page.locator('#clockDisplay').inner_text())
   assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
   for tile in page.locator('.approvedTemp').all():
    assert tile.evaluate('(e)=>getComputedStyle(e).borderTopWidth')=='0px'
    assert tile.evaluate('(e)=>getComputedStyle(e).boxShadow')=='none'
    assert tile.evaluate('(e)=>getComputedStyle(e).backgroundImage')=='none'
    assert tile.evaluate('(e)=>getComputedStyle(e).backgroundColor')=='rgba(0, 0, 0, 0)'
   assert page.locator('.approvedRig img').count()==0
   assert page.locator('.approvedRig svg').count()==1
   for button in page.locator('.appTabs button').all():
    box=button.bounding_box();assert box['x']>=0 and box['x']+box['width']<=width+1
   for tab in [1,2,3,4,5,7,0]:
    page.locator(f'[data-page="{tab}"]').click();assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
   if width==390 and theme=='system':page.screenshot(path=str(output/'BUSA_UI5_preview.png'),full_page=True)
   assert not errors,errors
   page.close()
 browser.close()
print('PASS: 15 viewport/theme combinations; all navigation; no number/duplicate brand; HH:MM; borderless temperature; vector image; no JS exception/overflow')
