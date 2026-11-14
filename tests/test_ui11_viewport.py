"""Production viewport layout, reachable pagination and preserved Calculator/Wi-Fi."""
from pathlib import Path
import re,json,urllib.parse,hashlib,shutil,subprocess
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1];html=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text().split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0];fixture=json.loads((root/'tests/fixtures/ui10-status.json').read_text());preserved=json.loads((root/'tests/fixtures/ui10_preserved.json').read_text())
if 'WIFI1-UI13-' in (root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text():
 preserved.update(json.loads((root/'tests/fixtures/ui13_local_changes.json').read_text())['authorized'])
sha=lambda s:hashlib.sha256(s.encode()).hexdigest()
assert sha(re.search(r'<script>(.*?)</script>',html,re.S).group(1))==preserved['primary_browser_script']
a=re.search(r'<div class="card settingCard[^\"]*" data-stab="1"',html).start();z=html.index('<div class="card settingCard" data-stab="2"',a);assert sha(html[a:z])==preserved['wifi_card_markup']
for name in ['calculator.js','calculator.css','calculator/ui.js','calculator/core.js']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==preserved[name],name
for js in re.findall(r'<script>(.*?)</script>',html,re.S):subprocess.run(['node','--check'],input=js.encode(),check=True)
out=root.parent/'ui11/screens';out.mkdir(parents=True,exist_ok=True)
with sync_playwright() as p:
 browser=p.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox'])
 for width,height in [(320,568),(375,667),(390,744),(390,844),(430,932),(844,390)]:
  page=browser.new_page(viewport={'width':width,'height':height},device_scale_factor=1,locale='ru-RU');errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
  def route(r):
   u=urllib.parse.urlsplit(r.request.url)
   if u.path=='/':r.fulfill(content_type='text/html',body=html)
   elif u.path=='/calculator.css':r.fulfill(content_type='text/css',body=(root/'calculator.css').read_text())
   elif u.path=='/calculator.js':r.fulfill(content_type='application/javascript',body=(root/'calculator.js').read_text())
   elif u.path=='/api/status':r.fulfill(content_type='application/json',body=json.dumps(fixture))
   elif u.path=='/api/web-events':r.fulfill(content_type='application/json',body='{"boot_id":1,"events":[]}')
   else:r.fulfill(status=503,body='{}')
  page.route('**/*',route);page.goto('http://busa.fit/');page.wait_for_function('document.documentElement.dataset.viewportReady==="1"&&connected&&!pollBusy')
  def settle():page.wait_for_timeout(45)
  def audit(name,extra=''):
   settle()
   result=page.evaluate('''extra=>{const main=document.querySelector('main').getBoundingClientRect(),pager=document.querySelector('.vpPager');const bottom=main.bottom-(pager.hidden?0:pager.getBoundingClientRect().height);const active=Array.from(document.querySelectorAll('main>.page')).find(e=>!e.hidden);let sel='button,input,select,textarea,.eventRow'+extra;const nodes=Array.from(active.querySelectorAll(sel));const bad=[];for(const e of nodes){if(e.closest('details:not([open])')&&!e.closest('summary'))continue;const r=e.getBoundingClientRect();if(getComputedStyle(e).display==='none'||!r.width||!r.height)continue;if(r.top<main.top-2||r.bottom>bottom+2||r.left< -2||r.right>innerWidth+2)bad.push([e.id||e.className,e.tagName,r.top,r.bottom,main.top,bottom]);}return {bad,scroll:document.documentElement.scrollHeight>innerHeight+1||document.documentElement.scrollWidth>innerWidth+1||scrollY!==0};}''',extra)
   assert not result['scroll'] and not result['bad'],(width,height,name,result)
  def walk(name):
   audit(name);seen=set()
   for i in range(80):
    for e in page.locator('main input[id],main select[id],main button[id]').all():
     if e.is_visible():seen.add(e.get_attribute('id'))
    n=page.locator('.vpPager button').last
    if not n.is_visible() or n.is_disabled():break
    n.click();audit(name+' page'+str(i+2))
   else:raise AssertionError('Unbounded pager '+name)
   return seen
  def diagnostic(item):
   button=page.locator(f'[data-diag-btn="{item}"]')
   for _ in range(80):
    if button.is_visible():break
    back=page.locator('.vpPager button').first
    assert back.is_visible() and not back.is_disabled(),('unreachable diagnostic tab',item)
    back.click();settle()
   button.click();settle()
  # All four values, both thresholds, all relays and lower values are simultaneously visible.
  extra=',.approvedTemp strong[id],.approvedTempTitle,#homeSet0 span,#homeSet1 span,#homeSet2 span,#homeSet3 span,.relayBadge,.collectionRateLabel,.flowOnPipe,.waterUsedOnPipe,.collectedOnGlass'
  audit('home',extra);assert page.locator('.vpPager').is_hidden()
  assert page.locator('#t1').inner_text()=='78,6 °C' and page.locator('#t3').inner_text()=='80,0 °C'
  page.mouse.wheel(0,600);settle();assert page.evaluate('scrollY')==0
  page.screenshot(path=str(out/f'home-{width}x{height}.png'))
  for idx in [1,2,3,5]:page.locator(f'[data-page="{idx}"]').click();walk('section'+str(idx))
  for idx in [0,2,3,4,5,6,7,8]:page.evaluate('settingsTab('+str(idx)+')');walk('setting'+str(idx))
  page.evaluate('settingsTab(6)')
  for item in ['screen','keys','relays','temps','wifi','esp','flow']:
   diagnostic(item);walk('diagnostic '+item)
  # Explicitly unchanged sections keep their original layout and editable DOM.
  page.locator('[data-page="7"]').click();settle();assert page.evaluate('document.documentElement.dataset.viewportMode')=='unchanged'
  assert page.locator('.calcPanel .vpField,.calcPanel .vpLongWindow,.calcPanel .vpHidden').count()==0
  assert page.locator('#calc-0-v').evaluate('(e)=>getComputedStyle(e).fontSize')=='22px'
  page.locator('[data-page="5"]').click();page.evaluate('settingsTab(1)');settle();assert page.evaluate('document.documentElement.dataset.viewportMode')=='fit'
  assert page.locator('[data-stab="1"] .vpField,[data-stab="1"] .vpHidden').count()==0
  # Returning from an ordinary scrollable form restores the fitted frame at the top.
  page.evaluate('scrollTo(0,300)');page.locator('[data-page="0"]').click();audit('return home',extra)
  if width==390 and height==744:
   for newheight in [620,844,744]:
    page.set_viewport_size({'width':390,'height':newheight});audit('browser chrome resize',extra)
   page.locator('[data-page="5"]').click();page.evaluate('settingsTab(0)');settle();page.locator('#calOffset0').fill('2.3');page.set_viewport_size({'width':390,'height':360});audit('keyboard resize');page.set_viewport_size({'width':390,'height':744});settle();assert page.locator('#calOffset0').input_value()=='2.3'
   page.locator('[data-page="5"]').click();page.evaluate('settingsTab(8)');page.evaluate("()=>{for(let i=0;i<90;i++)message('Журнал '+i+' — проверка доступности строки');}");settle();walk('long journal')
   # A long diagnostic report is split into reachable screens; its raw text is preserved.
   page.evaluate('settingsTab(6)');diagnostic('temps');page.evaluate("document.getElementById('diagOneWire').textContent=Array.from({length:100},(_,i)=>'Строка '+i+' данные DS18B20').join('\\n')");settle();walk('long report')
   assert 'Строка 99' in page.locator('#diagOneWire').inner_text()
   diagnostic('esp');page.evaluate("()=>{lastDiagnosticReport=Array.from({length:100},(_,i)=>'FULL_REPORT_LINE_'+i).join('\\n');document.getElementById('diagReport').value=lastDiagnosticReport;document.execCommand=()=>{window.reportCopied=document.activeElement.value;return true;};window.dispatchEvent(new Event('resize'));}");settle()
   for _ in range(30):
    if page.locator('#diagCopy').is_visible():break
    page.locator('.vpPager button').last.click();settle()
   page.locator('#diagCopy').click();page.wait_for_function("window.reportCopied&&window.reportCopied.endsWith('FULL_REPORT_LINE_99')")

  assert not errors,errors;page.close()
 browser.close()
print('PASS viewport: six sizes; home readouts and relays visible; remaining report pages reachable; full diagnostic copy; Calculator preserved; UI13 Wi-Fi changes explicitly pinned')
