"""All settings visible and no automatic presentation pages anywhere.

Embedded production UI, HTTP fixtures only. BUSA_TEST_BASELINE=21970fa
reproduces the missing ninth settings item in UI13.
"""
from pathlib import Path
import os, json, shutil, subprocess, urllib.parse, re, hashlib
from playwright.sync_api import sync_playwright

root = Path(__file__).resolve().parents[1]
baseline = os.environ.get('BUSA_TEST_BASELINE')
source = (subprocess.check_output(['git','show',baseline+':firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h'],cwd=root).decode()
          if baseline else (root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text())
html = source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
fixture = json.loads((root/'tests/fixtures/ui10-status.json').read_text())
preserved = json.loads((root/'tests/fixtures/ui10_preserved.json').read_text())
for name in ['calculator.js','calculator.css','calculator/ui.js','calculator/core.js']:
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == preserved[name], name
authorized = json.loads((root/'tests/fixtures/ui13_local_changes.json').read_text())['authorized']
assert hashlib.sha256(re.search(r'<script>(.*?)</script>',html,re.S).group(1).encode()).hexdigest() == authorized['primary_browser_script']
shots = root.parent/'ui14/screens'; shots.mkdir(parents=True,exist_ok=True)
sizes = [(320,568),(375,600),(375,667),(390,636),(390,744),(390,844),(430,932),(844,390)]
with sync_playwright() as p:
    browser = p.webkit.launch() if os.environ.get('BUSA_TEST_BROWSER') == 'webkit' else p.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox'])
    for width,height in sizes:
        page = browser.new_page(viewport={'width':width,'height':height},locale='ru-RU')
        errors=[]; page.on('pageerror',lambda e:errors.append(str(e)))
        def route(r):
            u=urllib.parse.urlsplit(r.request.url)
            if u.path=='/':r.fulfill(content_type='text/html',body=html)
            elif u.path in ['/calculator.css','/calculator.js']:r.fulfill(body=(root/u.path[1:]).read_bytes())
            elif u.path=='/api/status':r.fulfill(content_type='application/json',body=json.dumps(fixture))
            elif u.path=='/api/web-events':r.fulfill(content_type='application/json',body='{"boot_id":1,"events":[]}')
            elif u.path=='/api/wifi/scan':r.fulfill(content_type='application/json',body='{"scanning":false,"networks":[],"error":"no_networks"}')
            else:r.fulfill(status=503,body='{}')
        page.route('**/*',route);page.goto('http://busa-ui14.test/')
        page.wait_for_function('document.documentElement.dataset.viewportReady==="1"&&connected&&!pollBusy')
        def settle():page.wait_for_timeout(75)
        def inside(selector):
            result=page.locator(selector).evaluate_all('''es=>{
              const f=document.querySelector('main').getBoundingClientRect();
              return es.map(e=>{const r=e.getBoundingClientRect();return {id:e.id||e.textContent.slice(0,30),top:r.top,bottom:r.bottom,
                ok:r.width>0&&r.height>0&&r.top>=f.top-1&&r.bottom<=f.bottom+1&&r.left>=-1&&r.right<=innerWidth+1};});
            }''')
            if not result or not all(x['ok'] for x in result):
                page.screenshot(path=str(shots/f'failure-{width}x{height}.png'))
                raise AssertionError((width,height,selector,result))
        def frame():
            assert page.locator('.vpPager').count()==0
            assert not page.evaluate('scrollY!==0||document.documentElement.scrollWidth>innerWidth+1||document.documentElement.scrollHeight>innerHeight+1')
            assert page.locator('main').evaluate('(e)=>e.scrollTop')<=1
        page.locator('[data-page="5"]').click();settle()
        assert page.locator('#settingsMenu>.approvedMenuItem:visible').count()==9,(width,height,'missing settings item')
        assert page.locator('#settingsMenu>.approvedMenuItem').evaluate_all('(es)=>es.map(e=>Number(e.dataset.stabBtn))')==[0,1,2,3,7,5,8,6,4]
        inside('#settingsMenu>.approvedMenuItem');frame()
        menu_size=page.locator('#settingsMenu').evaluate('''e=>{
          const r=e.getBoundingClientRect();return {top:e.scrollTop,outside:[...e.children].some(x=>{const b=x.getBoundingClientRect();return b.top<r.top-1||b.bottom>r.bottom+1;})};
        }''')
        assert not menu_size['outside'] and menu_size['top']==0,(width,height,menu_size)
        if width in [375,390]:page.screenshot(path=str(shots/f'settings-{width}x{height}.png'))
        for item in [0,1,2,3,7,5,8,6,4]:
            page.locator(f'[data-stab-btn="{item}"]').click();settle();frame()
            card=page.locator(f'.settingCard[data-stab="{item}"]')
            assert card.is_visible()
            # Every control is on this screen or reachable by native in-panel
            # scrolling, never by presenting a second page or dropping nodes.
            for element in card.locator('button,input,select,textarea').all():
                if not element.is_visible():continue
                element.scroll_into_view_if_needed();settle()
                box=element.bounding_box();bounds=page.locator('main').bounding_box()
                if not (box['y']>=bounds['y']-1 and box['y']+box['height']<=bounds['y']+bounds['height']+1):
                    page.screenshot(path=str(shots/f'control-failure-{width}x{height}.png'))
                    extra=card.evaluate('(e)=>({classes:e.className,rect:e.getBoundingClientRect().toJSON(),height:e.clientHeight,scroll:e.scrollHeight,top:e.scrollTop,max:getComputedStyle(e).maxHeight,overflow:getComputedStyle(e).overflowY,room:e.style.getPropertyValue("--content-room")})')
                    raise AssertionError((width,height,item,element.get_attribute('id'),box,bounds,extra))
                frame()
            if item==0:inside('[id^="calOffset"],#tempCalSave')
            if item==1:inside('#net,#pass,#wifiRetry,#save,#disconnectWifi')
            if item==6:
                for name in ['screen','keys','relays','temps','wifi','esp','flow']:
                    tab=page.locator(f'[data-diag-btn="{name}"]');tab.scroll_into_view_if_needed();tab.click();settle();frame()
                    assert page.locator(f'[data-diag-panel="{name}"]').is_visible()
            page.locator('#settingsBack').click();settle()
            inside('#settingsMenu>.approvedMenuItem');frame()
        for section in [0,1,2,3]:
            page.locator(f'[data-page="{section}"]').click();settle();frame()
            if section==0:
                inside('.approvedTempTitle,.approvedTemp strong[id],.relayBadge')
                inside('.holoMetrics' if page.locator('.holoHome').count() else '.collectionRateLabel')
            if section==1:inside('.approvedSensorHead,#temperatureSave')
            if section==2:inside('.liquidPanel')
        page.locator('[data-page="5"]').click();page.evaluate('settingsTab(8)')
        page.evaluate("()=>{for(let i=0;i<100;i++)message('Событие '+i)}");settle()
        assert 'Событие 99' in page.locator('#events .eventRow').first.inner_text()
        assert page.locator('#events .eventRow').count()>=100;frame()
        page.locator('#events .eventRow').last.scroll_into_view_if_needed();settle();frame()
        page.locator('#settingsBack').click();page.evaluate('settingsTab(6)')
        page.locator('[data-diag-btn="esp"]').click();settle()
        page.evaluate("()=>{lastDiagnosticReport=Array.from({length:100},(_,i)=>'FULL_REPORT_LINE_'+i).join('\\n');document.getElementById('diagReport').value=lastDiagnosticReport;document.execCommand=()=>{window.reportCopied=document.activeElement.value;return true;};}")
        page.locator('#diagCopy').scroll_into_view_if_needed();page.locator('#diagCopy').click()
        page.wait_for_function("window.reportCopied&&window.reportCopied.endsWith('FULL_REPORT_LINE_99')");frame()
        page.locator('[data-page="7"]').click();settle()
        mode=page.evaluate('document.documentElement.dataset.viewportMode')
        assert mode==('calculator' if page.locator('html.busaDesign').count() else 'unchanged')
        assert page.locator('#calc-0-v').evaluate('(e)=>getComputedStyle(e).fontSize')=='22px'
        assert not errors,errors;page.close()
    browser.close()
print('PASS UI14: eight sizes; all nine settings items visible together in order; zero pagination; all sections/settings/diagnostics reachable; full journal/report retained; UI13 Wi-Fi and calculator preserved')
