"""Photo regressions: all offsets on one screen, aligned Wi-Fi, stable animated rig.

Runs the embedded production HTML with HTTP fixtures; does not access a real ESP.
Set BUSA_TEST_BASELINE=440d87b to reproduce the old pagination failure.
"""
from pathlib import Path
import os, json, re, shutil, subprocess, urllib.parse
from playwright.sync_api import sync_playwright

root = Path(__file__).resolve().parents[1]
baseline = os.environ.get('BUSA_TEST_BASELINE')
source = (subprocess.check_output(['git', 'show', baseline + ':firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h'], cwd=root).decode()
          if baseline else (root / 'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text())
html = source.split('R"HTMLBUSA(', 1)[1].split(')HTMLBUSA"', 1)[0]
for js in re.findall(r'<script>(.*?)</script>', html, re.S):
    subprocess.run(['node', '--check'], input=js.encode(), check=True)
fixture = json.loads((root / 'tests/fixtures/ui10-status.json').read_text())
fixture['station']['connected'] = False
fixture['station']['last_error'] = 'Не подключено. Проверь пароль'
shots = root.parent / 'ui13/screens'
shots.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    browser = (p.webkit.launch() if os.environ.get('BUSA_TEST_BROWSER') == 'webkit' else
               p.chromium.launch(executable_path=shutil.which('chromium'), args=['--no-sandbox']))
    for width, height in [(320, 568), (375, 667), (390, 636), (390, 744), (390, 844), (430, 932), (844, 390)]:
        page = browser.new_page(viewport={'width': width, 'height': height}, locale='ru-RU')
        errors, posts = [], []
        page.on('pageerror', lambda e: errors.append(str(e)))
        def route(r):
            u = urllib.parse.urlsplit(r.request.url)
            if u.path == '/':
                r.fulfill(content_type='text/html', body=html)
            elif u.path in ['/calculator.css', '/calculator.js']:
                r.fulfill(body=(root / u.path[1:]).read_bytes())
            elif u.path == '/api/status':
                r.fulfill(content_type='application/json', body=json.dumps(fixture))
            elif u.path == '/api/web-events':
                r.fulfill(content_type='application/json', body='{"boot_id":1,"events":[]}')
            elif u.path == '/api/wifi/scan':
                r.fulfill(content_type='application/json', body=json.dumps({'scanning': False, 'networks': [{'ssid': 'Домашняя сеть', 'rssi': -40}], 'error': ''}))
            elif u.path == '/api/temperature/calibration':
                posts.append(urllib.parse.parse_qs(r.request.post_data or ''))
                r.fulfill(content_type='text/plain', body='Сохранено')
            else:
                r.fulfill(status=503, body='{}')
        page.route('**/*', route)
        page.goto('http://busa-ui13.test/')
        page.wait_for_function('document.documentElement.dataset.viewportReady==="1"&&connected&&!pollBusy')
        page.locator('[data-page="5"]').click()
        page.evaluate('settingsTab(0)')
        page.wait_for_timeout(80)
        assert page.locator('.vpPager').is_hidden(), (width, height, 'calibration split across pages')
        def contained(selector):
            result = page.locator(selector).evaluate_all('''es => {
                const frame=document.querySelector('main').getBoundingClientRect();
                return es.map(e=>{const r=e.getBoundingClientRect();return {id:e.id,top:r.top,bottom:r.bottom,frameBottom:frame.bottom,visible:!!r.width&&!!r.height,
                    inside:r.top>=frame.top-1&&r.bottom<=frame.bottom+1&&r.left>=0&&r.right<=innerWidth+1};});
            }''')
            if not result or not all(x['visible'] and x['inside'] for x in result):
                page.screenshot(path=str(shots / ('failure-'+str(width)+'x'+str(height)+'.png')))
                raise AssertionError((width, height, result))
        contained('[id^="calOffset"],#tempCalSave')
        fields = page.locator('[id^="calOffset"]').evaluate_all('(es)=>es.map(e=>{const r=e.getBoundingClientRect();return [r.x,r.width]})')
        if width < height:
            assert max(x[0] for x in fields)-min(x[0] for x in fields)<1, fields
            assert max(x[1] for x in fields)-min(x[1] for x in fields)<1, fields
        for i, value in [(0, '0.1'), (2, '-0.2'), (1, '1.3'), (3, '-1.4')]:
            page.locator('#calOffset'+str(i)).fill(value)
        page.locator('#tempCalSave').click()
        page.wait_for_function('document.querySelector("#tempCalResult").textContent.includes("Сохранено")')
        assert len(posts) == 1 and posts[0] == {'confirm':['YES'],'offset0':['0.1'],'offset1':['1.3'],'offset2':['-0.2'],'offset3':['-1.4']}, posts
        if width == 390 and height == 744:
            page.screenshot(path=str(shots / 'calibration-390x744.png'))
            page.locator('#calOffset0').focus()
            page.set_viewport_size({'width': 390, 'height': 360})
            page.wait_for_timeout(80)
            assert page.locator('.vpPager').is_hidden()
            contained('[id^="calOffset"],#tempCalSave')
            page.set_viewport_size({'width': 390, 'height': 744})
        page.evaluate('settingsTab(1)')
        page.wait_for_function('document.getElementById("net").options.length===2')
        page.wait_for_timeout(80)
        assert page.locator('.vpPager').is_hidden()
        contained('#net,#pass,#wifiRetry,#save,#disconnectWifi')
        text = page.locator('.settingCard[data-stab="1"]').inner_text()
        for forbidden in ['SSID', 'вручную', 'с интернетом', 'НЕ ПОДКЛЮЧЕНО', 'Проверь пароль']:
            assert forbidden not in text, text
        assert page.locator('#manualSsid,#wstatus').count() == 0
        if width == 390 and height == 744:
            page.screenshot(path=str(shots / 'wifi-390x744.png'))
        page.locator('[data-page="0"]').click()
        page.wait_for_timeout(80)
        assert page.locator('.rigSheen').evaluate('(e)=>getComputedStyle(e).display') == 'none'
        assert page.locator('.rigCoolantGlow').first.evaluate('(e)=>getComputedStyle(e).animationName') == 'none'
        page.evaluate('''()=>{
          window.savedThreshold=document.querySelector('#homeSet0 span');
          window.savedRig=document.querySelector('.approvedRigArt');
          window.rigWrites=0;
          new MutationObserver(rs=>{window.rigWrites+=rs.length}).observe(document.querySelector('.approvedDashboard'),{subtree:true,attributes:true,attributeFilter:['style']});
        }''')
        for _ in range(5):
            page.evaluate('update()')
        page.wait_for_timeout(80)
        assert page.evaluate('savedThreshold===document.querySelector("#homeSet0 span")&&savedRig===document.querySelector(".approvedRigArt")&&rigWrites===0')
        bubble = page.locator('.boilerBoil i').first
        before = bubble.evaluate('(e)=>getComputedStyle(e).transform')
        page.wait_for_timeout(180)
        assert before != bubble.evaluate('(e)=>getComputedStyle(e).transform')
        assert page.locator('.collectionSurface').evaluate('(e)=>getComputedStyle(e).animationName') == 'busaSurface'
        assert not page.evaluate('document.documentElement.scrollWidth>innerWidth+1||document.documentElement.scrollHeight>innerHeight+1||scrollY!==0')
        if width == 390 and height == 744:
            page.screenshot(path=str(shots / 'home-390x744.png'))
        assert not errors, errors
        page.close()
    browser.close()
print('PASS UI13: seven mobile/landscape sizes; aligned four-offset form and save on one screen; Wi-Fi picker without manual SSID/duplicate status; steady apparatus, preserved boiling and liquid animations; polling preserves DOM and rig layout')
