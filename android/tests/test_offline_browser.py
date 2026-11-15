"""Exercise the actual packaged offline HTML; not an Android device claim."""
from pathlib import Path
import json, os, shutil, urllib.parse
from playwright.sync_api import sync_playwright

root=Path(__file__).resolve().parents[2]
assets=root/'android/app/src/main/assets/local'
html=(assets/'index.html').read_text()
fixture=json.loads((root/'tests/fixtures/ui10-status.json').read_text())
online=False;posts=[];errors=[]
with sync_playwright() as p:
    browser=p.chromium.launch(executable_path=os.environ.get('BUSA_CHROMIUM') or shutil.which('chromium'),args=['--no-sandbox'])
    page=browser.new_page(viewport={'width':390,'height':780},is_mobile=True,has_touch=True,locale='ru-RU')
    page.on('pageerror',lambda error:errors.append(str(error)))
    def route(r):
        path=urllib.parse.urlsplit(r.request.url).path
        if r.request.method=='POST':posts.append(path)
        if path=='/':r.fulfill(content_type='text/html',body=html)
        elif path in ['/calculator.js','/calculator.css']:
            r.fulfill(content_type='application/javascript' if path.endswith('.js') else 'text/css',body=(assets/path[1:]).read_bytes())
        elif online and path=='/api/status':r.fulfill(content_type='application/json',body=json.dumps(fixture))
        else:r.fulfill(status=503,body='{}')
    page.route('**/*',route);page.goto('http://192.168.4.1/')
    page.wait_for_function('document.documentElement.dataset.viewportReady==="1"&&!pollBusy')
    assert page.locator('#t0').inner_text()=='Н/Д'
    assert page.locator('#t2').inner_text()=='Н/Д'
    assert page.locator('.approvedRigArt').evaluate('e=>e.complete&&e.naturalWidth>0')
    assert page.locator('.hasCooling,.hasCollectionFlow').count()==0
    page.locator('[data-page="7"]').click()
    for name,value in [('v','1'),('from','96'),('to','40')]:page.locator('#calc-0-'+name).fill(value)
    assert '1,47' in page.locator('#calc-panel-0 [data-value="water"]').inner_text()
    page.locator('[data-page="5"]').click();page.wait_for_timeout(80)
    assert page.locator('#settingsMenu>.approvedMenuItem:visible').count()==9
    assert page.locator('.vpPager').count()==0
    assert all(path=='/api/time/sync' for path in posts),posts
    online=True;page.evaluate('update()');page.wait_for_function('connected&&!pollBusy')
    page.locator('[data-page="0"]').click()
    assert page.locator('#t0').inner_text()=='26,4°C'
    # Keep the browser's existing automatic clock sync, without new commands.
    assert all(path=='/api/time/sync' for path in posts),posts
    assert not page.evaluate('scrollY||document.documentElement.scrollWidth>innerWidth+1||document.documentElement.scrollHeight>innerHeight+1')
    assert not errors,errors
    browser.close()
print('PASS packaged offline UI: real apparatus, Н/Д without ESP, calculator works, 9 settings, polling recovery, existing clock sync only, no invented values')
