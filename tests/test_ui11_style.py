"""Compare approved UI9 controls with fitted production UI; never stretch charts/menu."""
from pathlib import Path
import re,json,urllib.parse,shutil
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1]
html=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text().split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
legacy=re.sub(r'/\* BUSA_VIEWPORT_CSS_BEGIN \*/.*?/\* BUSA_VIEWPORT_CSS_END \*/','',html,flags=re.S)
legacy=re.sub(r'<script>\s*/\* BUSA_VIEWPORT_JS_BEGIN \*/.*?</script>','',legacy,flags=re.S)
fixture=(root/'tests/fixtures/ui10-status.json').read_text()
selectors=['.appHeader','.appHeader h1','.clockBar','.netPill','.appTabs button','.appTabs .uiIcon','.approvedPageTitle','.chartTabs button','.chartRanges button','.approvedMenuItem']
properties=['fontSize','paddingTop','paddingRight','paddingBottom','paddingLeft','minHeight','gap','borderRadius']
with sync_playwright() as p:
 b=p.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox'])
 for w,h in [(320,568),(390,744),(430,932)]:
  shots=[]
  for body in [legacy,html]:
   page=b.new_page(viewport={'width':w,'height':h})
   def route(r):
    path=urllib.parse.urlsplit(r.request.url).path
    if path=='/':r.fulfill(content_type='text/html',body=body)
    elif path=='/api/status':r.fulfill(content_type='application/json',body=fixture)
    elif path=='/api/web-events':r.fulfill(content_type='application/json',body='{"boot_id":1,"events":[]}')
    elif path in ['/calculator.js','/calculator.css']:r.fulfill(body=(root/path[1:]).read_bytes())
    else:r.fulfill(status=503,body='{}')
   page.route('**/*',route);page.goto('http://busa.style/');page.wait_for_timeout(200)
   styles=page.evaluate('''arg=>Object.fromEntries(arg.selectors.map(s=>{const e=document.querySelector(s),c=getComputedStyle(e);return [s,Object.fromEntries(arg.properties.map(p=>[p,c[p]]))]}))''',{'selectors':selectors,'properties':properties})
   # UI14 permits the settings rows to shrink to fit all nine items. Every
   # other menu property and all header/navigation/chart styles stay pinned.
   styles['.approvedMenuItem'].pop('minHeight')
   shots.append(styles)
   if body==html:
    pills=page.locator('.netPill').evaluate_all('(es)=>es.map(e=>({height:e.getBoundingClientRect().height,line:parseFloat(getComputedStyle(e).lineHeight),padding:parseFloat(getComputedStyle(e).paddingTop)+parseFloat(getComputedStyle(e).paddingBottom)}))')
    assert all(x['height']<=x['line']+x['padding']+3 for x in pills),pills
    page.locator('[data-page="3"]').click();page.wait_for_timeout(100)
    canvas=page.locator('#chartCanvas').bounding_box();assert canvas['height']<=canvas['width']*440/640+2,canvas
    page.locator('[data-page="5"]').click();page.wait_for_timeout(100)
    menu=page.locator('#settingsMenu').bounding_box();items=page.locator('#settingsMenu>.approvedMenuItem').evaluate_all('(es)=>es.filter(e=>e.getBoundingClientRect().height>0).reduce((s,e)=>s+e.getBoundingClientRect().height,0)')
    if page.locator('.vpPager').is_hidden():assert abs(menu['height']-items)<5,(menu,items)
   page.close()
  assert shots[0]==shots[1],(w,[(k,shots[0][k],shots[1][k]) for k in selectors if shots[0][k]!=shots[1][k]])
 b.close()
print('PASS UI11: approved header, connection pills, navigation and tab/menu styles unchanged; single-line connections; chart never expanded past natural ratio; menu has no forced empty height')
