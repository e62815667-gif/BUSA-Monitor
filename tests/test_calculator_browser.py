"""Browser UI tests on a local HTTP fixture, never a claim of real ESP/iPhone testing."""
from pathlib import Path
import threading,http.server,socketserver,socket,os,re,json
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
origin_offline=False
local=(sketch/'BUSA_Main.h').read_text().split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
class Handler(http.server.SimpleHTTPRequestHandler):
 def __init__(self,*a,**kw):super().__init__(*a,directory=str(root),**kw)
 def log_message(self,*args):pass
 def do_GET(self):
  if origin_offline:
   self.connection.shutdown(socket.SHUT_RDWR);self.connection.close();return
  if self.path=='/local.html':
   self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.end_headers();self.wfile.write(local.encode())
  elif self.path.startswith('/api/'):
   self.send_response(503);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(b'{"error":"test fixture: no ESP32"}')
  else:super().do_GET()
 def do_POST(self):self.send_response(503);self.end_headers()
server=socketserver.TCPServer(('127.0.0.1',0),Handler);threading.Thread(target=server.serve_forever,daemon=True).start();url=f'http://127.0.0.1:{server.server_address[1]}'
artifacts=Path(os.environ.get('BUSA_BROWSER_ARTIFACTS','/tmp/busa-calculator-browser'));artifacts.mkdir(parents=True,exist_ok=True)
with sync_playwright() as p:
 browser=p.webkit.launch() if os.environ.get('BUSA_TEST_BROWSER')=='webkit' else p.chromium.launch(executable_path=os.environ.get('BUSA_CHROMIUM', '/usr/bin/chromium' if Path('/usr/bin/chromium').exists() and not os.environ.get('GITHUB_ACTIONS') else p.chromium.executable_path),args=['--no-sandbox'])
 for local_app in [False,True]:
  for width,height in [(390,844),(393,852)]:
   context=browser.new_context(viewport={'width':width,'height':height},is_mobile=True,has_touch=True,locale='ru-RU')
   page=context.new_page();errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
   path='/local.html' if local_app else '/index.html';page.goto(url+path);page.wait_for_selector('#busaCalculator .calcPanel',state='attached')
   page.locator('[data-page="7"]' if local_app else '[data-tab="calculator"]').click()
   page.locator('#calc-0-v').fill('1');page.locator('#calc-0-from').fill('96');page.locator('#calc-0-to').fill('40');page.locator('#calc-panel-0 button[type=submit]').click()
   assert '1,47' in page.locator('#calc-panel-0 [data-value=water]').inner_text()
   page.locator('#calc-0-to').fill('0');assert page.locator('#calc-panel-0 .calcError').inner_text();assert page.locator('#calc-panel-0 .calcResult').inner_text()==''
   page.locator('#calc-0-to').fill('40')
   page.locator('#calc-tab-1').click();page.locator('#calc-1-t').fill('20');page.locator('#calc-1-reading').fill('40');assert '40,00' in page.locator('#calc-panel-1 [data-value=strength]').inner_text()
   page.locator('#calc-1-t').fill('30');assert '35,98' in page.locator('#calc-panel-1 [data-value=strength]').inner_text()
   page.locator('#calc-tab-2').click()
   for key,value in [('v','10'),('strength','40'),('out','80'),('head','10'),('tail','10')]:page.locator('#calc-2-'+key).fill(value)
   assert '4,00' in page.locator('#calc-panel-2 [data-value=product]').inner_text()
   assert '0,40' in page.locator('#calc-panel-2 [data-value=heads]').inner_text()
   assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'),(local_app,width)
   page.screenshot(path=str(artifacts/f'{"local" if local_app else "online"}-{width}.png'),full_page=True)
   page.reload();page.wait_for_selector('#busaCalculator .calcPanel',state='attached')
   assert page.locator('#calc-0-from').input_value()=='96'
   page.locator('[data-page="7"]' if local_app else '[data-tab="calculator"]').click();page.locator('#calc-panel-0 button[type=reset]').click();page.wait_for_function("document.getElementById('calc-0-v').value===''")
   page.evaluate("document.documentElement.dataset.theme='dark'");assert page.locator('#calc-panel-0 button[type=submit]').is_visible()
   if not local_app:
    page.evaluate('navigator.serviceWorker.ready.then(()=>true)');page.reload();page.wait_for_function('!!navigator.serviceWorker.controller')
    # WebKit's Playwright setOffline causes an internal navigation error before
    # service worker fallback. Drop origin TCP responses instead; cached navigation
    # and both JS/CSS must still load successfully, with no server response.
    if os.environ.get('BUSA_TEST_BROWSER')=='webkit':origin_offline=True
    else:context.set_offline(True)
    page.reload();page.wait_for_selector('#busaCalculator .calcPanel',state='attached');page.locator('[data-tab="calculator"]').click()
    page.locator('#calc-0-v').fill('1');page.locator('#calc-0-from').fill('60');page.locator('#calc-0-to').fill('40');assert page.locator('#calc-panel-0 [data-value=water]').inner_text()
    origin_offline=False
   # API fixture responses are handled by the existing app; JS exceptions are failures.
   assert not errors,errors
   context.close()
 browser.close()
server.shutdown()
print('PASS '+os.environ.get('BUSA_TEST_BROWSER','chromium')+' browser: local/online, iPhone 12/16 viewport, decimal comma, 3 calculators, validation, persistence/reset, dark mode, PWA offline, no horizontal overflow or JS exceptions')
