"""Local web Wi-Fi UI against an HTTP fixture, never a physical ESP32 test."""
from pathlib import Path
import threading,http.server,socketserver,os,json,urllib.parse
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1];html=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text().split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
state={'case':'found','starts':0,'posts':[]}
class Handler(http.server.SimpleHTTPRequestHandler):
 def __init__(self,*a,**kw):super().__init__(*a,directory=str(root),**kw)
 def log_message(self,*a):pass
 def do_GET(self):
  if self.path=='/local.html':self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.end_headers();self.wfile.write(html.encode());return
  if self.path.startswith('/api/wifi/scan'):
   if 'start=1' in self.path:state['starts']+=1
   data={'scanning':False,'networks':[],'error':'','diagnostic':{'start_result':0,'event_status':0}}
   if state['case']=='found':data['networks']=[{'ssid':'Тестовая сеть','rssi':-40},{'ssid':'Тестовая сеть','rssi':-50}]
   elif state['case']=='error':data['error']='scan_start_failed';data['diagnostic']['start_result']=12294
   else:data['error']='no_networks'
   self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps(data).encode());return
  if self.path.startswith('/api/'):
   self.send_response(503);self.end_headers();return
  super().do_GET()
 def do_POST(self):
  data=self.rfile.read(int(self.headers.get('Content-Length','0')))
  if self.path=='/api/wifi':
   state['posts'].append(urllib.parse.parse_qs(data.decode()));self.send_response(200);self.send_header('Content-Type','text/plain; charset=utf-8');self.end_headers();self.wfile.write('Подключение запускается'.encode())
  else:self.send_response(503);self.end_headers()
server=socketserver.TCPServer(('127.0.0.1',0),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
with sync_playwright() as p:
 engine=os.environ.get('BUSA_TEST_BROWSER','chromium')
 if engine=='webkit':browser=p.webkit.launch()
 else:browser=p.chromium.launch(executable_path='/usr/bin/chromium' if Path('/usr/bin/chromium').exists() and not os.environ.get('GITHUB_ACTIONS') else p.chromium.executable_path,args=['--no-sandbox'])
 context=browser.new_context(viewport={'width':390,'height':844},is_mobile=True,has_touch=True,locale='ru-RU');page=context.new_page();errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 page.goto(f'http://127.0.0.1:{server.server_address[1]}/local.html');page.locator('[data-page="5"]').click();page.locator('[data-stab-btn="1"]').click()
 page.wait_for_function("Array.from(document.getElementById('net').options).filter(x=>x.value==='Тестовая сеть').length===1")
 page.locator('#net').select_option(label='Тестовая сеть');page.locator('#pass').fill('example123');page.locator('#save').click();page.wait_for_function("document.getElementById('pass').value===''");assert state['posts'][-1]['ssid']==['Тестовая сеть']
 state['case']='error';page.locator('#wifiRetry').click();page.wait_for_function("document.getElementById('response').textContent.includes('12294')");assert page.locator('#manualSsid').is_visible();assert 'Ошибка' not in page.locator('#net').inner_text() # picker is usable, error is separate
 page.locator('#manualSsid').fill('Тестовая сеть');page.locator('#pass').fill('example123');page.locator('#save').click();page.wait_for_function("document.getElementById('pass').value===''");assert len(state['posts'])==2 and state['posts'][-1]['ssid']==['Тестовая сеть']
 page.locator('#manualSsid').fill('я'*17);page.locator('#pass').fill('example123');page.locator('#save').click();assert len(state['posts'])==2;assert '32' in page.locator('#response').inner_text()
 state['case']='empty';page.locator('#wifiRetry').click();page.wait_for_function("document.getElementById('response').textContent.includes('2,4')");assert page.locator('#manualSsid').is_visible();assert not page.locator('#wifiRetry').is_disabled();assert state['starts']==3
 assert page.evaluate('document.documentElement.scrollWidth<=innerWidth');assert not errors,errors
 context.close();browser.close()
server.shutdown();print('PASS '+engine+' Wi-Fi fixture: Cyrillic SSID/dedup, explicit retry, native error details, manual connection after scan failure, UTF-8 byte validation, no JS exception/overflow')
