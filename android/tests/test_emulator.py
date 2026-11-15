"""Run the built APK in an Android emulator against a local ESP32 HTTP fixture.

Requires adb, a booted emulator, a DEBUG APK and Playwright. No real ESP32.
"""
from pathlib import Path
import copy, http.server, json, re, socketserver, subprocess, sys, threading, time, urllib.parse
from playwright.sync_api import sync_playwright

root=Path(__file__).resolve().parents[2]
apk=Path(sys.argv[1]).resolve()
out=root/'android/out/emulator';out.mkdir(parents=True,exist_ok=True)
fixture=json.loads((root/'tests/fixtures/ui10-status.json').read_text())
fixture['collection']={'sensor_present':True,'ml_per_min':24,'total_ml':1234,'duration_ms':3600000}
state={'offline':False,'tick':0,'posts':[]}
html=(root/'android/app/src/main/assets/local/index.html').read_bytes()
def adb(*args,data=None):
    return subprocess.run(['adb',*args],input=data,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout
class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def do_GET(self):
        path=urllib.parse.urlsplit(self.path).path
        body=b'{}';kind='application/json';code=200
        if state['offline']:code=503
        elif path=='/':body=html;kind='text/html; charset=utf-8'
        elif path in ['/calculator.js','/calculator.css']:
            body=(root/path[1:]).read_bytes();kind='application/javascript' if path.endswith('.js') else 'text/css'
        elif path=='/api/status':
            f=copy.deepcopy(fixture);state['tick']+=1;f['uptime_ms']=10000+state['tick']*2000
            f['water']['total_l_estimate']=12.32+state['tick']*.04;body=json.dumps(f).encode()
        elif path=='/api/web-events':body=b'{"boot_id":1,"events":[]}'
        elif path=='/api/chart-history':
            f={'ok':True,'minutes':5,'temperature':[[v]*36 for v in [26.4,78.6,96.2,80]],'collection_ml_min':[24]*36,'cooling_lph':[72]*36,'collection_sensor':True,'cooling_calibrated':True}
            body=json.dumps(f).encode()
        else:code=503
        self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length','0')));state['posts'].append(self.path)
        self.send_response(200);self.send_header('Content-Length','2');self.end_headers();self.wfile.write(b'OK')
class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address=True;daemon_threads=True
server=Server(('127.0.0.1',18080),Handler)
threading.Thread(target=server.serve_forever,daemon=True).start()
try:
    adb('wait-for-device');adb('install','-r',str(apk));adb('reverse','tcp:18080','tcp:18080')
    adb('shell','am','force-stop','ru.busa.monitor')
    adb('shell','run-as','ru.busa.monitor','mkdir','-p','shared_prefs')
    prefs=b'<?xml version="1.0" encoding="utf-8"?><map><string name="address">http://127.0.0.1:18080</string></map>'
    adb('shell','run-as','ru.busa.monitor','tee','shared_prefs/connection.xml',data=prefs)
    adb('shell','am','start','-W','-n','ru.busa.monitor/.MainActivity')
    socket=None
    for _ in range(60):
        sockets=adb('shell','cat','/proc/net/unix').decode()
        found=re.findall(r'@(webview_devtools_remote_\d+)',sockets)
        if found:socket=found[-1];break
        time.sleep(1)
    assert socket,'WebView debug socket not found'
    adb('forward','tcp:9222','localabstract:'+socket)
    with sync_playwright() as p:
        browser=p.chromium.connect_over_cdp('http://127.0.0.1:9222')
        page=browser.contexts[0].pages[0];errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
        page.wait_for_function('typeof connected!=="undefined"&&connected&&!pollBusy&&document.documentElement.dataset.viewportReady==="1"',timeout=60000)
        assert page.locator('#t0').inner_text()=='26,4°C'
        assert page.locator('.approvedRigArt').evaluate('e=>e.complete&&e.naturalWidth>0')
        for section,name in [(0,'home'),(1,'temperatures'),(2,'water'),(3,'graphs'),(7,'calculator'),(5,'settings')]:
            page.locator('[data-page="'+str(section)+'"]').click();page.wait_for_timeout(150)
            assert not page.evaluate('scrollY||document.documentElement.scrollWidth>innerWidth+1||document.documentElement.scrollHeight>innerHeight+1'),name
            assert page.locator('.vpPager').count()==0
            (out/(name+'.png')).write_bytes(adb('exec-out','screencap','-p'))
        assert page.locator('#settingsMenu>.approvedMenuItem:visible').count()==9
        page.locator('[data-stab-btn="5"]').click()
        page.locator('#firmwareFile').click()
        # A native document picker must open; cancelling must return to the app.
        time.sleep(2)
        activity=adb('shell','dumpsys','activity','activities').decode()
        assert 'documentsui' in activity or 'DocumentsActivity' in activity,'No Android document picker'
        adb('shell','input','keyevent','4');page.wait_for_timeout(500)
        adb('shell','input','keyevent','4');page.wait_for_function('settingsActive===-1')
        adb('shell','input','keyevent','4');page.wait_for_function('active===0')
        # Test the actual native Back handling, not a replacement HTML toolbar.
        page.locator('[data-page="7"]').click()
        for key,value in [('v','1'),('from','96'),('to','40')]:page.locator('#calc-0-'+key).fill(value)
        assert '1,47' in page.locator('#calc-panel-0 [data-value="water"]').inner_text()
        adb('shell','input','keyevent','4');page.wait_for_timeout(300)
        if page.evaluate('active')!=0:adb('shell','input','keyevent','4') # First Back can dismiss Android's keyboard.
        page.wait_for_function('active===0')
        state['offline']=True
        for _ in range(3):page.evaluate('update()');page.wait_for_timeout(100)
        page.wait_for_function('!connected&&!pollBusy')
        assert page.locator('#t0').inner_text()=='Н/Д'
        assert not page.locator('#rigArtStage').evaluate('e=>e.classList.contains("hasCooling")||e.classList.contains("hasCollection")')
        assert all(x=='/api/time/sync' for x in state['posts']),state['posts']
        assert not errors,errors
        browser.close()
    (out/'RESULT.txt').write_text('PASS real Android APK on emulator: six screens, exact live UI, document picker/cancel, native Back, calculator, connection loss, no outer scrolling; all data from HTTP fixtures, no physical ESP32\n')
    print((out/'RESULT.txt').read_text())
finally:
    server.shutdown()
    (out/'logcat.txt').write_bytes(adb('logcat','-d','-t','2500'))
